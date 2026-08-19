#include "ir_receiver.hpp"

#include "adc.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

// hadc1 / htim2 由 CubeMX 生成的 adc.c / tim.c 定义，头文件已 extern "C" 声明

void IrReceiver::start(Signal& sig, uint8_t adcChannel)
{
    sig.clear();
    sig_ = &sig;
    state_ = CaptureState::WaitingEdge;

    // IR/RF 共用一个 ADC1：先按通道切到对应引脚（通道未变化时为 no-op）。
    // 仅在通道变化时 HAL_ADC_Stop + 重配，避免连续模式反复启停的 EOC/OVR 错乱。
    if (!selectChannel(adcChannel))
    {
        state_ = CaptureState::Timeout;   // 通道配置失败，直接判超时结束
        return;
    }

    // IR 用单次转换模式：由 readAdc 显式 SWSTART 触发，状态机干净、不会 OVR 停转，
    // 也不受上一次命令残留状态影响（连续模式反复启停会 EOC 卡死 → xx01 TIMEOUT）。
    // RF(PA4) 改用连续模式：单次 SWSTART 在空闲线上会读出交替假边沿（实测不按遥控
    // 也能填满 510 段）；raw/dbg 的连续模式采同一条线却稳定干净（3~35 edges/秒），
    // 并能采到干净的 400/1200μs 真实帧，故 RF 走连续模式。
    // 注意：连续模式必须配合 3 周期采样（见 selectChannel），84 周期会重新诱发假边沿。
    ADC1->CR2 &= ~ADC_CR2_CONT;
    contMode_ = false;
    if (adcChannel == 4)
    {
        ADC1->CR2 |= ADC_CR2_CONT;
        contMode_ = true;
    }
    HAL_ADC_Start(&hadc1);        // 使能 ADC（含内部稳定延时）

    // 空闲基准：连读若干次取平均，作为首个边沿判定的前值
    uint32_t sum = 0;
    for (int i = 0; i < 8; ++i)
        sum += readAdc();
    prevSample_ = static_cast<uint16_t>(sum / 8);
    // 空闲线电平：HS0038(红外)空闲=高、远-R1(射频)空闲=低，以半量程为界判定。
    // 空闲结束判定与存储极性都以此为准，做到接收模块输出极性无关。
    idleLevel_ = (prevSample_ > 2048);

    startTick_ = HAL_GetTick();
}

bool IrReceiver::selectChannel(uint8_t channel)
{
    if (channel == channel_)
        return true;
    HAL_ADC_Stop(&hadc1);
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = (channel == 4) ? ADC_CHANNEL_4 : ADC_CHANNEL_0;
    cfg.Rank         = 1;
    // 采样时间统一 3 周期：实测 RF(PA4) 拉长到 84 周期(≈4μs)反而让空闲线采出假边沿
    //（长采样期间 S&H 经 10k 串联电阻对 R1 DATA 强拉电流，诱发振荡）。只有 3 周期 +
    // 连续模式（即 raw 的配置）才是稳定干净的组合。IR(PA0, HS0038 低阻)同样 3 周期。
    cfg.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK)
        return false;
    channel_ = channel;
    // 与通道绑定采样模式：RF(PA4)=连续、IR(PA0)=单次，保证 readAdc() 走对应分支。
    // 诊断命令只切通道不调 start()，故这里必须同步维护 contMode_。
    contMode_ = (channel == 4);
    return true;
}

uint16_t IrReceiver::readAdc()
{
    if (contMode_)
    {
        // 连续模式（RF/PA4）：每次读前清 OVR/EOC，再等下一次转换完成。
        // 关键：若长时间未读（如 adcmon 每 40ms 打印阻塞 ~3ms），连续模式会
        // 触发 overrun，数据寄存器随即冻结——此后读到的永远是旧值(实测恒定 3984)。
        // 清标志是唯一解冻手段，故每次读都清，不能只在失败路径清。
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR | ADC_FLAG_EOC);
        if (HAL_ADC_PollForConversion(&hadc1, 1) != HAL_OK)
        {
            __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR | ADC_FLAG_EOC);
            HAL_ADC_PollForConversion(&hadc1, 1);
        }
        return static_cast<uint16_t>(HAL_ADC_GetValue(&hadc1));
    }

    // 单次模式（IR/PA0）：每次采样显式软件触发一次转换，等 EOC 后读 DR。
    // 这是最稳的用法——不存在连续模式下的 OVR 停转 / EOC 卡死，
    // 也不依赖上一次命令留下的 ADC 运行状态。
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR | ADC_FLAG_EOC);
    ADC1->CR2 |= ADC_CR2_SWSTART;
    if (HAL_ADC_PollForConversion(&hadc1, 1) != HAL_OK)
    {
        // 兜底：单次触发偶发失败时清 OVR 重试一次，避免读到陈旧 DR
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR | ADC_FLAG_EOC);
        ADC1->CR2 |= ADC_CR2_SWSTART;
        HAL_ADC_PollForConversion(&hadc1, 1);
    }
    return static_cast<uint16_t>(HAL_ADC_GetValue(&hadc1));
}

CaptureState IrReceiver::poll()
{
    if (state_ != CaptureState::WaitingEdge &&
        state_ != CaptureState::WaitingGap &&
        state_ != CaptureState::Capturing)
        return state_;

    // 整个录制会话 15s 总超时（unsigned 相减天然处理回绕）
    if (HAL_GetTick() - startTick_ >= kTimeoutMs)
    {
        state_ = CaptureState::Timeout;
        return state_;
    }

    const uint16_t sample = readAdc();
    const uint16_t diff   = (sample > prevSample_) ? (sample - prevSample_) : (prevSample_ - sample);

    // IR(HS0038, 空闲=高/干净) 走首边沿即录；RF(远-R1, 空闲噪声) 走帧级捕获。
    return (channel_ == 4) ? pollRf(sample, diff) : pollIr(sample, diff);
}

// ---- IR 捕获（HS0038）：首个边沿即进入录制，空闲 100ms 判定结束 ----
CaptureState IrReceiver::pollIr(uint16_t sample, uint16_t diff)
{
    if (state_ == CaptureState::WaitingEdge)
    {
        // 首个边沿：确定起始电平，清零 TIM2，进入录制
        if (diff > kEdgeThreshold)
        {
            levelHigh_ = (sample > prevSample_);
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            state_ = CaptureState::Capturing;
        }
        prevSample_ = sample;
        return state_;
    }

    // ---- Capturing (IR) ----
    if (diff > kEdgeThreshold)
    {
        const bool     newLevel = (sample > prevSample_);
        const uint32_t duration = __HAL_TIM_GET_COUNTER(&htim2);

        // 去抖：一条真实跳变在过渡期可能被 ADC 采出 2~3 个连续假边沿，
        // 产生 1~3μs 的垃圾段（raw 抓取中大量 H1/L1 即此）。段长 < kMinSegUs
        // 视为假边沿：不写入、不翻转电平，仅清零计时继续跟踪当前电平。
        if (duration < kMinSegUs)
        {
            __HAL_TIM_SET_COUNTER(&htim2, 0);
        }
        else
        {
            // 存储极性以空闲电平为准：载波段(非空闲电平)存为 mark(+)、空闲段存为
            // space(-)。HS0038(空闲=高)等价于原取反写法；远-R1(空闲=低)自动翻转，
            // 两者都保证数组按真实信号存放(载波=高/mark)，与发送侧约定一致。
            if (!sig_->append(levelHigh_ != idleLevel_, duration))
            {
                state_ = CaptureState::BufferFull;
                return state_;
            }
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            levelHigh_ = newLevel;
        }
    }
    else
    {
        const uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim2);

        // 空闲结束判定：只对「空闲电平」段计时（HS0038 空闲=高）。
        // 载波段(非空闲电平，如 NEC 9ms 引导脉冲)持续再久也不视为结束，不会被切断。
        if (levelHigh_ == idleLevel_ && cnt >= kIdleEndUs)
        {
            // 尾部空闲段不入信号：回放停在最后一个真实脉冲
            state_ = CaptureState::Done;
            return state_;
        }

        // 防御性兜底：TIM2 现为 32 位(CNT 上限≈4295s)，kMaxSegUs(≈2147s)仍先于
        // CNT 翻转触发，超长电平段被拆包。真实 IR 段远低于此，正常不会走到。
        if (cnt >= Signal::kMaxSegUs)
        {
            if (!sig_->append(levelHigh_ != idleLevel_, Signal::kMaxSegUs))
            {
                state_ = CaptureState::BufferFull;
                return state_;
            }
            __HAL_TIM_SET_COUNTER(&htim2, 0);
        }
    }

    prevSample_ = sample;
    return state_;
}

// ---- RF 捕获（远-R1）：帧间隔界定 + 段长校验，抗空闲噪声 ----
CaptureState IrReceiver::pollRf(uint16_t sample, uint16_t diff)
{
    // WaitingGap：已确认一个帧间隔(连续空闲≥kFrameGapUs)，等它结束的边沿 = 帧起点
    if (state_ == CaptureState::WaitingGap)
    {
        if (diff > kEdgeThreshold)
        {
            levelHigh_ = (sample > prevSample_);
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            state_ = CaptureState::Capturing;   // 帧起始边沿，开始记录
        }
        prevSample_ = sample;
        return state_;
    }

    if (state_ == CaptureState::WaitingEdge)
    {
        if (diff > kEdgeThreshold)
        {
            levelHigh_ = (sample > prevSample_);
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            // RF 不立即进入录制：继续等「连续空闲≥kFrameGapUs」作为帧边界，
            // 避免空闲噪声的随机短段直接触发录制。
        }
        else
        {
            const uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim2);
            // 任一段(不分电平)持续 ≥kFrameGapUs = 帧间隔/帧边界。刻意不依赖
            // idleLevel_：远-R1 空闲噪声下空闲基准可能误判，而真实帧间隔(~10ms低)
            // 与噪声长段(~13ms)都 >8ms，这里先放行，真伪交给 validateFrame 过滤。
            if (cnt >= kFrameGapUs)
                state_ = CaptureState::WaitingGap;   // 边界已确认，等帧起点边沿
        }
        prevSample_ = sample;
        return state_;
    }

    // ---- Capturing (RF) ----
    if (diff > kEdgeThreshold)
    {
        const bool     newLevel = (sample > prevSample_);
        const uint32_t duration = __HAL_TIM_GET_COUNTER(&htim2);

        // 去抖同 IR：过渡期假边沿丢弃，只跟踪真实跳变
        if (duration < kMinSegUs)
        {
            __HAL_TIM_SET_COUNTER(&htim2, 0);
        }
        else
        {
            // RF 存储极性：实测远-R1 是反极性（载波=输出低、无载波=输出高），
            // 所以存 !levelHigh_：R1 输出低=载波存在=mark(+)。之前文档/代码误以为
            // 载波=高，实测 rftst 回环 RX 与 TX 精确反相后修正。
            if (!sig_->append(!levelHigh_, duration))
            {
                state_ = CaptureState::BufferFull;
                return state_;
            }
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            levelHigh_ = newLevel;
        }
    }
    else
    {
        const uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim2);

        // 连续同电平段 ≥帧间隔 = 本帧结束（不分电平，见 WaitingEdge 处注释）。
        // 帧内码元段最长为 sync mark(~1.6ms)，远小于 8ms，不会被误判为帧尾。
        // 帧间隔段不入信号。
        if (cnt >= kFrameGapUs)
        {
            if (validateFrame())
            {
                state_ = CaptureState::Done;   // 真帧 → 保存
            }
            else
            {
                sig_->clear();                 // 噪声伪帧 → 清空，继续找下一帧
                state_ = CaptureState::WaitingEdge;
            }
            return state_;
        }

        // 防御性兜底：超长段拆包
        if (cnt >= Signal::kMaxSegUs)
        {
            if (!sig_->append(levelHigh_, Signal::kMaxSegUs))
            {
                state_ = CaptureState::BufferFull;
                return state_;
            }
            __HAL_TIM_SET_COUNTER(&htim2, 0);
        }
    }

    prevSample_ = sample;
    return state_;
}

// ---- RF 帧有效性校验：段数与段长范围 + 存在长段(sync/长mark) ----
// 真 EV1527 帧 ≈ sync(1.6ms)+20bit(400/1200μs 交替) ≈ 41 段，全落在 250~5000μs。
// 空闲噪声段长随机分布，大量短段(<250μs)与偶发超长段(>5ms)会命中拒绝条件。
bool IrReceiver::validateFrame() const
{
    const uint32_t n = sig_->length();
    if (n < kMinRfSegs || n > kMaxRfSegs)
        return false;

    bool hasLong = false;
    for (uint32_t i = 0; i < n; ++i)
    {
        const uint32_t us = sig_->us(sig_->at(i));
        if (us < kMinRfSegUs || us > kMaxRfSegUs)
            return false;         // 段长超出真帧范围（噪声微段/超长段）
        if (us >= kMinRfLongSeg)
            hasLong = true;       // 存在 sync/长 mark
    }
    return hasLong;               // 全部短段且无长段 → 纯噪声微段流，拒
}
