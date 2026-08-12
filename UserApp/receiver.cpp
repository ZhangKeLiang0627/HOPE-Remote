#include "receiver.hpp"

#include "adc.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

// hadc1 / htim2 由 CubeMX 生成的 adc.c / tim.c 定义，头文件已 extern "C" 声明

void Receiver::start(Signal& sig, uint8_t adcChannel)
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

    startTick_ = HAL_GetTick();
}

bool Receiver::selectChannel(uint8_t channel)
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
    return true;
}

uint16_t Receiver::readAdc()
{
    if (contMode_)
    {
        // 连续模式（RF/PA4）：转换自动连续进行，读 DR 自动清 EOC，
        // 每次调用等下一次转换完成即可。raw/dbg 已验证该模式下 PA4 读数稳定。
        if (HAL_ADC_PollForConversion(&hadc1, 1) != HAL_OK)
        {
            // 兜底：偶发 OVR 时清标志重试一次
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

CaptureState Receiver::poll()
{
    if (state_ != CaptureState::WaitingEdge && state_ != CaptureState::Capturing)
        return state_;

    // 整个录制会话 15s 总超时（unsigned 相减天然处理回绕）
    if (HAL_GetTick() - startTick_ >= kTimeoutMs)
    {
        state_ = CaptureState::Timeout;
        return state_;
    }

    const uint16_t sample = readAdc();
    const uint16_t diff   = (sample > prevSample_) ? (sample - prevSample_) : (prevSample_ - sample);

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

    // ---- Capturing ----
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
            // 存储时电平取反：levelHigh_ 是 HS0038 解调电平(载波=低)，
            // 数组按真实 IR 信号存放(载波=高/mark)，与发送侧约定一致。
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

        // 空闲结束判定：只对高电平（无载波的空间段）计时。
        // 载波低电平段（如 NEC 9ms 引导脉冲）持续再久也不视为结束，不会被切断。
        if (levelHigh_ && cnt >= kIdleEndUs)
        {
            // 尾部空闲段不入信号：回放停在最后一个真实脉冲
            state_ = CaptureState::Done;
            return state_;
        }

        // 防御性兜底：TIM2 现为 32 位(CNT 上限≈4295s)，kMaxSegUs(≈2147s)仍先于
        // CNT 翻转触发，超长电平段被拆包。真实 IR 段远低于此，正常不会走到。
        //（高电平段早已被空闲判定结束，仅极端超长载波低电平段会到这里。）
        if (cnt >= Signal::kMaxSegUs)
        {
            if (!sig_->append(!levelHigh_, Signal::kMaxSegUs))
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
