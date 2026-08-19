#include "ir_receiver.hpp"

#include "adc.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

// hadc1 / htim2 由 CubeMX 生成的 adc.c / tim.c 定义，头文件已 extern "C" 声明

void IrReceiver::start(Signal& sig)
{
    sig.clear();
    sig_ = &sig;
    state_ = CaptureState::WaitingEdge;

    // IR(PA0/ADC1_CH0) 用单次转换模式：由 readAdc 显式 SWSTART 触发，
    // 状态机干净、不会 OVR 停转，也不受上一次命令残留状态影响。
    ADC1->CR2 &= ~ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);        // 使能 ADC（含内部稳定延时）

    // 空闲基准：连读若干次取平均，作为首个边沿判定的前值
    uint32_t sum = 0;
    for (int i = 0; i < 8; ++i)
        sum += readAdc();
    prevSample_ = static_cast<uint16_t>(sum / 8);
    // 空闲线电平：HS0038(红外)空闲=高，以半量程为界判定。
    // 空闲结束判定与存储极性都以此为准。
    idleLevel_ = (prevSample_ > 2048);

    startTick_ = HAL_GetTick();
}

uint16_t IrReceiver::readAdc()
{
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

    return pollIr(sample, diff);
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
            // space(-)。HS0038(空闲=高)等价于原取反写法，保证数组按真实信号存放
            // (载波=高/mark)，与发送侧约定一致。
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
