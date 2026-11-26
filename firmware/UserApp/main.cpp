#include "common_inc.h"
#include "interface_uart.h"

#include "tim.h"
#include "signal.hpp"
#include "slot_store.hpp"
#include "ir_receiver.hpp"
#include "rf_receiver.hpp"
#include "transmitter.hpp"
#include "ir_transmitter.hpp"
#include "rf_transmitter.hpp"
#include "cli.hpp"
#include "ws2812b.hpp"
#include "user_led.hpp"

static_assert(Signal::kMaxSegments == WaveTraits::kMaxPayload,
              "Signal 缓冲须与 IR 槽容量一致");

class App
{
public:
    void run()
    {
        // 引脚初始化全部由 CubeMX 生成：
        //   USART2(PA2/PA3 @9600) = RF 串口模块
        //   PA5(GPIO_Output)      = RF 发射 DATA 直驱
        //   PA1(GPIO_Output)      = IR 发射 38kHz
        //   PA0(ADC1_CH0)         = IR 接收
        //   PA6(TIM3_CH1 PWM+DMA) = RGB 灯珠（WS2812B）
        //   PC13(GPIO_Output)     = 用户指示灯（低电平点亮）
        userLed_.begin();            // 确保上电熄灭，仅学习中点亮
        cli_.init();

        uint32_t bootColor[WS2812B_NUM] = {
            WS2812B::Color(0x02, 0x00, 0x02),   
            WS2812B::Color(0x00, 0x02, 0x02),   
        };
        led_.SetPixels(WS2812B_NUM, bootColor);
        led_.UpdatePixels();

        Usart_debugMsg("HOPE-Remote ready (help for commands)");
        for (;;)
        {
            cli_.poll();
        }
    }

private:
    IrStore        irStore_;      // IR 波形槽 0~95（扇区4/5/6）
    RfStore        rfStore_;      // RF 码值槽 100~611（扇区7 前 4KB）
    RfReceiver     rfReceiver_;   // RF 串口帧接收器（USART2）
    Signal         signal_;       // 2KB 波形缓冲（510 段）
    IrReceiver     irReceiver_;   // IR 接收器（PA0/ADC1_CH0）
    IrTransmitter  irTransmitter_;
    RfTransmitter  rfTransmitter_;
    WS2812B        led_{&htim3};   // RGB 灯珠（PA6 / TIM3_CH1 PWM+DMA）
    UserLed        userLed_;       // 用户指示灯（PC13，低电平点亮）
    Cli            cli_{irStore_, rfStore_, rfReceiver_, signal_, irReceiver_,
                        irTransmitter_, rfTransmitter_, led_, userLed_};
};

App app;

void Main(void)
{
    HAL_Delay(200);                 // 等待电源稳定
    HAL_TIM_Base_Start(&htim2);     // 启动 1μs 计时器

    Usart_debugMsg("HOPE-Remote boot");
    app.run();
}
