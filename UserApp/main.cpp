#include "common_inc.h"
#include "interface_uart.h"

#include "tim.h"
#include "signal.hpp"
#include "storage.hpp"
#include "receiver.hpp"
#include "transmitter.hpp"
#include "ir_transmitter.hpp"
#include "cli.hpp"

static_assert(Signal::kMaxSegments == Storage::kMaxSegsPerSlot,
              "Signal 缓冲须与 Flash 槽容量一致");

class App
{
public:
    void run()
    {
        cli_.init();
        Usart_debugMsg("HOPE-Remote ready (help for commands)");
        for (;;)
        {
            cli_.poll();
        }
    }

private:
    Storage        storage_;
    Signal         signal_;      // 2KB 波形缓冲（510 段）
    Receiver       receiver_;
    IrTransmitter transmitter_;
    Cli           cli_{storage_, signal_, receiver_, transmitter_};
};

App app;

void Main(void)
{
    HAL_Delay(200);                 // 等待电源稳定
    HAL_TIM_Base_Start(&htim2);     // 启动 1μs 计时器

    Usart_debugMsg("HOPE-Remote boot");
    app.run();
}
