#ifndef __USER_LED_HPP
#define __USER_LED_HPP

#include "main.h"

// 用户指示灯驱动（PC13，低电平点亮）。
//
// 电路：3.3V → 2kΩ → LED → PC13，故 **拉低=亮、拉高=灭**。
// 引脚模式/速度由 CubeMX 生成的 MX_GPIO_Init()（Core/Src/gpio.c）配置，
// 本类只负责写电平，不做 HAL_GPIO_Init，避免与生成代码重复配置。
//
// PC13 属备份域引脚，数据手册限定输出速度 ≤2MHz、灌电流 ≤3mA，
// CubeMX 已配 GPIO_SPEED_FREQ_LOW，勿改成 HIGH。
//
// 行为约定：仅 xxNNN 学习命令期间常亮，学习结束（OK / TIMEOUT /
// EMPTY / FLASHERR 任一出口）立即熄灭；其余命令不点灯。

#define USER_LED_PORT GPIOC
#define USER_LED_PIN  GPIO_PIN_13

class UserLed
{
public:
    // 初始化为熄灭态（PC13 拉高）。在 App::run() 里调用一次。
    void begin(void);

    // 点亮（PC13 拉低）/ 熄灭（PC13 拉高）
    void on(void);
    void off(void);
    void set(bool _lit) { _lit ? on() : off(); }

    // 作用域守卫：构造即点亮，析构必熄灭。
    // 学习流程有 OK/TIMEOUT/EMPTY/FLASHERR 多个 return 出口（IR/RF 两条
    // 路径共用），用它保证"任何出口都灭"，不必逐处补 off()。
    class Guard
    {
    public:
        explicit Guard(UserLed& _led) : led(_led) { led.on(); }
        ~Guard() { led.off(); }

        Guard(const Guard&)            = delete;
        Guard& operator=(const Guard&) = delete;

    private:
        UserLed& led;
    };
};

#endif // __USER_LED_HPP
