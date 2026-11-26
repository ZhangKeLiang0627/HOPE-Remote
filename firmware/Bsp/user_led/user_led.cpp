#include "user_led.hpp"

// PC13 用户指示灯：3.3V → 2kΩ → LED → PC13，低电平导通点亮。
// 上电时 CubeMX 已写入 GPIO_PIN_SET（灭），begin() 再显式保证一次，
// 这样即使后续有人在别的初始化里动过 PC13，也能回到确定的熄灭态。

void UserLed::begin(void)
{
    off();
}

void UserLed::on(void)
{
    HAL_GPIO_WritePin(USER_LED_PORT, USER_LED_PIN, GPIO_PIN_RESET);
}

void UserLed::off(void)
{
    HAL_GPIO_WritePin(USER_LED_PORT, USER_LED_PIN, GPIO_PIN_SET);
}
