#pragma once

// 舵机硬件约束的唯一配置，控制器、TCP客户端和串口共用。
#define PWMSERVOMAX 1900 // 舵机PWM最大值（左）
#define PWMSERVOMID 1500 // 舵机PWM中值
#define PWMSERVOMIN 1100 // 舵机PWM最小值（右）
