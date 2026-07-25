#ifndef ESP32_CAR_PINS_H
#define ESP32_CAR_PINS_H
// 此处定义平衡车的所有引脚
#define  MPU6050_SCL 32
#define  MPU6050_SDA 33
#define  MPU6050_INT 25

#define  MOTOR_L_IN1 22
#define  MOTOR_L_IN2 23
#define  MOTOR_L_EN1 12
#define  MOTOR_L_EN2 14

#define  MOTOR_R_IN1 19
#define  MOTOR_R_IN2 18
#define  MOTOR_R_EN1 27
#define  MOTOR_R_EN2 26

#define CH_L_IN1 0
#define CH_L_IN2 1
#define CH_R_IN1 2
#define CH_R_IN2 3
#define CH_LED 4
#define LEDC_BITS_WIDTH 8
#define MAX_PWM (pow(2, LEDC_BITS_WIDTH) - 1)
#define LEDC_FREQ_HZ 1000
#define LEDC_LED_FREQ_HZ 5000

#define BAT_DETECT 34
#define LED 21

#endif //ESP32_CAR_PINS_H
