
#ifndef _MOTORDRIVERCONSTANTS_H_
#define _MOTORDRIVERCONSTANTS_H_

#define MOTORS_ADDRESS 0x62
#define MOTORS_MAX_PWM_PERIOD 0xFFFF
#define MOTORS_ACCELERATION_TIME_MICROS 10000

#define COMMAND_SET_MODE 0x00  //write mode- command, register access
#define COMMAND_SERVO_1 0x01 //write 16 bit pwm value 1
#define COMMAND_SERVO_2 0x02 //write 16 bit pwm value 2
#define COMMAND_SERVO_3 0x03 //write 16 bit pwm value 3
#define COMMAND_SERVO_4 0x04 //write 16 bit pwm value 4
#define COMMAND_MOTOR_1 0x05 //write first two 16 bit pwm values
#define COMMAND_MOTOR_2 0x06 //write second two 16 bit pwm values
#define COMMAND_TIMER_1 0x08 //write 16 bit timer 1 top value
#define COMMAND_TIMER_2 0x09 //write 16 bit timer 2 top value
#define COMMAND_PRESCALER_1 0x0A //write timer 1 prescaler
#define COMMAND_PRESCALER_2 0x0B //write timer 2 prescaler
#define COMMAND_CLOCK_PRESCALER 0x0C //write system clock prescaler
#define COMMAND_SET_SLEEP_MODE 0x0D //set sleep mode
#define COMMAND_SLEEP 0x0E //go to sleep after I2C communication is done
#define COMMAND_SET_FAILSAFE_VALUES 0x0F //set failsafe PWM values - default is 0
#define COMMAND_SET_FAILSAFE_PRESCALER 0x10 //set failsafe timeout
#define COMMAND_SET_FAILSAFE_TIMEOUT 0x11 //set failsafe timeout
#define COMMAND_ALL_PWM_8 0x12 //write four 8 bit pwm values
#define COMMAND_ALL_PWM 0x07 //write four 16 bit pwm values

#define T841_CLOCK_PRESCALER_1 0x00
#define T841_CLOCK_PRESCALER_2 0x01
#define T841_CLOCK_PRESCALER_4 0x02
#define T841_CLOCK_PRESCALER_8 0x03
#define T841_CLOCK_PRESCALER_16 0x04
#define T841_CLOCK_PRESCALER_32 0x05
#define T841_CLOCK_PRESCALER_64 0x06
#define T841_CLOCK_PRESCALER_128 0x07
#define T841_CLOCK_PRESCALER_256 0x08

#define T841_TIMER_PRESCALER_0 0x00
#define T841_TIMER_PRESCALER_1 0x01
#define T841_TIMER_PRESCALER_8 0x02
#define T841_TIMER_PRESCALER_64 0x03
#define T841_TIMER_PRESCALER_256 0x04
#define T841_TIMER_PRESCALER_1024 0x05

#define T841_SLEEP_MODE_IDLE 0
#define T841_SLEEP_MODE_ADC _BV(T841_SM0)
#define T841_SLEEP_MODE_PWR_DOWN _BV(T841_SM1)

#define MODE_REGISTER_INC 0xAA
#define MODE_REGISTER_DEC 0xAB
#define MODE_COMMAND 0xAC

#define FIRMWARE_REVISION_REG 0x19
#define EXPECTED_FIRMWARE 0x1A

#endif
