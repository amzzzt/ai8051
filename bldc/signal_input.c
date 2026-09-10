/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		pit_timer
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2024-01-22
 ********************************************************************************************************************/

#include "zf_common_typedef.h"
#include "zf_common_clock.h"

#include "bldc_config.h"
#include "motor_control.h"
#include "pwm_out.h"
#include "signal_input.h"

// 油门信号输入引脚。注意初始化里实际用的是 IO_P21，这个宏没有被引用，属于冗余定义。
#define PWMIN_PIN   P21

// 全局的油门解析结果，中断里填写，主控制循环读取
pwmin_struct pwmin;


// 信号超时计数，信号正常时在中断里被清零
uint8 pwm_input_timeout_count = 0;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWMB输入捕获中断
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 中断号 27 对应 PWMB。三个标志位分别是：
//   PWMB_SR1 bit1 (0x02) = CC5IF，通道 5 捕获事件
//   PWMB_SR1 bit2 (0x04) = CC6IF，通道 6 捕获事件
//   PWMB_SR1 bit0 (0x01) = UIF，更新事件
// 每次进中断都按标志位分别处理，最后统一由 PWMB_SR1 = 0 清掉全部标志。
//
// 输入信号是航模接收机的标准 PWM：周期约 20ms，高电平 1ms 至 2ms 对应油门 0 至 100%。
void pwmb_isr()interrupt 27
{
    uint16 temp;
	if(PWMB_SR1 & 0x02)
	{
		// 通道 5 捕获到上升沿，CCR5 里存的是与上一个上升沿之间的计数值，也就是信号周期。
		// 计数器时基是 1us，所以这个数值直接就是微秒数。
		pwmin.period = (PWMB_CCR5H << 8) + PWMB_CCR5L;	    // CC1捕获周期宽度
		PWMB_SR1 = 0;

        // 计算输入PWM信号的频率
        pwmin.frequency = system_clock / (PWMB_PSCRL + 1) / pwmin.period;
	}

	if(PWMB_SR1 & 0x04)
	{
		// 通道 6 捕获到下降沿，CCR6 存的是本次上升沿到下降沿之间的计数值，即高电平宽度。
		pwmin.high_value = (PWMB_CCR6H << 8) + PWMB_CCR6L;   // CC2捕获高电平宽度
		PWMB_SR1 = 0;

        // 频率在合理的范围内才计算
        if((30 < pwmin.frequency) && (400 > pwmin.frequency))
        {
            // 计算高电平时间 仅在高电平时间为1-2ms内有效
            pwmin.high_time = pwmin.high_value;

            if((3000 < pwmin.high_time) || (1000 > pwmin.high_time))
            {
                // 高电平时间过长或者过短，则油门设置为0
                // 这一段是失联保护：接收机没信号或者信号异常时，高电平宽度会跑出 1ms 至 2ms。
                pwmin.throttle = 0;
            }
            else
            {
                if(2000 < pwmin.high_time)
                {
                    // 上限钳位，高电平超过 2ms 的一律按满油门算
                    pwmin.high_time = 2000;
                }

                // 计算油门大小
                // 1ms 对应油门 0，2ms 对应油门 1000，所以减掉 1000 做归一化。
                temp = pwmin.high_time - 1000;
                // 如果输入的油门大小 小于启动占空比则油门设置为5%
                // 注意这里的注释和代码不一致：代码实际是把 50us 以下直接归零，
                // 也就是死区内输出 0，而不是注释写的 5%。
                if(temp < 50)
                {
                    temp = 0;
                }
                pwmin.throttle = temp;
            }

			pwm_input_timeout_count = 0;
        }

        // 更新占空比
        // throttle 取值 0 至 1000，按比例换算成 PWM 比较寄存器的 0 至 BLDC_PWM_ARR_MAX。
        // 注意这一句在频率判断之外：如果频率超范围导致 throttle 没被更新，
        // 这里会拿上一次的旧油门重新算一遍 duty。
        motor.duty = (uint32)pwmin.throttle * BLDC_PWM_ARR_MAX / 1000;
	}

    if(PWMB_SR1 & 0x01)
    {
        PWMB_SR1 = 0;


        // 未检测到输入信号则输出油门都清零
        // 连着一个油门量却没有收到高电平捕获，累计两次就把油门清零。
        // 由于 PWMB 工作在复位模式，没有信号输入时更新事件本身是否会产生需要结合
        // 从模式的行为确认，所以这段保护的实际触发条件以实测为准。
		if(motor.duty > 0)
		{
			if(++pwm_input_timeout_count >= 2)
			{
				pwm_input_timeout_count = 0;

				pwmin.throttle = 0;
				motor.duty = 0;
			}
		}
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWMB输入捕获初始化
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwm_input_init(void)
{
    // 两个引脚都配成高阻输入。P21 接实际的油门信号，P23 是 PWM 引脚映射后的备用位置。
    gpio_init(IO_P21, GPI, 0, GPI_IMPEDANCE);
    gpio_init(IO_P23, GPI, 0, GPI_IMPEDANCE);

    PWMB_PS = 0x0A;		// 通道引脚切换
    PWMB_CCMR1 = 0x01;	// CC5为输入模式,且映射到TI5FP5上
	PWMB_CCMR2 = 0x02;	// CC6为输入模式,且映射到TI5FP6上

	// CC5E 开启输入捕获
	// CC5P 捕获发生在TI5F的上升沿
	// CC6E 开启输入捕获
	// CC6P 捕获发生在TI5F的下降沿
	// CCER1 低 4 位是通道 5 的配置，高 4 位是通道 6 的配置，0x31 即两个通道都开。
    PWMB_CCER1 = 0x31;

    PWMB_PSCRH = 0;		// 分频值
    // 预分频把计数器时基配成 1us，这样捕获到的计数值直接就是微秒数，
    // 免去后面再做换算。
	PWMB_PSCRL = system_clock / 1000000 - 1;    // 分频值
    // TS = TI1FP1，SMS = 复位模式：每次通道 1 的上升沿到来时把计数器清零，
    // 于是 CCR5 单独就能读出完整周期，不需要做两次捕获相减。
    PWMB_SMCR = 0x54;	// TS=TI1FP1,SMS=TI1上升沿复位模式
	PWMB_CR1 = 0x01;	// 启动PWMB，向上计数
	PWMB_IER = 0x07;	// 使能CC1、CC2、UIE中断

    // 清掉上一轮的捕获值，避免上电第一拍读到脏数据
    pwmin.period = 0;
    pwmin.high_value = 0;
    pwmin.high_time = 0;
}
