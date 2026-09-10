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
#include "zf_driver_pwm.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "zf_driver_exti.h"
#include "zf_driver_timer.h"
#include "zf_driver_pit.h"

#include "comparator.h"
#include "bldc_config.h"
#include "pwm_out.h"
#include "battery.h"
#include "motor_control.h"


// 全局电机状态结构体，状态机、中断、换相函数都通过它交换数据
 motor_struct motor;

// 本次换相与上一次换相之间经过的计数，单位 0.5us。
// 在比较器中断里从定时器 4 读出，在定时器 0 中断里被存入滑窗数组。
 static uint16 temp_commutation_time;

// 换相时序的小状态机，在定时器 0 与比较器两个中断之间传递进度：
//   0 = 等换相。比较器中断把过零时刻的延时装进 T0，T0 到点后执行换相。
//   1 = 等消磁。换相完成后重新装 T0 延时，给二极管续流留出时间。
//   2 = 消磁结束。这个值只在定时器 0 中断里被写上，随后立刻打开比较器中断。
 static uint8 xc_flag = 0;
// 0-换相
// 1-消磁
// 2-消磁结束

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机step加一
//  @param      void
//  @return
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 六步换相，step 在 0 至 5 之间循环，走满六步就是一个完整电周期
void motor_next_step(void)
{
    if(++motor.step >= 6) motor.step = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器4重新配置
//  @param      void
//  @return     uint16          返回当前计时器的时间
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 读出定时器 4 从上次清零到现在的计数值，并把计数器清零重新开始计时。
// 读的时候要先把定时器停掉，否则读高低字节的中间被计数进位打断，会读出错误的值。
// 当前工程中没有任何调用点，实际读取 T4 的代码分散在各中断里，属于保留函数。
// 返回值的单位是 0.5us，与 TM4PS 设置的时基一致。
uint16 tim4_reconfig(void)
{
    uint16 temp;
    // 获取换相时间
    T4T3M &= ~0x80; // 停止定时器
    temp = (T4H << 8) | T4L;
    T4L = 0;
    T4H = 0;
    T4T3M |= 0x80;  // 开启定时器
    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器4中断函数
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 定时器 4 的时基是 0.5us，从 0 数到 0xFFFF 约 32.8ms 才会溢出一次。
// 每换一次相都会把 T4 清零，所以这个中断只在"很久没有换相"的时候才会进来，
// 相当于闭环状态下的一道兜底保护。
void TM4_Isr(void) interrupt 20
{
    TIM4_CLEAR_FLAG;
    // 换相超时
    // 只有闭环运行中、且已经累积了足够多的换相次数（说明此前是正常转动的）才判定堵转。
    // 前 500 次换相不判，是为了避开启动阶段转速还不稳定的那段。
    if(MOTOR_CLOSE_LOOP == motor.run_flag && (500) < motor.commutation_num)
    {
        // 正在运行的时候 进入此中断应该立即关闭输出
        motor.run_flag = MOTOR_STOP_STALL;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器0换相中断
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 定时器 0 在这里承担两个不同的延时任务，靠 xc_flag 区分当前处于哪一段。
// 时基也是 0.5us，比较器中断和本中断都用 0xffff 减去目标延时来装载计数器。
// 中断号 TMR0_VECTOR 就是 1，标志位由硬件自动清除。
void TM0_Isr(void) interrupt TMR0_VECTOR
{
	//	硬件自动清除中断标志位
	uint16 tim_com;

	if(xc_flag == 0)
	{
		// 第一段：过零之后又等满了 BLDC_MOTOR_ANGLE 度，现在真正执行换相
		if(MOTOR_CLOSE_LOOP == motor.run_flag)
		{
			// step+1
			motor_next_step();
			// 换相
			pwm_x_output[motor.step]();
			xc_flag = 1;

			// 装载下一次的延时。filter_commutation_time_sum 是一整个电周期（360 度）的时间，
			// 除以 36 就是 10 度的时间，用作换相后的消磁等待。
			tim_com = 0xffff - (motor.filter_commutation_time_sum / 36);

			// 停止定时器0
			TR0 = 0;
			TL0 = (uint8)tim_com;
			TH0 = (uint8)(tim_com >> 8);
			TR0 = 1; 		// 启动定时器
			ET0 = 1; 		// 使能定时器中断

			// 去掉最早的数据
			// 滑窗数组存的是最近 6 次换相各自的耗时，下标就是当时的 step。
			// 本次要覆盖掉的那个格子是上一轮同一步的记录，先把它从总和里减掉。
			motor.commutation_time_sum -= motor.commutation_time[motor.step];
			// 保存换相时间
			motor.commutation_time[motor.step] = temp_commutation_time;
			// 叠加新的换相时间，求6次换相总时长
			motor.commutation_time_sum += temp_commutation_time;
			motor.commutation_num++;
			// 一阶低通滤波
			// 闭环用 7:1 的系数，新值占 1/8，曲线比较平滑，适合已经稳定的转速。
			motor.filter_commutation_time_sum = (motor.filter_commutation_time_sum * 7 + motor.commutation_time_sum * 1) >> 3;

			// 等待稳定，再开始换相错误判断
			if((BLDC_CLOSE_LOOP_WAIT) < motor.commutation_num)
			{
				// 本次换向60度的时间，在上一次换向一圈时间的30度到90度，否则认为换向错误
				// 正常一步应该接近 60 度，允许 30 至 90 度的宽范围，
				// 超出这个范围说明过零点检测错了，累积到 BLDC_COMMUTATION_FAILED_MAX 就停机。
				if((temp_commutation_time > (motor.filter_commutation_time_sum * 30 / 360)) && (temp_commutation_time < (motor.filter_commutation_time_sum * 90 / 360)))
				{
					// 延时减去换向失败计数器
					// 判断正常就减一，等于给计数器做衰减，避免偶发干扰累计成误保护
					if((motor.commutation_failed_num))
					{
						motor.commutation_failed_num--;
					}
				}
				else
				{
					// 只有在占空比大于10%的时候，才进行换相错误判断。
	//				if(motor.duty_register >= (BLDC_PWM_ARR_MAX / 10))
					{
						motor.commutation_failed_num ++;
						if(BLDC_COMMUTATION_FAILED_MAX < motor.commutation_failed_num)
						{
							motor_stop();
							motor.run_flag = MOTOR_STOP_STALL;
						}
					}

				}

//                // 换向时间超过1000us，这种是不可能的。
//                if(motor.filter_commutation_time_sum > (2000*2))
//                {
//                    motor_stop();
//                    motor.run_flag = MOTOR_STOP_STALL;
//                }
			}


		}
	}
	else if(xc_flag == 1)
	{
		// 第二段：换相后已经过了 10 度，续流二极管的反向电流也衰减完了，
		// 这时候再把比较器中断打开，检测到的才是真正的反电动势过零点。
		xc_flag = 2;

//		// 等待消磁, 10度
//		while(((TH0 << 8) | TL0) < (motor.filter_commutation_time_sum / 36));

		comparator_open_isr();

		ET0 = 0; 		// 关闭定时器0中断
		TR0 = 0; 		// 关闭定时器0
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      比较器中断函数
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 悬空相的反电动势过零时触发，是整个无感算法的触发点。
// 中断号 21。
void comparator_isr(void) interrupt 21		// 比较器中断函数, 检测到反电动势过0事件
{
	// BLDC_MOTOR_ANGLE度换相
	// 过零不等于换相点：真正的换相点还要再转过 BLDC_MOTOR_ANGLE 度（默认 10 度）。
	// 这里把这段角度换算成时间装进定时器 0，到点了由 TM0_Isr 执行换相。
	uint16 tim_com = 0xffff - (motor.filter_commutation_time_sum * BLDC_MOTOR_ANGLE / 360);

    // 获取换相时间
    // 一个电周期里相邻两次过零之间正好是 60 度，所以两次比较器中断的间隔就是换相周期。
	temp_commutation_time = (T4H << 8) | T4L;

	xc_flag = 0;

//	// 去除反电动势毛刺
//    if((temp_commutation_time < (motor.commutation_time_sum / 36) && (BLDC_CLOSE_LOOP_WAIT) < motor.commutation_num))
//    {
//        return;
//    }

	// 读完就把定时器 4 清零，下一次过零时读到的就是这一段的净时长
	T4T3M &= ~0x80; 		// 停止定时器4
	T4L = 0;
	T4H = 0;
	T4T3M |= 0x80;  		// 开启定时器

	// 停止定时器0
	TR0 = 0;
	TL0 = (uint8)tim_com;
	TH0 = (uint8)(tim_com >> 8);
	TR0 = 1; 		// 启动定时器
	ET0 = 1; 		// 使能定时器中断


	// 失能比较器中断
	// 换相前后这段时间电压最脏，先关掉，等 TM0_Isr 里消磁结束再重新打开
	comparator_close_isr();


	// 比较器清除中断标志位
	CMPCR1 &= ~0x40;
}


// 上电鸣叫每段音调之间的间隔
#define MUSIC_DELAY_MS   250
//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机上电鸣叫
//  @param      volume          鸣叫音量大小
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 玩法是拿电机绕组当喇叭：按六步换相的方式让两相通电，但用音频频率去斩波，
// 线圈就按音频频率振动发出声音。开机响三声表示初始化完成。
//
// 注意这个函数会把 PWMA 重新配置成小占空比的音频 PWM，
// 所以 main.c 里必须在这之后才调用 pwm_out_init()，
// 否则三相 PWM 的周期、死区会被这里改掉，上电就可能烧管。
void motor_power_on_beep(uint16 volume)
{
	// 音调表。下标 0 是占位，实际只用 1、2、3 三个音：523Hz、587Hz、659Hz。
	uint16  frequency_spectrum[6] = {0, 523, 587, 659, 698, 783};
    uint16 beep_duty;
    beep_duty = volume;
    // 保护限制，避免设置过大烧毁电机
    if(100 < beep_duty)
    {
        beep_duty = 100;
    }

	// A上桥PWM B下桥常开
	// 通电回路是 A 相上桥经绕组到 B 相下桥，绕组里流过音频电流
    PWM_A_H_PIN = 0;
    PWM_A_L_PIN = 0;
    PWM_B_H_PIN = 0;
    PWM_B_L_PIN = 1;
    PWM_C_H_PIN = 0;
    PWM_C_L_PIN = 0;

//	pwm_out_duty_update(motor.duty_register);
	pwm_init(PWMA_CH1P_P00, frequency_spectrum[1], beep_duty);
	PWMA_ENO = 1<<0;
	system_delay_ms(MUSIC_DELAY_MS);


	// B上桥PWM C下桥常开
	PWM_A_H_PIN = 0;
    PWM_A_L_PIN = 0;
    PWM_B_H_PIN = 0;
    PWM_B_L_PIN = 0;
    PWM_C_H_PIN = 0;
    PWM_C_L_PIN = 1;
	pwm_init(PWMA_CH2P_P02, frequency_spectrum[2], beep_duty);
	PWMA_ENO = 1<<2;
	system_delay_ms(MUSIC_DELAY_MS);


	// C上桥PWM A下桥常开
	PWM_A_H_PIN = 0;
    PWM_A_L_PIN = 1;
    PWM_B_H_PIN = 0;
    PWM_B_L_PIN = 0;
    PWM_C_H_PIN = 0;
    PWM_C_L_PIN = 0;
	pwm_init(PWMA_CH3P_P04, frequency_spectrum[3], beep_duty);
	PWMA_ENO = 1<<4;
	system_delay_ms(MUSIC_DELAY_MS);


	// 响完把六路引脚全部拉低并关闭输出
    PWM_A_H_PIN = 0;
    PWM_A_L_PIN = 0;
    PWM_B_H_PIN = 0;
    PWM_B_L_PIN = 0;
    PWM_C_H_PIN = 0;
    PWM_C_L_PIN = 0;

	PWMA_ENO = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机停止
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 占空比清零再关六路输出，最后关比较器中断。
// 用的是 pwm_close_output() 而不是 pwm_brake()，也就是让电机自由滑行，
// 不做能耗制动，避免突然刹车造成电流冲击。
void motor_stop(void)
{
    pwm_out_duty_update(0);
	pwm_close_output();
    comparator_close_isr();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机初始化
//  @param      void
//  @return      void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    // 初始化为推挽输出
    // 六路引脚必须先由 GPIO 配置好，后面 pwm_out_init() 才能让 PWM 外设顺利接管
    gpio_init(IO_P00, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P01, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P02, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P03, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P04, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P05, GPO, 0, GPO_PUSH_PULL);


    // 变量清零
    motor.duty = 0;
    motor.duty_register = 0;
    motor.run_flag = 0;
    motor.motor_start_delay = 0;
    motor.motor_start_wait = 0;
    motor.restart_delay = 0;
    motor.commutation_time_sum = 0;
    motor.commutation_num = 0;

	T4T3M = 0;

    T4T3M |= 1<<7;	// 定时器4使能
    T4T3M |= 1<<5;	// T4 1T模式


	AUXR |= 1<<7;     // 1T模式
	TMOD = 0x00; 	// 模式 0
	TL0 = 0;
	TH0 = 0;
//	TR0 = 1; 		// 启动定时器
//	ET0 = 1; 		// 使能定时器中断
	TM0PS = (system_clock / 1000000 / 2) - 1;	// 设置分频系数，时基为0.5us

    T4L = 0;
    T4H = 0;
    TM4PS = (system_clock / 1000000 / 2) - 1;	// 设置分频系数，时基为0.5us

    // 先把所有中断优先级清零，再逐个设置，避免受复位默认值影响
    IP = 0;
    IPH = 0;
    IP2 = 0;
    IP2H = 0;

    // 设置定时器0优先级为 3 ,最高优先级为3
    // IP 和 IPH 的同一位组合成优先级：01 为 1，10 为 2，11 为 3
    IP  |= 1<<1;
    IPH |= 1<<1;

	// 设置比较器优先级 为 3 ,最高优先级为3
	// 比较器过零和换相都是时序敏感的，都给最高级，保证不被其他中断拖延
	IP2  |= 1<<5;
	IP2H |= 1<<5;

	// 设置PWMB输入捕获高优先级 为 2 ,最高优先级为3
	// IP2 该位为 0、IP2H 该位为 1，即优先级 2，比电机相关的两个中断低一级
    IP2  |= 0<<3;
    IP2H |= 1<<3;



#if (1 == BLDC_BEEP_ENABLE)
    // 电机鸣叫表示初始化完成
    motor_power_on_beep(BLDC_BEEP_VOLUME);
#endif
}
