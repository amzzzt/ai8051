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

#include "zf_driver_timer.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "zf_driver_pit.h"

#include "bldc_config.h"
#include "pwm_out.h"
#include "signal_input.h"
#include "motor_control.h"
#include "battery.h"
#include "pit_timer.h"
#include "comparator.h"
#include "motor_control.h"

#include "sine_control.h"


// 屏蔽编译器警告 183。下面 gpio_state 是 uint8 做移位，编译器对提升后的类型比较敏感。
#pragma warning disable = 183


// 两个状态指示灯。两灯都是低电平点亮。
// 具体闪烁含义见 led_control()。
#define ERR_LED P37
#define RUN_LED P36

// 50us 中断的次数。所有的时间基准都由它换算，是整份代码里唯一的"系统时钟"。
static uint32 pit_count = 0;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      LED灯光控制
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 每 50us 进一次该函数，用取余的方式实现分频闪烁。
// 各状态对应的表现：
//   低压保护 BYTE_LOW_VOLTAGE   ERR 常亮，RUN 熄灭
//   堵转停机 MOTOR_RESTART      ERR 每 50ms 翻转一次，RUN 熄灭
//   空闲     MOTOR_IDLE         RUN 每 200ms 翻转一次，ERR 熄灭
//   转动中   其余状态           RUN 常亮，ERR 熄灭
void led_control(void)
{
    #define TICK_TO_MS(ms)  (ms * 20)
    // 50us进一次该函数
    // LED状态显示
    if(BYTE_LOW_VOLTAGE == motor.run_flag)
    {
        // 电池电压过低，ERROR LED慢闪
        // 注意这里的注释和代码不一致：代码是把 ERR 一直拉低，
        // 也就是常亮不闪，不是注释写的慢闪。
        ERR_LED = 0;
        RUN_LED = 1;
    }
    else if(MOTOR_RESTART == motor.run_flag)
    {
        // 由于堵转造成的停止
        // 每 50ms 翻转一次，即 10Hz 闪烁
        if(0 == (pit_count%TICK_TO_MS(50)))
        {
            ERR_LED = !ERR_LED;
            RUN_LED = 1;
        }
    }
    else if(MOTOR_IDLE == motor.run_flag)
    {
        // 空闲状态
        // 每 200ms 翻转一次，即 2.5Hz 慢闪，表示已上电等待油门
        if(0 == (pit_count%TICK_TO_MS(200)))
        {
            RUN_LED = !RUN_LED;
            ERR_LED = 1;
        }
    }
    else if(MOTOR_START == motor.run_flag || MOTOR_OPEN_LOOP == motor.run_flag || MOTOR_CLOSE_LOOP == motor.run_flag)
    {
        // 电机转动状态
        ERR_LED = 1;
        RUN_LED = 0;
    }

}

//-------------------------------------------------------------------------------------------------------------------
//  电机状态机主体
//
//  每个分支对应 motor.run_flag 的一个取值。状态的流转关系：
//
//      MOTOR_IDLE -------- 油门不为 0 --------> MOTOR_START
//      MOTOR_START ------- 正弦强拉一圈 ------> MOTOR_OPEN_LOOP
//      MOTOR_OPEN_LOOP --- 换相次数达标 ------> MOTOR_CLOSE_LOOP
//      MOTOR_CLOSE_LOOP ----------------------> MOTOR_STOP_STALL （堵转或换相错误）
//      MOTOR_STOP_STALL -- 关闭输出 ----------> MOTOR_RESTART
//      MOTOR_RESTART ----- 等待 BLDC_START_DELAY -> MOTOR_IDLE
//
//  BYTE_LOW_VOLTAGE 是最高优先级的状态，由 TM1_Isr 在检测到持续低压时直接写入，
//  一旦进入就不会再自己退出，必须断电重启。
//-------------------------------------------------------------------------------------------------------------------
void pit_motor_control()
{
    // 堵转检测计数器，单位是进入本函数的次数，也就是 50us
    static uint16 stall_time_out_check = 0;
    // 开环阶段用于连续采样的引脚电平移位寄存器。注意它只是本函数内部的局部静态变量，
    // 并没有文件作用域的定义，因此 pit_timer.h 里那句 extern uint8 gpio_state 是悬空的。
	static uint8 gpio_state = 0;
    uint8 pin_state = 0;
    // 闭环阶段用于比较引脚电平是否发生跳变的上一次采样值
	static uint8 oled_pin_state = 0;

    switch(motor.run_flag)
    {
		case MOTOR_START:
		{
			uint16 sine_delay;
			// 启动阶段用不到这些中断，先全部关掉，避免换相干扰正弦强拉
			ET0 = 0; 					// 关闭定时器0中断
			IE2 = ~0x40;			  	// 关闭定时器4中断
			comparator_close_isr();		// 关闭比较器中断
			stall_time_out_check = 0;	// 堵转检测计数器设置为0
			motor.commutation_num = 0;	// 电机换相计数器设置为0

            PWMA_ENO = 0x3F;            // 打开所有PWM通道

			#if BLDC_USE_SINE_START

			// 正弦启动：三相同时出力产生旋转磁场，把静止的转子强行拖起来。
			// 这段 do-while 会一直阻塞在 50us 的 PIT 中断里，
			// 按默认参数算 360 步乘 30us 约 10.8ms，期间其他中断仍然可以抢占。
			sine_init();
			do
            {
                // 把油门换算成占空比寄存器值，并夹在 BLDC_MIN_DUTY 与 BLDC_MAX_DUTY 之间。
                // 启动阶段限制最大占空比，避免一上来就大电流。
                motor.duty_register = motor.duty;

                if (motor.duty_register < ((uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100))
                {
                    motor.duty_register = (uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100;
                }
                else if (motor.duty_register > ((uint32)BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100))
                {
                    motor.duty_register = ((uint32)BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100);
                }

				sine_start(motor.duty_register);

				// 用定时器 4 给每一步计时
				T4T3M &= ~0x80; // 停止定时器
				T4L = 0;
				T4H = 0;
				T4T3M |= 0x80;  // 开启定时器

				// 每一步的停留时间。极对数越多，同样的机械角度对应的电角度越大，
				// 所以用 420 除以极对数来缩短单步时间，保证不同电机拖起来的转速接近。
				sine_delay = 420 / BLDC_POLES;
				// 时基 0.5us，默认极对数 7 时约 30us 一步
				while(((T4H << 8) | T4L) < sine_delay);
			}while(motor_a_position != 0);
			// 转到 motor_a_position 回到 0，说明磁场已经转满一圈，此时转子已经跟上来了

			// 从正弦强拉的终点直接进入六步换相的对应相位
			motor.step = 2;
			// 给滑窗填一个初值。6000 是 0.5us 时基下的 3ms，对应比较低的转速，
			// 让滤波器的起始值偏慢，换相延迟从大往小收敛，比一开始就冲快更稳。
			motor.commutation_time[0] = 6000;
			motor.commutation_time[1] = 6000;
			motor.commutation_time[2] = 6000;
			motor.commutation_time[3] = 6000;
			motor.commutation_time[4] = 6000;
			motor.commutation_time[5] = 6000;
			motor.commutation_time_sum = motor.commutation_time[0] + motor.commutation_time[1] + motor.commutation_time[2] + \
										 motor.commutation_time[3] + motor.commutation_time[4] + motor.commutation_time[5] ;
			motor.filter_commutation_time_sum = motor.commutation_time_sum;
			// 清空定时器计数器值
			T4T3M &= ~0x80; // 停止定时器
			T4L = 0;
			T4H = 0;
			T4T3M |= 0x80;  // 开启定时器

			IE2 &= ~0x20; 	// 关闭定时器3中断

			motor.run_flag = MOTOR_OPEN_LOOP;

			// 设置占空比
			// 进入开环后可以放到 BLDC_MAX_DUTY，比启动阶段放开一些
			motor.duty_register = motor.duty;

            if (motor.duty_register < ((uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100))
            {
                motor.duty_register = (uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100;
            }
            else if (motor.duty_register > ((uint32)BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100))
            {
                motor.duty_register = ((uint32)BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100);
            }

			pwm_out_duty_update(motor.duty_register);

            // 换相
			motor_next_step();
			pwm_x_output[motor.step]();

			#else

			// 不使用正弦启动时的备选方案：直接给一个占空比，让电机自己转起来。
			// 这种强起方式在轻载下能用，重载或静止阻力大时容易堵转。

			// 设置启动占空比
			motor.duty_register = motor.duty;

            if (motor.duty_register < (BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100))
            {
                motor.duty_register = BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100;
            }
            else if (motor.duty_register > (BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100))
            {
                motor.duty_register = (BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100);
            }

			pwm_out_duty_update(motor.duty_register);
			// 等定时器 4 数满一段时间再切换相。时基 0.5us，20000 个计数即 10ms。
			if(((T4H << 8) | T4L) > 20*1000)
			{
				motor.commutation_time[0] = 6000;
				motor.commutation_time[1] = 6000;
				motor.commutation_time[2] = 6000;
				motor.commutation_time[3] = 6000;
				motor.commutation_time[4] = 6000;
				motor.commutation_time[5] = 6000;
				motor.commutation_time_sum = motor.commutation_time[0] + motor.commutation_time[1] + motor.commutation_time[2] + \
											 motor.commutation_time[3] + motor.commutation_time[4] + motor.commutation_time[5] ;
				motor.filter_commutation_time_sum = motor.commutation_time_sum;
				// 清空定时器计数器值
				T4T3M &= ~0x80; // 停止定时器
				T4L = 0;
				T4H = 0;
				T4T3M |= 0x80;  // 开启定时器

				IE2 &= ~0x20; 	// 关闭定时器3中断

				motor_next_step();
				pwm_x_output[motor.step]();
				motor.run_flag = MOTOR_OPEN_LOOP;
			}
			#endif
		}
		break;
		case MOTOR_OPEN_LOOP:
		{
			// 开环阶段比较器中断是关着的，这里靠 50us 轮询比较器输出来找过零点。
			// 轮询精度只有 50us，比中断差很多，所以这一段只能用来把转速带上来源，
			// 真正精确的过零检测要等切到闭环之后。
			uint8 motor_next_step_flag = 0;
			uint16 temp_commutation_time = (T4H << 8) | T4L;

            if (motor.duty_register < ((uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100))
            {
                motor.duty_register = (uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100;
            }
            else if (motor.duty_register > ((uint32)BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100))
            {
                motor.duty_register = ((uint32)BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100);
            }


			// 通过位移的方式，将电平数据存入变量中
			// 每次把历史状态左移一位，新的比较器输出补到最低位。
			// 这样一连串采样就压进一个字节里，连续 8 次同电平就分别得到 0x00 或 0xFF。
			gpio_state = (gpio_state << 1) | (CMPCR1 & 0x01);
			if (!(motor.step % 2))
			{
				//下降沿
				// 偶数步监测的是下降沿过零，连续 8 次都是低电平说明已经越过零点
				if(gpio_state == 0)
				{
					motor_next_step_flag = 1;
					// 预置成半字节低电平，下一次再连续 4 次低电平就能凑满 0x00
					gpio_state = 0x0F;
				}
			}
			else
			{
				// 上升沿
				if(gpio_state == 0xFF)
				{
					motor_next_step_flag = 1;
					gpio_state = 0xF0;
				}
			}

			if(motor_next_step_flag)
			{
				// 检测到过零，重新开始计时并换相
				T4T3M &= ~0x80; // 停止定时器
				T4L = 0;
				T4H = 0;
				T4T3M |= 0x80;  // 开启定时器
				// 堵转计数清空
				stall_time_out_check = 0;
				// 去掉最早的数据
				motor.commutation_time_sum -= motor.commutation_time[motor.step];
				// 保存换相时间
				motor.commutation_time[motor.step] = temp_commutation_time;
				// 叠加新的换相时间，求6次换相总时长
				motor.commutation_time_sum += motor.commutation_time[motor.step];
				// 换相次数增加一
				motor.commutation_num++;
				// 一阶低通滤波
				// 开环用 3:1 的系数，新值占 1/4，比闭环的 7:1 跟得更快，
				// 因为开环阶段转速变化剧烈，需要滤波器快速跟上。
				motor.filter_commutation_time_sum = (motor.filter_commutation_time_sum * 3 + motor.commutation_time_sum * 1) >> 2;
				// 等待15度
				// 一个电周期是 360 度，除以 24 就是 15 度。
				// 轮询到过零本身已经滞后了，这里只再补 15 度就换相。
				while(((T4H << 8) | T4L) < (motor.filter_commutation_time_sum / 24));
				// 电机step加一
				motor_next_step();
				// 电机换相
				pwm_x_output[motor.step]();
			}
			else
			{
				// 堵转检测
				// 10*200 = 2000 次，每次 50us，也就是连续 100ms 没有检测到过零点就判堵转。
				// 上面那句注释写的 0.1ms 计数一次是错的，PIT 的实际周期是 50us。
				if((10 * 200) < stall_time_out_check++)
				{
					stall_time_out_check = 0;
					motor.commutation_num = 0;
					motor.run_flag = MOTOR_STOP_STALL;
				}
			}

			// 闭环状态切换
			//if((motor.commutation_num > (BLDC_OPEN_LOOP_WAIT)) && (motor.filter_commutation_time_sum < 30000))
			// 换够 BLDC_OPEN_LOOP_WAIT 次说明电机已经稳定转起来了，切闭环。
			// 被注释掉的那个条件还额外要求转速够快，如果切闭环后出现抖动，
			// 可以把转速条件打开试试。
			if(motor.commutation_num > BLDC_OPEN_LOOP_WAIT)
			{
				// 清空计数器
				motor.commutation_failed_num = 0;
				// 清除一些变量
				motor.commutation_num = 0;
				stall_time_out_check = 0;
				// 打开比较器中断
				// 闭环靠中断检测过零，精度远高于这里的 50us 轮询
				comparator_open_isr();
				// 使能定时器4中断
				IE2 |= 0x40;
				// 使能定时器4
				T4T3M |= 0x80;
				// 切换为闭环状态
				motor.run_flag = MOTOR_CLOSE_LOOP;
			}

			// 超过20ms没有检测到跳变，则从进入电机开始状态
			// 同样按 0.5us 时基换算，20000 个计数实际是 10ms，不是注释写的 20ms。
			// 长时间找不到过零点，说明开环拖不动了，退回去重新走一遍启动流程。
			if(((T4H << 8) | T4L) > 20*1000)
			{
				T4T3M &= ~0x80; // 停止定时器
				T4L = 0;
				T4H = 0;
				T4T3M |= 0x80;  // 开启定时器
				// 电机step加一
				motor_next_step();
				// 电机换相
				pwm_x_output[motor.step]();
				motor.run_flag = MOTOR_START;
			}
		}
		break;
		case MOTOR_CLOSE_LOOP:
		{
			// 闭环阶段过零由比较器中断负责换相，这里只管两件事：
			// 一是堵转检测，二是油门变化时的加减速斜坡。
			pin_state = (CMPCR1 & 0x01);
			// 通过PA2引脚的电平状态，进行堵转检测
			// 这句注释是过时的：实际读的是比较器结果寄存器 CMPCR1 的最低位，
			// 不是 PA2 引脚。
			if(oled_pin_state != pin_state)
			{
				// 电平还在翻转说明电机在转，计数清零
				stall_time_out_check = 0;
				oled_pin_state = pin_state;
			}
			else
			{
				// 如果引脚一直是一个状态，且超过了 BLDC_STALL_TIME_OUT ms 则认为堵转了
				// 这里用的不是 BLDC_STALL_TIME_OUT，而是写死的 10*200 = 2000 次。
				// 每次 50us，所以实际是 100ms。
				if(stall_time_out_check++ > (10 * 200))          // 10KHZ，0.1ms计数一次
				{
					stall_time_out_check = 0;
					motor.run_flag = MOTOR_STOP_STALL;
				}
			}

//			// 闭环缓慢加速
//			if(motor.commutation_num < (BLDC_CLOSE_LOOP_WAIT))
//            {
//                 if (motor.duty_register < BLDC_MIN_DUTY)
//                 {
//                     motor.duty_register = BLDC_MIN_DUTY;
//                 }
//                 if (motor.duty_register > (BLDC_PWM_ARR_MAX / 10 + BLDC_MIN_DUTY))
//                 {
//                     motor.duty_register = BLDC_PWM_ARR_MAX / 10 + BLDC_MIN_DUTY;
//                 }
//            }


			// 缓慢加减速
			// BLDC_PWM_ARR_MAX = 833
			// 50us进一次PIT中断，一毫秒进20次。
			// 如果每一次进来，就修改一次duty的值，那么41.65ms就能从0%拉到100%
            // 等待稳定。
			// 下面两个 if 是嵌套关系，第一个管的是"换相次数够了才开始调速"，
            // 也就是刚切闭环时先保持当前占空比，等转速稳了再跟随油门。
            // 间距上的写法容易看错，改代码时注意别把大括号加错位置。
            if((BLDC_CLOSE_LOOP_WAIT) < motor.commutation_num)
			if((motor.duty_register != motor.duty) && (pit_count % BLDC_SPEED_INCREMENTAL == 0))
			{
				// 每次只把 duty_register 改动 1，靠 BLDC_SPEED_INCREMENTAL 控制改动频率，
				// 两者共同决定爬升速度，避免油门突变时电流冲击过大。
				if(motor.duty > motor.duty_register)
				{
					motor.duty_register ++;
					if(BLDC_PWM_ARR_MAX < motor.duty_register)
					{
						motor.duty_register = BLDC_PWM_ARR_MAX;
					}
				}
				else
				{
					// 减速时同样限一个小占空比，不能直接掉到 0，
					// 否则反电动势还没有建立的这段时间电机就失去转矩了。
					motor.duty_register--;
                    if (motor.duty_register < ((uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100))
                    {
                        motor.duty_register = (uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100;
                    }
				}

//				// 只有在减速的时候打开互补PWM
//				if((int16)((int16)motor.duty_register - (int16)motor.duty) >= (BLDC_PWM_ARR_MAX >> 4))
//				{
//					g_use_complementary = 1;
//				}
//				else
//				{
//					g_use_complementary = 0;
//				}

				pwm_out_duty_update(motor.duty_register);
			}


		}
		break;
		case MOTOR_STOP_STALL:
		{
			// 先真正关掉输出，再进重启倒计时，保证停机状态下功率管是断开的
			motor_stop();
			IE2 = ~0x40;				// 关闭定时器4中断
			comparator_close_isr();		// 关闭比较器中断
			motor.run_flag = MOTOR_RESTART;
			motor.restart_delay = BLDC_START_DELAY;
		}
		break;
		case MOTOR_RESTART:
		{
			if(motor.restart_delay)
			{
				// 延时启动时间减减
				// 每 50us 减一，BLDC_START_DELAY 默认 10000，也就是等约 500ms。
				// 留这段时间是为了让转子完全停下来，避免在还转着的时候又去强拉。
				motor.restart_delay--;
			}
			else
			{
				// 倒计时结束回到空闲，此时如果油门还在，TM1_Isr 会立刻重新触发启动
				motor.run_flag = MOTOR_IDLE;
			}
		}
		break;
		case MOTOR_IDLE:
		case BYTE_LOW_VOLTAGE:
		{
			// 空闲和低压保护共用同一套处理：关掉所有换相相关的中断和输出。
			// 区别只在于状态是从哪里来的，低压保护一旦进入就不会退出。
			ET0 = 0; 					// 关闭定时器0中断
			IE2 = ~0x40;			  	// 关闭定时器4中断
			comparator_close_isr();		// 关闭比较器中断
			motor_stop();
		}
		break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器1中断
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 整个电调的主循环，每 50us 执行一次，中断号 3。
// 顺序上先做安全检查，再决定状态，最后刷新指示灯和推进状态机。
void TM1_Isr() interrupt 3
{
    pit_count++;
    if(battery_voltage_get())
    {
        // 低压保护是锁存的：一旦写进去，下面的判断就不会再改它，
        // 状态机也只会停在关输出的分支里，必须断电才能恢复。
        motor.run_flag = BYTE_LOW_VOLTAGE;
    }
    // 电池电压检测为最高优先级，当电压过低，就不转
    if(motor.run_flag != BYTE_LOW_VOLTAGE)
    {
        // 输入的占空比为0，则为空闲状态
        // 油门归零就直接回空闲，也就是松油门是立刻断输出，
        // 没有让电机靠惯性滑行减速的过渡过程。
        if(motor.duty == 0)
        {
            motor.run_flag = MOTOR_IDLE;
        }
        else
        {
            // 输入的占空比不为0，则从空闲状态转为电机开始
            if(motor.run_flag == MOTOR_IDLE)
            {
                motor.run_flag = MOTOR_START;
            }
        }
    }
    led_control();
    pit_motor_control();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      周期定时器初始化
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
   void pit_timer_init(void)
{
//	uint8 freq_div = 0;
//    uint16 period_temp = 0;
//    uint16 temp = 0;
//	uint16 tim_us = 50;		// 设置为0.5ms中断一次,200hz频率

//	AUXR |= 0x40;													// 设置为1T模式
//	freq_div = ((tim_us * system_clock / 1000000) / (1 << 16));          // 计算预分频
//	period_temp = ((tim_us * system_clock / 1000000) / (freq_div));  // 计算自动重装载值
//	temp = (uint16)65536 - period_temp;

//	TM1PS = freq_div;	// 设置分频值
//	TMOD |= 0x00; 		// 模式 0
//	TL1 = temp;
//	TH1 = temp >> 8;
//	TR1 = 1; 			// 启动定时器
//	ET1 = 1; 			// 使能定时器中断
	// 上面的手工配置已经被逐飞的库函数替代，最终效果一样：定时器 1 每 50us 中断一次。
	// 必须在所有外设都初始化完之后再调用，因为一开中断状态机就开始跑了。
	pit_us_init(TIM1_PIT, 50);
	pit_count = 0;
    motor.run_flag = MOTOR_IDLE;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      LED初始化
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void led_init(void)
{
	gpio_init(IO_P36, GPO, 0, GPO_PUSH_PULL);
	gpio_init(IO_P37, GPO, 0, GPO_PUSH_PULL);

    // 等 GPIO 配置稳定下来再写电平
    system_delay_ms(20);

    // 两灯都写 1 即熄灭，上电默认全灭
    RUN_LED = 1;
    ERR_LED = 1;
}
