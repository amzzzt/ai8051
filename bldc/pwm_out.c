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

#include "intrins.h"
#include "zf_driver_gpio.h"
#include "bldc_config.h"
#include "comparator.h"
#include "pwm_out.h"


// 互补输出开关变量。
// 当前代码中只有定义，没有任何地方真正读取它来改变 PWMA_ENO，
// pit_timer.c 里尝试运行时切换互补输出的那段逻辑是注释掉的。
// 所以实际是否互补由编译期的 BLDC_USR_COMPLEMENTARY 决定。
uint8 g_use_complementary = 0;



//-------------------------------------------------------------------------------------------------------------------
//  六步换相函数表
//
//  数组下标就是 motor.step，换相时执行 pwm_x_output[motor.step]()。
//  顺序不能随意调整，它必须和 comparator_open_isr() 里判断奇偶来选触发沿的逻辑对应：
//      下标偶数 -> 下降沿检测
//      下标奇数 -> 上升沿检测
//  一旦调换其中任意两项的顺序，过零点就会检测反，电机表现为抖动或者直接堵转。
//-------------------------------------------------------------------------------------------------------------------
pwm_x_output_func* pwm_x_output[6] =
{
    pwm_a_bn_output,
    pwm_a_cn_output,
    pwm_b_cn_output,
    pwm_b_an_output,
    pwm_c_an_output,
    pwm_c_bn_output
};



//-------------------------------------------------------------------------------------------------------------------
//  @brief      软延时
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 由 9 个空操作指令拼成的极短延时，用于等 PWM 输出的电平稳定下来。
// 当前工程中没有任何调用点，属于保留函数。
void delay_500ns(void)
{
	_nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
	_nop_();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      刹车（关闭输出并开启所有下桥）
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 下桥全部导通，把电机三相绕组短路，转子转动产生的反电动势被短路电流消耗掉，
// 属于能耗制动，减速很快但不会给母线电容充电。
// 注意上下桥不能同时导通，所以这里先把 PWMA_ENO 清零让 PWM 输出全部脱离引脚，
// 再单独拉高三个下桥。
void pwm_brake(void)
{
    PWMA_ENO = 0;
    PWM_A_L_PIN = 1;
    PWM_B_L_PIN = 1;
    PWM_C_L_PIN = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      关闭输出
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 六路输出全部关闭并拉低，电机断电后自由滑行，不产生制动转矩。
// 与 pwm_brake() 的区别就在这里。
void pwm_close_output(void)
{
    PWM_A_H_PIN = 0;
    PWM_B_H_PIN = 0;
    PWM_C_H_PIN = 0;

    PWM_A_L_PIN = 0;
    PWM_B_L_PIN = 0;
    PWM_C_L_PIN = 0;
    PWMA_ENO = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启A上B下
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// step 0：A 相上桥 PWM，B 相下桥常通，C 相悬空用于检测过零。
// PWMA_ENO 的每一比特对应一路输出，0x01 是 A 相上桥，0x03 是 A 相上桥加下桥。
void pwm_a_bn_output()
{
#if BLDC_USR_COMPLEMENTARY
    PWMA_ENO = 0x03;
#else
    PWMA_ENO = 0x01;
#endif

    // 下桥的三个引脚先全部拉低，再把要导通的那一相拉高，避免换相瞬间上下桥直通
    PWM_A_L_PIN = 0;
    PWM_C_L_PIN = 0;
    PWM_B_L_PIN = 1;

    // 指定比较器去监测悬空的 C 相
    CMP_SELECT_C;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启A上C下
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// step 1：A 相上桥 PWM，C 相下桥常通，B 相悬空。
void pwm_a_cn_output()
{
#if BLDC_USR_COMPLEMENTARY
    PWMA_ENO = 0x03;
#else
    PWMA_ENO = 0x01;
#endif

    PWM_A_L_PIN = 0;
    PWM_B_L_PIN = 0;
    PWM_C_L_PIN = 1;

    CMP_SELECT_B;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启B上C下
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// step 2：B 相上桥 PWM，C 相下桥常通，A 相悬空。
// 上桥从 A 换成 B，PWMA_ENO 相应地从 0x01 换成 0x04。
void pwm_b_cn_output()
{
#if BLDC_USR_COMPLEMENTARY
    PWMA_ENO = 0x0C;
#else
    PWMA_ENO = 0x04;
#endif

    PWM_A_L_PIN = 0;
    PWM_B_L_PIN = 0;
    PWM_C_L_PIN = 1;

    CMP_SELECT_A;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启B上A下
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// step 3：B 相上桥 PWM，A 相下桥常通，C 相悬空。
void pwm_b_an_output()
{
#if BLDC_USR_COMPLEMENTARY
    PWMA_ENO = 0x0C;
#else
    PWMA_ENO = 0x04;
#endif

    PWM_B_L_PIN = 0;
    PWM_C_L_PIN = 0;
    PWM_A_L_PIN = 1;

    CMP_SELECT_C;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启C上A下
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// step 4：C 相上桥 PWM，A 相下桥常通，B 相悬空。
// 上桥换成 C，PWMA_ENO 用 0x10，即第 5 位。
void pwm_c_an_output()
{
#if BLDC_USR_COMPLEMENTARY
    PWMA_ENO = 0x30;
#else
    PWMA_ENO = 0x10;
#endif

    PWM_B_L_PIN = 0;
    PWM_C_L_PIN = 0;
    PWM_A_L_PIN = 1;

    CMP_SELECT_B;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启C上B下
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// step 5：C 相上桥 PWM，B 相下桥常通，A 相悬空。
// 走完这一步再 motor_next_step() 就回到 step 0，完成一轮电周期。
void pwm_c_bn_output()
{
#if BLDC_USR_COMPLEMENTARY
    PWMA_ENO = 0x30;
#else
    PWMA_ENO = 0x10;
#endif

    PWM_A_L_PIN = 0;
    PWM_C_L_PIN = 0;
    PWM_B_L_PIN = 1;

    CMP_SELECT_A;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      更新PWM占空比
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 六步换相每步只有一相上桥在斩波，而且三相轮流出力，所以三相写同一个占空比即可，
// 具体哪一路真正输出由 PWMA_ENO 决定。
// 先算好高低字节再分别写入，避免写到一半时值已经变了。
void pwm_out_duty_update(uint16 duty)
{
    uint8 temp_h, temp_l;

    temp_h = (duty >> 8) & 0xFF;
    temp_l = (uint8)duty;

    PWMA_CCR1H = temp_h;
    PWMA_CCR1L = temp_l;

    PWMA_CCR2H = temp_h;
    PWMA_CCR2L = temp_l;

    PWMA_CCR3H = temp_h;
    PWMA_CCR3L = temp_l;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWM初始化（中心对齐方式）
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
// 必须放在 motor_init() 之后调用。
// 原因是引脚要先由 motor_init() 的 gpio_init 配成推挽输出，PWM 外设才能顺利接管，
// 顺序反了会在上电瞬间出现上下桥同时导通的窗口，直接烧管。
void pwm_out_init(void)
{
    // 先把六路引脚全部拉低并让 PWM 脱离引脚，确保初始化过程中不会有输出
    PWM_A_H_PIN = 0;
	PWM_A_L_PIN = 0;
	PWM_B_H_PIN = 0;
	PWM_B_L_PIN = 0;
	PWM_C_H_PIN = 0;
	PWM_C_L_PIN = 0;

	PWMA_ENO = 0;

	PWMA_CCER1  = 0;
	PWMA_CCER2  = 0;
	PWMA_SR1    = 0;
	PWMA_SR2    = 0;
    PWMA_ENO    = 0;
	PWMA_IER    = 0;

    // 设置PWM引脚
    // 每两位控制一路 PWM 的引脚组选择，映射到 pwm_out.h 里定义的 P00 至 P05。
    // 换引脚时这里要跟着一起改，另外 motor_init() 里的 gpio_init 也要同步。
    PWMA_PS     = 0x55;

    PWMA_CCMR1  = 0x78;		// 通道模式配置, PWM模式2, 预装载允许
	PWMA_CCR1H  = 0;
    PWMA_CCR1L  = 0;
	PWMA_CCER1 |= 0x0F;		// 开启比较输出, 低电平有效

	PWMA_CCMR2  = 0x78;		// 通道模式配置, PWM模式2, 预装载允许
	PWMA_CCR2H  = 0;
    PWMA_CCR2L  = 0;
	PWMA_CCER1 |= 0xF0;		// 开启比较输出, 低电平有效

    PWMA_CCMR3  = 0x78;		// 通道模式配置, PWM模式2, 预装载允许
	PWMA_CCR3H  = 0;
    PWMA_CCR3L  = 0;
	PWMA_CCER2 |= 0x0F;		// 开启比较输出, 低电平有效

//    PWMA_IER    = 0x10;     // 开启更新中
    //PWMA_ENO = 0xff;//0X15;

    // 预分频
    // 不分频，计数器时钟就是 40MHz 的系统时钟
    PWMA_PSCRH = 0;
    PWMA_PSCRL = 0;

    // 设置周期
    // 自动重装载值即 BLDC_PWM_ARR_MAX-1 = 922，一个周期 924 个计数，
    // 40MHz / 924 约等于 43.3kHz，这就是三相 PWM 的载波频率。
    PWMA_ARRH = (uint8)((BLDC_PWM_ARR_MAX-1) >> 8);
    PWMA_ARRL = (uint8)((BLDC_PWM_ARR_MAX-1) & 0xff);

    // 死区时间，防止同一桥臂上下管直通
    PWMA_DTR    = BLDC_PWM_DEADTIME;
    PWMA_BKR    = 0x80;		// 主输出使能 相当于总开关
	PWMA_CR1    = 0x85;		// 使能计数器, 允许自动重装载寄存器缓冲, 边沿对齐, 向上计数, 只有计数器上下溢出才触发更新中断,  bit7=1:写自动重装载寄存器缓冲(本周期不会被打扰), =0:直接写自动重装载寄存器本(周期可能会乱掉)
	PWMA_EGR    = 0x01;		// 产生一次更新事件, 清除计数器和与分频计数器, 装载预分频寄存器的值

//    // 刚开始使用刹车将
//    PWM_A_L_PIN = 1;
//    PWM_B_L_PIN = 1;
//    PWM_C_L_PIN = 1;
}
