/*********************************************************************************************************************
* AI8051U Opensourec Library 即（AI8051U 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是STC 开源库的一部分
*
* AI8051U 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          AI8051U
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2024-08-01        大W            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"


//-------------------------------------------------------------------------------------------------------------------
//  中断服务函数集合
//
//  这个文件里的函数都是逐飞库的中断入口。库本身把中断处理好之后，用一个函数指针
//  的形式暴露出来（比如 uart1_irq_handler），用户只要给这个指针赋值就能挂上自己的
//  回调，不需要改动库代码。
//
//  本工程实际上只用到其中的串口接收中断，其余为库的默认实现，保留不动。
//
//  【注意】
//  下面出现的 interrupt N 里的 N 是 251 的中断向量号，不是随便编的数字。
//  文件末尾有一整张向量号对照表（已注释），改中断号之前先查那张表。
//-------------------------------------------------------------------------------------------------------------------


// UART1 接收中断，向量号 4。
// 逐飞库的串口接收走 DMA 自动搬运，这里的标志位含义：
//   DMA_UR1R_STA bit0 (0x01) = 接收完成
//   DMA_UR1R_STA bit1 (0x02) = 数据被覆盖丢弃
void DMA_UART1_IRQHandler (void) interrupt 4
{
    static vuint8 dwon_count = 0;
    if (DMA_UR1R_STA & 0x01)		// 接收完成
    {
        DMA_UR1R_STA &= ~0x01;		// 清标志位
        uart_rx_start_buff(UART_1);	// 设置下一次接收，务必保留

        //程序自动下载
        // 上位机下载工具会连续发 0x7F 作为握手信号，攒够 21 个就触发一次软复位，
        // 于是不用按复位键也能进下载模式。这是 STC 单片机的标准下载流程。
        if(uart_rx_buff[UART_1][0] == 0x7F)
        {
            if(dwon_count++ > 20)
            {
                // IAP_CONTR 写 0x60：软复位并停留在 ISP 监控区，等待重新下载程序
                IAP_CONTR = 0x60;
            }
        }
        else
        {
            // 收到任何别的数据就重新计数，避免正常通信中零星出现 0x7F 误触发复位
            dwon_count = 0;
        }

        // 把收到的字节交给用户回调。回调指针默认为空，用户没挂就什么都不做。
        // 注意这里只传了 buffer 的第一个字节，也就是每次只处理一个字节。
        if(uart1_irq_handler != NULL)
        {
            uart1_irq_handler(uart_rx_buff[UART_1][0]);
        }
    }

    if (DMA_UR1R_STA & 0x02)	//数据丢弃
    {
        DMA_UR1R_STA &= ~0x02;	//清标志位
        uart_rx_start_buff(UART_1);	// 设置下一次接收，务必保留
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 简单说就是接收速度超过了主程序的处理速度，数据丢了。
        // 真机调试时如果频繁进这里，说明上位机发得太快，需要加流控或者提高处理频率。
    }
}



// UART2 接收中断，向量号 8。结构与 UART1 相同，但没有下载握手那段。
void DMA_UART2_IRQHandler (void) interrupt 8
{

    if (DMA_UR2R_STA & 0x01)		// 接收完成
    {
        DMA_UR2R_STA &= ~0x01;		// 清标志位
        uart_rx_start_buff(UART_2);	// 设置下一次接收，务必保留

        if(uart2_irq_handler != NULL)
        {
            uart2_irq_handler(uart_rx_buff[UART_2][0]);
        }
    }

    if (DMA_UR2R_STA & 0x02)		//数据丢弃
    {
        DMA_UR2R_STA &= ~0x02;		//清标志位
        uart_rx_start_buff(UART_2);	// 设置下一次接收，务必保留
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!

    }
}

// UART3 接收中断，向量号 17
void DMA_UART3_IRQHandler (void) interrupt 17
{

    if (DMA_UR3R_STA & 0x01)		// 接收完成
    {
        DMA_UR3R_STA &= ~0x01;		// 清标志位
        uart_rx_start_buff(UART_3);	// 设置下一次接收，务必保留

        if(uart3_irq_handler != NULL)
        {

            uart3_irq_handler(uart_rx_buff[UART_3][0]);

        }
    }

    if (DMA_UR3R_STA & 0x02)		//数据丢弃
    {
        DMA_UR3R_STA &= ~0x02;		//清标志位
        uart_rx_start_buff(UART_3);	// 设置下一次接收，务必保留
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!

    }
}

// UART4 接收中断，向量号 18
void DMA_UART4_IRQHandler (void) interrupt 18
{

    if (DMA_UR4R_STA & 0x01)		// 接收完成
    {
        DMA_UR4R_STA &= ~0x01;		// 清标志位
        uart_rx_start_buff(UART_4);	// 设置下一次接收，务必保留

        if(uart4_irq_handler != NULL)
        {
            uart4_irq_handler(uart_rx_buff[UART_4][0]);

        }
    }

    if (DMA_UR4R_STA & 0x02)	//数据丢弃
    {
        DMA_UR4R_STA &= ~0x02;	//清标志位
        uart_rx_start_buff(UART_4);	// 设置下一次接收，务必保留
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!

    }
}


//-------------------------------------------------------------------------------------------------------------------
//  以下是被注释掉的库默认中断入口。
//
//  被注释掉是有原因的，不是随手删的：
//  定时器 0 的中断已经被 bldc/motor_control.c 的 TM0_Isr 占用（换相延时），
//  定时器 4 的中断已经被 bldc/motor_control.c 的 TM4_Isr 占用（换相超时保护）。
//  同一个中断向量在 C251 里只能有一个函数响应，如果这里放开，
//  就会和电机控制的中断冲突，编译会报重复定义或者行为异常。
//  定时器 1 的中断在 bldc/pit_timer.c 里以 TM1_Isr 实现，同理不能放开。
//
//  所以这几个入口没有直接删除，而是留成注释，
//  既保证库的完整性，也提醒后来者这里的中断已经被电机模块接管了。
//-------------------------------------------------------------------------------------------------------------------
//void TM0_IRQHandler() interrupt 1
//{
//    TIM0_CLEAR_FLAG;
//
//    if(tim0_irq_handler != NULL)
//    {
//        tim0_irq_handler();
//    }
//}
//void TM1_IRQHandler() interrupt 3
//{
//    TIM1_CLEAR_FLAG;
//
//    if(tim1_irq_handler != NULL)
//    {
//        tim1_irq_handler();
//    }
//}
// 定时器 2 中断，向量号 12
void TM2_IRQHandler() interrupt 12
{
    TIM2_CLEAR_FLAG;

    if(tim2_irq_handler != NULL)
    {
        tim2_irq_handler();
    }
}
// 定时器 3 中断，向量号 19。注意 pit_timer.c 里用 IE2 &= ~0x20 关掉了它，
// 所以即使挂上回调也不会被执行。
void TM3_IRQHandler() interrupt 19
{
    TIM3_CLEAR_FLAG;

    if(tim3_irq_handler != NULL)
    {
        tim3_irq_handler();
    }
}

//void TM4_IRQHandler() interrupt 20
//{
//    TIM4_CLEAR_FLAG;
//
//    if(tim4_irq_handler != NULL)
//    {
//        tim4_irq_handler();
//    }
//}

// 定时器 11 中断，向量号 24
void TM11_IRQHandler() interrupt 24
{
    TIM11_CLEAR_FLAG;

    if(tim11_irq_handler != NULL)
    {
        tim11_irq_handler();
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  251 中断向量号对照表（原始注释，保留备查）
//
//  interrupt 后面跟的数字必须和这张表对上。注意其中有两处和常见认知不同：
//  定时器 3 是 19 而不是 13，定时器 4 是 20 而不是 14，写中断函数时容易搞错。
//-------------------------------------------------------------------------------------------------------------------
//#define     INT0_VECTOR             0       //0003H
//#define     TMR0_VECTOR             1       //000BH
//#define     INT1_VECTOR             2       //0013H
//#define     TMR1_VECTOR             3       //001BH
//#define     UART1_VECTOR            4       //0023H
//#define     ADC_VECTOR              5       //002BH
//#define     LVD_VECTOR              6       //0033H
//#define     PCA_VECTOR              7       //003BH
//#define     UART2_VECTOR            8       //0043H
//#define     SPI_VECTOR              9       //004BH
//#define     INT2_VECTOR             10      //0053H
//#define     INT3_VECTOR             11      //005BH
//#define     TMR2_VECTOR             12      //0063H
//#define     USER_VECTOR             13      //006BH
//#define     INT4_VECTOR             16      //0083H
//#define     UART3_VECTOR            17      //008BH
//#define     UART4_VECTOR            18      //0093H
//#define     TMR3_VECTOR             19      //009BH
//#define     TMR4_VECTOR             20      //00A3H
//#define     CMP_VECTOR              21      //00ABH
//#define     I2C_VECTOR              24      //00C3H
//#define     USB_VECTOR              25      //00CBH
//#define     PWMA_VECTOR             26      //00D3H
//#define     PWMB_VECTOR             27      //00DBH

//#define     RTC_VECTOR              36      //0123H
//#define     P0INT_VECTOR            37      //012BH
//#define     P1INT_VECTOR            38      //0133H
//#define     P2INT_VECTOR            39      //013BH
//#define     P3INT_VECTOR            40      //0143H
//#define     P4INT_VECTOR            41      //014BH
//#define     P5INT_VECTOR            42      //0153H
//#define     P6INT_VECTOR            43      //015BH
//#define     P7INT_VECTOR            44      //0163H
//#define     DMA_M2M_VECTOR          47      //017BH
//#define     DMA_ADC_VECTOR          48      //0183H
//#define     DMA_SPI_VECTOR          49      //018BH
//#define     DMA_UR1T_VECTOR         50      //0193H
//#define     DMA_UR1R_VECTOR         51      //019BH
//#define     DMA_UR2T_VECTOR         52      //01A3H
//#define     DMA_UR2R_VECTOR         53      //01ABH
//#define     DMA_UR3T_VECTOR         54      //01B3H
//#define     DMA_UR3R_VECTOR         55      //01BBH
//#define     DMA_UR4T_VECTOR         56      //01C3H
//#define     DMA_UR4R_VECTOR         57      //01CBH
//#define     DMA_LCM_VECTOR          58      //01D3H
//#define     LCM_VECTOR              59      //01DBH
//#define     DMA_I2CT_VECTOR         60      //01E3H
//#define     DMA_I2CR_VECTOR         61      //01EBH
//#define     I2S_VECTOR              62      //01F3H
//#define     DMA_I2ST_VECTOR         63      //01FBH
//#define     DMA_I2SR_VECTOR         64      //0203H
//#define     DMA_QSPI_VECTOR         65      //020BH
//#define     QSPI_VECTOR             66      //0213H
//#define     TMR11_VECTOR            67      //021BH
//#define     DMA_PWMAT_VECTOR        72      //0243H
//#define     DMA_PWMAR_VECTOR        73      //024BH
