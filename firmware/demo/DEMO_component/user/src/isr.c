/*********************************************************************************************************************
* 本文件基于逐飞科技 RT1064DVL6A 开源库（Copyright (c) 2022 SEEKFREE 成都逐飞科技有限公司），GPL3.0 协议。
* 修改时请保留逐飞科技的版权声明；完整条款见 libraries/doc 下的 GPL3_permission_statement.txt 与 LICENSE。
* 文件名：isr        目标平台：RT1064DVL6A
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "isr.h"
#include "imu.h"
#include "state_machine.h"
#include "serial_link.h"




void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();     // 调用SDK自带的中断函数 这个函数最后会调用我们设置的回调函数
    __DSB();                    // 数据同步隔离
}

extern void pit_handler(void);

void PIT_IRQHandler(void)
{
    if(pit_flag_get(PIT_CH))
    {
      pit_flag_clear(PIT_CH);
			pit_handler();
    }
    
    if(PIT_CH != PIT_CH1 && pit_flag_get(PIT_CH1))
    {
        pit_flag_clear(PIT_CH1);
    }
    
    if(PIT_CH != PIT_CH2 && pit_flag_get(PIT_CH2))
    {
        pit_flag_clear(PIT_CH2);
    }
    
    if(PIT_CH != PIT_CH3 && pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
    }

    __DSB();
}

void PendSV_Handler(void)
{
    state_machine_transition_irq();
}

void DMA_ERROR_IRQHandler(void)
{
    serial_link_dma_error_irq();
}

void LPUART1_IRQHandler(void)
{
    serial_link_uart_irq();
}

void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        // 接收中断 
        if(NULL != flexio_camera_uart_handler)
        {
            flexio_camera_uart_handler();
        }
        
		// gps_uart_callback();
		// 本库已删除 zf_device_gps_tau1201 驱动，这个回调不存在，取消注释会编译报错
    }
        
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // 接收中断
        if(NULL != camera_uart_handler)
        {
            camera_uart_handler();
        }
    }
        
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);    // 不允许删除
}


void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // 接收中断
        if(NULL != wireless_module_uart_handler)
        {
            wireless_module_uart_handler();
        }
        
    }
        
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);    // 不允许删除
}


void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0))
    {
        exti_flag_clear(B0);// 清除中断标志位
    }
    
}


void GPIO1_Combined_16_31_IRQHandler(void)
{
    if(exti_flag_get(B16))
    {
        exti_flag_clear(B16); // 清除中断标志位
    }

    
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    // 勿删除此IF语句
    if(NULL != flexio_camera_vsync_handler)
    {
        flexio_camera_vsync_handler();
    }
    
    
    if(exti_flag_get(C0))
    {
        exti_flag_clear(C0);// 清除中断标志位
    }

}


void GPIO2_Combined_16_31_IRQHandler(void)
{
    // -----------------* ToF INT 更新中断 预置中断处理函数 *-----------------
    tof_module_exti_handler();
    // -----------------* ToF INT 更新中断 预置中断处理函数 *-----------------
    
    if(exti_flag_get(C16))
    {
        exti_flag_clear(C16); // 清除中断标志位
    }
    
    
}




void GPIO3_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);// 清除中断标志位
    }
}












