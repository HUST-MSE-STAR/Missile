#include "imu_communication.h"
#include "main.h"
#include "bsp_usart.h"
#include "string.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

imu_info imu_data;


//接收原始数据，为11个字节，给了22个字节长度，防止DMA传输越界
static uint8_t imu_buf[2][IMU_RX_BUF_NUM];


void imu_communication_init(void)
{
    usart6_rx_dma_init(imu_buf[0], imu_buf[1], IMU_RX_BUF_NUM);
}
void receive_imu_data(void)
{
	//判断是否空闲中断
  if(USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;
       
        __HAL_UART_CLEAR_PEFLAG(&huart6);
         //判断进行接收的缓冲区是0号缓冲区还是1号缓冲区  DMA_SxCR_CT=1
        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = IMU_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = IMU_RX_BUF_NUM;
            //设定缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            
            if(this_time_rx_len == IMU_FRAME_LENGTH)
            {
							  //处理遥控器数据
                get_imu_data(&imu_data,imu_buf[0]);
            }
        }
        else
        {
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = IMU_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = IMU_RX_BUF_NUM;
            //设定缓冲区0
            hdma_usart6_rx.Instance->CR &= ~(DMA_SxCR_CT);
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if(this_time_rx_len == IMU_FRAME_LENGTH)
            {
                //处理遥控器数据
                get_imu_data(&imu_data,imu_buf[1]);
            }
        }
    }
}


