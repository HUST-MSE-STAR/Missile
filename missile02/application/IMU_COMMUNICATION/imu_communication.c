#include "imu_communication.h"
#include "main.h"
#include "bsp_usart.h"
#include "string.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

imu_info imu_data;


//����ԭʼ���ݣ�Ϊ11���ֽڣ�����22���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t imu_buf[2][IMU_RX_BUF_NUM];


void imu_communication_init(void)
{
    usart6_rx_dma_init(imu_buf[0], imu_buf[1], IMU_RX_BUF_NUM);
}
void receive_imu_data(void)
{
	//�ж��Ƿ�����ж�
  if(USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;
       
        __HAL_UART_CLEAR_PEFLAG(&huart6);
         //�жϽ��н��յĻ�������0�Ż���������1�Ż�����  DMA_SxCR_CT=1
        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = IMU_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
            //�����趨���ݳ���
            hdma_usart6_rx.Instance->NDTR = IMU_RX_BUF_NUM;
            //�趨������1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            
            if(this_time_rx_len == IMU_FRAME_LENGTH)
            {
							  //����ң��������
                get_imu_data(&imu_data,imu_buf[0]);
            }
        }
        else
        {
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = IMU_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
            //�����趨���ݳ���
            hdma_usart6_rx.Instance->NDTR = IMU_RX_BUF_NUM;
            //�趨������0
            hdma_usart6_rx.Instance->CR &= ~(DMA_SxCR_CT);
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if(this_time_rx_len == IMU_FRAME_LENGTH)
            {
                //����ң��������
                get_imu_data(&imu_data,imu_buf[1]);
            }
        }
    }
}


