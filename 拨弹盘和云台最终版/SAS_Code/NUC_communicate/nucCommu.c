#include "nucCommu.h"
#include "main.h"
#include "bsp_nucCom.h"
// #include "monitor_task.h"
#include "struct_typedef.h"
#include "detect_task.h"
#include "OLED.h"
#include "robot_total_mode.h"
#include <stdlib.h>
#include <string.h>
#include "bsp_buzzer.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

static void sbus_to_nucCtrl(volatile const uint8_t *sbus_buf, toSTM32_t *nuc_info);

// nuc control data
// nuc���Ʊ���
// ��ͨ�����ԣ����������bug
toSTM32_t GimbalRxMsg;

// added at 2022��1��26��23��54�֣�NUC��Чʱ��
static uint32_t lastValidTime = 0;
static int dataIsValid = 0;
static int buzzer_time;
// added at 2022��1��26��23��54�֣�NUC��Чʱ�� ���ƺ���
void setNUCValid(void)
{
    dataIsValid = 1;
    return;
}

void setNUCInvalid(void)
{
    dataIsValid = 0;
}

uint32_t getNUCLastValidTime_ms(void)
{
    return lastValidTime;
}

int NUCIsValid(void)
{
    return dataIsValid;
}

// receive data, 44 bytes one frame, but set 88 bytes
// ����ԭʼ���ݣ�Ϊ44���ֽڣ�����88���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t nucinfo_rx_buf[2][NUCINFO_RX_BUF_NUM];
// �ṩ�߲�η�װ������ֱ�����ýϵͲ��ͷ�ļ�
void nuc_control_init(void)
{
    NUC_com_init(nucinfo_rx_buf[0], nucinfo_rx_buf[1], NUCINFO_RX_BUF_NUM);
}

/**
 * @brief          ��ȡnuc����ָ��
 * @param[in]      none
 * @retval         nuc����ָ��
 */
const toSTM32_t *get_nuc_control_point(void)
{
    return &GimbalRxMsg;
}

void Encode(uint8_t RawData[33], enum Robot_ID self_id, enum Robot_Colors self_color, fp32 pos_x, fp32 pos_y, fp32 gimbal_yaw, fp32 gimbal_pitch, fp32 ballet_speed, enum Auto_Num is_auto)
{
    RawData[0] = 0xE7;
    RawData[1] = 0x7E;

    int32_t id = (int32_t) self_id;
    int32_t color = (int32_t) self_color;
	  uint8_t is_A=(uint8_t)is_auto;
    memcpy(&RawData[2], &id, sizeof(int32_t));
    memcpy(&RawData[6], &color, sizeof(int32_t));

    memcpy(&RawData[10], &pos_x, sizeof(fp32));
    memcpy(&RawData[14], &pos_y, sizeof(fp32));
    memcpy(&RawData[18], &gimbal_yaw, sizeof(fp32));
    memcpy(&RawData[22], &gimbal_pitch, sizeof(fp32));
    memcpy(&RawData[26], &ballet_speed, sizeof(fp32));
    
	  memcpy(&RawData[30],&is_auto ,sizeof(uint8_t));
	
    RawData[31] = 0x00;
    RawData[32] = 0xEE;
}



// �����ж�
extern TIM_HandleTypeDef htim5;
void USART1_IRQHandler(void)
{
    if (huart1.Instance->SR & UART_FLAG_RXNE) // ���յ�����//�����ж�
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if (USART1->SR & UART_FLAG_IDLE) // �����ж�
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = NUCINFO_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            // reset set_data_lenght
            // �����趨���ݳ���
            hdma_usart1_rx.Instance->NDTR = NUCINFO_RX_BUF_NUM;

            // set memory buffer 1
            // �趨����1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if (this_time_rx_len == NUCINFO_FRAME_LENGTH)
            {
                if(buzzer_time++<3)
									buzzer_on(1,300);
								else
									buzzer_off();
								
                sbus_to_nucCtrl(nucinfo_rx_buf[0], &GimbalRxMsg);
                //usart_printf("%d,%d,%d,%f,%f,%f,%f\r\n",GimbalRxMsg.is_fire,GimbalRxMsg.is_spin,GimbalRxMsg.recognized,GimbalRxMsg.yaw.data,GimbalRxMsg.pitch.data,GimbalRxMsg.velocity_x.data,GimbalRxMsg.velocity_y.data);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = NUCINFO_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            // reset set_data_lenght
            // �����趨���ݳ���
            hdma_usart1_rx.Instance->NDTR = NUCINFO_RX_BUF_NUM;

            // set memory buffer 0
            // �趨����??0
            DMA2_Stream5->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if (this_time_rx_len == NUCINFO_FRAME_LENGTH)
            {
                // ����nuc��������
							//���ڴ���ͨ�����Ƽ���Ƿ�
                //__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0x00000000);
								if(buzzer_time++<3)
										buzzer_on(1,300);
								else
									buzzer_off();
                sbus_to_nucCtrl(nucinfo_rx_buf[1], &GimbalRxMsg);
                //usart_printf("%d,%d,%d,%f,%f,%f,%f\r\n",GimbalRxMsg.is_fire,GimbalRxMsg.is_spin,GimbalRxMsg.recognized,GimbalRxMsg.yaw.data,GimbalRxMsg.pitch.data,GimbalRxMsg.velocity_x.data,GimbalRxMsg.velocity_y.data);
            }
        }
    }
}

/**
 * @brief          nuc����Э�����
 * @param[in]      sbus_buf: ԭ������ָ��
 * @param[out]     rc_ctrl: nuc����ָ��
 * @retval         none
 */

// ���ݽ����������ԵĴ��ݹ������Ƿ�Ϊ��ȷ�ģ������Ƿ�ᷴ����??

static void sbus_to_nucCtrl(volatile const uint8_t *sbus_buf, toSTM32_t *nuc_ctrl)
{
    if (sbus_buf[0] == SOF1 && sbus_buf[1] == SOF2&&sbus_buf[19]==0x00&&sbus_buf[20]==0xEE)
    {
        nuc_ctrl->yaw.bytes[0] = sbus_buf[3];
        nuc_ctrl->yaw.bytes[1] = sbus_buf[4];
        nuc_ctrl->yaw.bytes[2] = sbus_buf[5];
        nuc_ctrl->yaw.bytes[3] = sbus_buf[6];

        nuc_ctrl->pitch.bytes[0] = sbus_buf[7];
        nuc_ctrl->pitch.bytes[1] = sbus_buf[8];
        nuc_ctrl->pitch.bytes[2] = sbus_buf[9];
        nuc_ctrl->pitch.bytes[3] = sbus_buf[10];

        // Ctrl CMD decode
        nuc_ctrl->is_spin = (sbus_buf[2] & 0x01) != 0;
        nuc_ctrl->is_fire= (sbus_buf[2] & 0x02) != 0;
        nuc_ctrl->recognized = (sbus_buf[2] & 0x04) != 0;

        nuc_ctrl->velocity_x.bytes[0] = sbus_buf[11];
        nuc_ctrl->velocity_x.bytes[1] = sbus_buf[12];
        nuc_ctrl->velocity_x.bytes[2] = sbus_buf[13];
        nuc_ctrl->velocity_x.bytes[3] = sbus_buf[14];

        nuc_ctrl->velocity_y.bytes[0] = sbus_buf[15];
        nuc_ctrl->velocity_y.bytes[1] = sbus_buf[16];
        nuc_ctrl->velocity_y.bytes[2] = sbus_buf[17];
        nuc_ctrl->velocity_y.bytes[3] = sbus_buf[18];
				
				
    }
}

// �ҵ��˷���1��δ�ҵ��򷵻�0.�����͵�������ʼΪ'S'��Ϊ�ҵ�
// int foundArmor(void)
//{
//     if(toe_is_error(MINI_PC_TOE))
//         return 0;
//     if(((char)(nuc_ctrl.startFlag))==SEND_START)
//         return 1;
//     else
//         return 0;
// }

// void OLED_nuc(void)
//{
//     OLED_printf(0,0,"a:%d,f:%d,pitch:%f,yaw:%f",robotIsAuto(),foundArmor(),GimbalRxMsg.pitch,GimbalRxMsg.yaw);
//     //����̫�࣬����ȥ����
//     // OLED_printf(0,0,"s:%c,task:%d,tht:%f,phi:%f,r:%f,color:%d,end:%c,auto:%d",
//     // (char)(nuc_ctrl.startFlag),nuc_ctrl.taskType,nuc_ctrl.theta,nuc_ctrl.phi,nuc_ctrl.r,nuc_ctrl.detectedColor,(char)(nuc_ctrl.endFlag),robotIsAuto());
// }

// void OLED_nuc(void)
// {
//     OLED_printf(0,0,"s:%c,task:%d,tht:%.1f,phi%.1f",
//     (char)(nuc_ctrl.startFlag),nuc_ctrl.taskType,nuc_ctrl.theta,nuc_ctrl.phi);
// }