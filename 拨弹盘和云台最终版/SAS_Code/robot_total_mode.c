/**
 * @file robot_total_mode.c
 * @author ��׿ѫ ����� UCAS-SAS �й���ѧԺ��ѧ�����˶�
 * @brief ����ң��������жϲ��������˵���״̬�������Ƿ��Զ���������̨�����̡�����������������״̬�жϷ�״̬����֤һ����
 *          ���⣬��.c�ļ��ж����������¼robotAuto״̬��
 * @version 0.1
 * @date 2022-01-28 
 * 
 * @version 0.2
 * @date 2022-02-05 �޸�Ϊ�ĸ�ģʽ����������ͨ����С���ݳ�����yaw��λ��
 *                                 ����    C       V        B
 *                                          ��      ��      ��
 * �����̿�ʼ���ƺ�ң�������˾���ʱʧȥ�����ˡ�������״̬�����ı䣬��ô������̿��Ʊ�ǣ�ң�������»�ø���״̬��Ȩ��
 * 
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "cmsis_os.h"
#include "robot_total_mode.h"
#include "remote_control.h"
#include "nucCommu.h"
#include "OLED.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "RobotStructure.h"
#include "main.h"
//***************debug����*******************************//
// #define MODE_SWITCH_VERSION_1       //��һ�汾��ͨ��ң�������˱任״̬2022��1��28�� ֻ��sticky yaw

//OLED�������
// #define OLED_DEBUG_TOTAL_MODE_ON
#ifdef OLED_DEBUG_TOTAL_MODE_ON
#define OLED_REFRESH_TIME 300
#endif

//***************ģʽ������غ궨��***********************//
#define RC_ROBOT_MODE_CHANNEL 0     //�ұߵĲ��˿���ģʽѡ�񡣵�һ�汾�ͨ��λ�õ�ͣ��ʵ��ģʽ�ı䣬֮�������λ���ܱ仯������

//***************������غ궨��***************************//
#define ROBOT_MODE_TASK_TIME 10    //10msִ��һ�Σ���Ϊ�漰���ͼ���״̬�Ķ�ȡ��������Ҫ�졣

//***************�������ñ���**************************//
#ifdef OLED_DEBUG_TOTAL_MODE_ON
static int time_passed_debug=0;
#endif

//***************ȫ�ֱ���*******************************//
static int robotAuto=0;     //���ڼ�
static const RC_ctrl_t *rc_p;   //ң����λ��ָ�룬��Ҫ��ʼ��
static const toSTM32_t *nuc_p;  //NUC����λ�� 
static enum RobotState_e robotState=RobotState_e_Powerless;  //�����˳�ʼ״̬
static uint8_t keyBoardAndMouseHasChanged=0;        //��¼�Ƿ��м���������
                                                //���Ĺ����Ǳ���ң������ָ��Ѽ�������ָ�����
                                                //���ֻ��Ҫ��ע��ң������������ͻ��״̬���Ƶļ�����

/* ���������κΰ�����ʼ���º󣬾�ת�������̺��������ˣ�ң��������ʧЧ����ң�������ϲ��˷���
״̬�ı����ң��������ʱ����������¼�����°�ң��������Ϊ׼��
*/ 

void robot_total_mode_task(void const *pvParameters)
{
    //��ʼ������ָ��
    rc_p=get_remote_control_point();
    nuc_p=get_nuc_control_point();
    int8_t lastMode,nowMode;
    nowMode=rc_p->rc.s[RC_ROBOT_MODE_CHANNEL];
    robotAuto=0;
    while (1)
    {
        //*****************************OLED ��ʾ����******************//
        #ifdef OLED_DEBUG_TOTAL_MODE_ON
        time_passed_debug+=ROBOT_MODE_TASK_TIME;
        if(time_passed_debug>OLED_REFRESH_TIME)
        {
            time_passed_debug=0;
            //*********����͸���******************
            OLED_printf(0,0,"                    ");
            OLED_printf(1,0,"                    ");
            OLED_printf(2,0,"                    ");
            OLED_printf(3,0,"                    ");
            OLED_printf(4,0,"                    ");
            OLED_printf(0,0,"robot mode:%d",getRobotPresentMode());
            OLED_printf(1,0,"key:%d",keyBoardAndMouseHasChanged);
            OLED_refresh_gram();
            //***********************************
        }
        #endif
        //***********************************************************//

        //*****************************��һ�汾��ͨ��ң�������˱任״̬**//
        // NUC����ʱ��ǿ�Ʊ�Ϊ�ֶ�ģʽ
        // ң��������ʱ��ǿ�ƻص�PowerLess״̬
        // ���������κΰ�����ʼ���º󣬾�ת�������̺���������
        // ����Z�����£������Զ���׼ģʽ��ƴ��zi���ԡ�����
        // ����X�����£��ر��Զ���׼ģʽ�������Զ���׼��ȥ�ˣ�

        lastMode=nowMode;
        nowMode=rc_p->rc.s[RC_ROBOT_MODE_CHANNEL];
        if(lastMode!=nowMode)
            keyBoardAndMouseHasChanged=0;
        if(!keyBoardAndMouseHasChanged) //����״̬���
        {
            if(switch_is_mid(rc_p->rc.s[RC_ROBOT_MODE_CHANNEL]))
            {
                robotState=RobotState_e_CommonCar;
            }
            else if(switch_is_down(rc_p->rc.s[RC_ROBOT_MODE_CHANNEL]))
            {
                robotState=RobotState_e_Spinner;
            }
            else if(switch_is_up(rc_p->rc.s[RC_ROBOT_MODE_CHANNEL]))
            {
                
                robotState=RobotState_e_BadYawCar;
            }
        }  
        //̽��������״̬
        if(rc_p->key.v & KEY_PRESSED_OFFSET_Z)
        {
            robotAuto=1;
            keyBoardAndMouseHasChanged=1;
					  //usart_printf("yes\r\n");
					  
        }
        if(rc_p->key.v & KEY_PRESSED_OFFSET_X)
        {
            robotAuto=0;
            keyBoardAndMouseHasChanged=1;
        }
        if(rc_p->key.v & KEY_PRESSED_OFFSET_C)  //c������ͨ��
        {
            robotState=RobotState_e_CommonCar;
            keyBoardAndMouseHasChanged=1;
        }
        if(rc_p->key.v & KEY_PRESSED_OFFSET_V)  //v����С����
        {
            robotState=RobotState_e_Spinner;
            keyBoardAndMouseHasChanged=1;
        }
        if(rc_p->key.v & KEY_PRESSED_OFFSET_B)  //b������yaw����
        {
            robotState=RobotState_e_BadYawCar;
            keyBoardAndMouseHasChanged=1;
        }
				 if(rc_p->mouse.press_r||rc_p->rc.ch[4]>500)
				{
					robotAuto=1;
				}
				else
				{
					robotAuto=0;
				}
        //***********************************************************//


        //*********************���߼��*******************************//
        if(toe_is_error(MINI_PC_TOE))
            robotAuto=0;
        if(toe_is_error(DBUS_TOE))
        {
            robotState=RobotState_e_Powerless;
            keyBoardAndMouseHasChanged=0;
        }
            
        //*************************************************************

        osDelay(ROBOT_MODE_TASK_TIME);
    }
}

int robotIsAuto(void)
{
    return robotAuto;
}

const enum RobotState_e * getRobotPresentMode(void)
{
    return &robotState;
}

uint8_t getKeyHasChanged (void)
{
    return keyBoardAndMouseHasChanged;
}

void OLED_mode(void)
{
    OLED_printf(0,0,"Auto:%d",robotIsAuto());
}
