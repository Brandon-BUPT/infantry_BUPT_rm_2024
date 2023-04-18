#include "main.h"
#include "can.h"
#include "gpio.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "robot_total_mode.h"
#include "INS_task.h"
#include "remote_control.h"
#include "naive_gimbal_task.h"
#include "nucCommu.h"
#include "user_lib.h"
#include "pid.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
struct milestoneStack_s {
    uint8_t head;
    uint8_t stack[2+1];
};
struct gimbalMotorCtrl_s{
    //rads
    const fp32 * anglePoint;          //����̬��������õ��ĽǶ�λ��
    fp32 nowAbsoluteAngle;      //��ǰ��̬��
    uint32_t nowTime;           //��ǰʱ��
    fp32 lastAbsoluteAngle;     //�ϴ���̬��
    uint32_t lastTime;          //�ϴ�ʱ��
    fp32 radSpeed;              //���ٶ� rad/s
    fp32 wantedAbsoluteAngle;   //Ŀ����̬��

    //ECDs
    const uint16_t * ECDPoint;        //ECD��������λ��
    uint16_t maxECD,minECD,middleECD,nowECD;    //�����С����ֵ����ǰECD
    int16_t nowRounds;          //����ת����Ȧ��
    int16_t maxRounds;          //ת�����������Ȧ��
    struct milestoneStack_s mstack; //��Ȧ����¼��ر���

    //PIDs
    pid_type_def spd_pid;       //���ڿ����ٶȵ�PID����������ٶ� rad/s, ���GM6020��ѹ int16
    pid_type_def agl_pid;       //���ڿ��ƽǶȵ�PID��������Ƕ� rad,������ٶ�rad/s

    //������
    int16_t giveVolt;       //GM6020���͵�ѹ
    uint8_t zeroVoltMark;

    // һ���˲��������˽��ٶ�
    first_order_filter_type_t spd_filter;
};

//��������Ϣ��ȡ��ʵ���Ǻ�
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
�������, 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
4:yaw��̨��� 6020���; 5:pitch��̨��� 6020���; 6:������� 2006���*/
//6:������� 2006��� ��������

//���ڴ��������Ϣ���ṩ�����ӿڹ����ʹ��
static can_send_referee_s gimbal_rx_referee;
static motor_measure_t motor_chassis[7];
static motor_measure_t motor_shoot[2];
static const fp32* yaw_angle;
static const enum RobotState_e* robotMode;
static const struct gimbalMotorCtrl_s* set_yaw;

//�Ͳ�εĹ�������
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];

static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];

static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];


//�ص�����������can����Ϣ��ȡ����fifo�ṹ�ڡ�
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            static uint8_t i = 0;
            //get motor id
            //��������ʵ��ʹ�õ��޸�
            i = rx_header.StdId - CAN_3508_M1_ID;
            if((hcan==&SHOOT_CAN)&&(i==0||i==1))
            {
              get_motor_measure(&motor_shoot[i],rx_data);
              break;
            }

            get_motor_measure(&motor_chassis[i], rx_data);
            detect_hook(CHASSIS_MOTOR1_TOE + i);
            break;
        }
				case 0x004:
				{
						typedef union
						{
							can_send_referee_s data_s;
							uint8_t data_uint8_referee[sizeof(can_send_referee_s)];
						}data_u_referee;
						data_u_referee data_0;
						for(int i=0;i<sizeof(can_send_referee_s);i++){
							data_0.data_uint8_referee[i]=rx_data[i];
						}
						gimbal_rx_referee.shootspeed=data_0.data_s.shootspeed;
						gimbal_rx_referee.robot_color=data_0.data_s.robot_color;
						break;
				}
        default:
        {
            break;
        }
    }
}

/**
  * @brief          ������̨������Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          ����̷���IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */



static const RC_ctrl_t *rc_chassis;

//can2 ����
static CAN_TxHeaderTypeDef  can1_tx_message;
static CAN_TxHeaderTypeDef  can1_tx_channel_message;
static CAN_TxHeaderTypeDef  can2_tx_message;
static CAN_TxHeaderTypeDef  can1_tx_nuc_message;
static CAN_TxHeaderTypeDef  can1_tx_keyboard_message;

static uint8_t can1_send_data[8];
static uint8_t can2_send_data[8];
static uint8_t can1_send_data_channel[8];
static uint8_t can1_send_data_nuc_yaw[8];
static uint8_t can1_send_data_keyboard[8];

//�������yaw���Ƶ������Ϣ
typedef struct
{
	fp32 yaw;
	fp32 nuc_yaw;
}can_send_data_s;
void CAN1_send_yaw()
{
	uint32_t send_mail_box;
	typedef union
	{
		can_send_data_s data;
		uint8_t data_uint8[sizeof(can_send_data_s)];
	}can_send_data_u;
	can_send_data_u data_u;
	data_u.data.yaw=*get_INS_angle_point();
	data_u.data.nuc_yaw=get_wantedyaw_point()->wantedAbsoluteAngle;
	
	for(int i = 0;i < sizeof(can_send_data_s);i++)
	{
		can1_send_data[i] = data_u.data_uint8[i];
	}
	
    can1_tx_message.StdId = 0x002;
    can1_tx_message.IDE = CAN_ID_STD;
    can1_tx_message.RTR = CAN_RTR_DATA;
    can1_tx_message.DLC = 0x08;
	
	//usart_printf("%f,%f\r\n",data_u.data.yaw,data_u.data.nuc_yaw);
	HAL_CAN_AddTxMessage(&hcan1, &can1_tx_message, can1_send_data, &send_mail_box);
	
}


//���ͨ�����͵��̿�����
typedef struct
{
	int16_t channel_0;
	int16_t channel_2;
	int16_t channel_3;
	enum RobotState_e mode;
	uint8_t keyboard;
}can_send_encode_data_s;
void CAN1_send_channel()
{
	uint32_t send_mail_box;
	typedef union
	{
		can_send_encode_data_s data;
		uint8_t data_1[sizeof(can_send_encode_data_s)];
	}can_send_data_channel_u;
	can_send_data_channel_u data_u;
	rc_chassis = get_remote_control_point();
	data_u.data.channel_0 =rc_chassis->rc.ch[0];
	data_u.data.channel_2 =rc_chassis->rc.ch[2];
	data_u.data.channel_3 =rc_chassis->rc.ch[3];
	robotMode=getRobotPresentMode();
	data_u.data.mode=*robotMode;
	data_u.data.keyboard=0;

	if(rc_chassis->key.v&KEY_PRESSED_OFFSET_W)
		data_u.data.keyboard = (data_u.data.keyboard | 128);
	if(rc_chassis->key.v&KEY_PRESSED_OFFSET_A)
		data_u.data.keyboard = (data_u.data.keyboard | 64);
	if(rc_chassis->key.v&KEY_PRESSED_OFFSET_S)
		data_u.data.keyboard = (data_u.data.keyboard | 32);
	if(rc_chassis->key.v&KEY_PRESSED_OFFSET_D)
		data_u.data.keyboard = (data_u.data.keyboard | 16);
	if(rc_chassis->key.v&KEY_PRESSED_OFFSET_E)
		data_u.data.keyboard = (data_u.data.keyboard | 8);
	if(rc_chassis->key.v&KEY_PRESSED_OFFSET_F)
		data_u.data.keyboard = (data_u.data.keyboard | 4);
	if(rc_chassis->key.v&KEY_PRESSED_OFFSET_G)
		data_u.data.keyboard = (data_u.data.keyboard | 2);
	for(int i = 0; i <sizeof(can_send_encode_data_s); i++)
	{
		can1_send_data_channel[i] = data_u.data_1[i];
	}
	

	//usart_printf("%d,%d,%d,%d\r\n",data_u.data_s_1.channel_0 ,data_u.data_s_1.channel_1 ,data_u.data_s_1.channel_2 ,data_u.data_s_1.channel_3 );
	  can1_tx_channel_message.StdId = 0x003;
    can1_tx_channel_message.IDE = CAN_ID_STD;
    can1_tx_channel_message.RTR = CAN_RTR_DATA;
    can1_tx_channel_message.DLC = 0x08;
	
	
	HAL_CAN_AddTxMessage(&hcan1, &can1_tx_channel_message, can1_send_data_channel, &send_mail_box);
	
}









//NUC_YAW �൱�ڿ��Ƶ�Ŀ��ֵ������ٵ�
typedef struct{
  fp32 nuc_yaw;
	uint8_t is_auto;
}can_send_data_nuc_yaw_s;
void CAN1_send_nuc_yaw(){
  uint32_t send_mail_box;
	typedef union
	{
		can_send_data_nuc_yaw_s data_s_2;
		uint8_t data_2[sizeof(can_send_data_nuc_yaw_s)];
	}can_send_data_nuc_yaw_u;
	can_send_data_nuc_yaw_u data_u;
	data_u.data_s_2.nuc_yaw=get_wantedyaw_point()->wantedAbsoluteAngle;
	data_u.data_s_2.is_auto = 0;
	//usart_printf("%f\r\n",data_u.data_s_2.nuc_yaw);
	for(int i = 0; i <sizeof(can_send_data_nuc_yaw_s); i++)
	{
		can1_send_data_nuc_yaw[i] = data_u.data_2[i];
	}
	can1_tx_nuc_message.StdId = 0x004;
    can1_tx_nuc_message.IDE = CAN_ID_STD;
    can1_tx_nuc_message.RTR = CAN_RTR_DATA;
    can1_tx_nuc_message.DLC = 0x08;
	HAL_CAN_AddTxMessage(&hcan1,&can1_tx_nuc_message,can1_send_data_nuc_yaw,&send_mail_box);
} 

//KeyBoard_Command
typedef struct {
	uint8_t W;
	uint8_t A;
	uint8_t S;
	uint8_t D;//���Ƶ����˶�
	fp32 nuc_yaw;
}can_send_data_keyboard_s;
void CAN1_send_keyboard(){
	uint32_t send_mail_box;
	typedef union
	{
		can_send_data_keyboard_s data_s_3;
		uint8_t data_3[sizeof(can_send_data_keyboard_s)];
	}can_send_data_keyboard_u;
	can_send_data_keyboard_u data_u;
	rc_chassis=get_remote_control_point();
	data_u.data_s_3.W=0;
	data_u.data_s_3.A=0;
	data_u.data_s_3.S=0;
	data_u.data_s_3.D=0;
	data_u.data_s_3.nuc_yaw=get_wantedyaw_point()->wantedAbsoluteAngle;
	if(rc_chassis->key.v&KEY_PRESSED_OFFSET_W)
		data_u.data_s_3.W=1;
	if(rc_chassis->key.v&KEY_PRESSED_OFFSET_A)
		data_u.data_s_3.A=1;
	if(rc_chassis->key.v&KEY_PRESSED_OFFSET_S)
		data_u.data_s_3.S=1;
	if(rc_chassis->key.v&KEY_PRESSED_OFFSET_D)
		data_u.data_s_3.D=1;
	data_u.data_s_3.nuc_yaw=get_wantedyaw_point()->wantedAbsoluteAngle;
	//usart_printf("%d,%d,%d,%d\r\n",data_u.data_s_3.W,data_u.data_s_3.A,data_u.data_s_3.S,data_u.data_s_3.D);
	for(int i=0;i<sizeof(can_send_data_keyboard_s);i++){
		can1_send_data_keyboard[i]=data_u.data_3[i];
	}
	can1_tx_keyboard_message.StdId=0x005;
	can1_tx_keyboard_message.IDE = CAN_ID_STD;
  can1_tx_keyboard_message.RTR = CAN_RTR_DATA;
  can1_tx_keyboard_message.DLC = 0x08;
	HAL_CAN_AddTxMessage(&hcan1,&can1_tx_keyboard_message,can1_send_data_keyboard,&send_mail_box);
}

void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          �����ĸ����̵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_shoot(int16_t motor5, int16_t motor6)
{
    uint32_t send_mail_box;
    shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
    shoot_can_send_data[0] = (motor5 >> 8);
    shoot_can_send_data[1] = motor5;
    shoot_can_send_data[2] = (motor6 >> 8);
    shoot_can_send_data[3] = motor6;
    shoot_can_send_data[4] = 0;
    shoot_can_send_data[5] = 0;
    shoot_can_send_data[6] = 0;
    shoot_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

void CAN_cmd_shoot_reset(int16_t motor5, int16_t motor6)
{
    uint32_t send_mail_box;
    shoot_tx_message.StdId = 0x700;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
    shoot_can_send_data[0] = 0;
    shoot_can_send_data[1] = 0;
    shoot_can_send_data[2] = 0;
    shoot_can_send_data[3] = 0;
    shoot_can_send_data[4] = 0;
    shoot_can_send_data[5] = 0;
    shoot_can_send_data[6] = 0;
    shoot_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}
const can_send_referee_s* get_refree_point()
{
		return &gimbal_rx_referee;
}
//һЩ���ڴ��ݲ����ĺ���������pid����

/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}
/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}

/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}

/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

/**
  * @brief          ����Ħ���ֵ�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,1]
  * @retval         �������ָ��
  */
const motor_measure_t *get_shoot_motor_measure_point(uint8_t i)
{
    return &motor_shoot[(i & 0x03)];
}

