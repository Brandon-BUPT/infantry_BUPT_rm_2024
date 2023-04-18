/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan2
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,
	  CAN_GET_YAW_ID=0x002

} can_msg_id_e;
enum RobotState_e{
    RobotState_e_Powerless=0,     //����ģʽ��ң��������ʱ����

    RobotState_e_CommonCar=3,     //��ͨ��ģʽ�����̿��Ƶ�������ϵ�˶���edsfag�ֱ��Ӧǰ��ƽ�ơ�������ת������ƽ�ơ�
                                //��������̨������ת��
                                // ��ʱ��ң�������ҡ�˿��Ƶ����ڵ�������ϵǰ��ƽ�ơ�������ת���Ҳ���ȷ����̨��������ת
                                // ���˳���ʱ���룬�Լ���cʱ����
                                // auto NUC������̨�����ķ���

    RobotState_e_GimbalCar=1,        //��̨��ģʽ��������yaw���ܿ�ס�������ģʽʱ���᳢�Ի�����̨��һ��ʱ���ڻ������˾ͷ���
                                //wsadǰ������ƽ�ƣ����pitch���º�������ת
                                // ��ʱ��ң�������ҡ�˿��ƻ���������̨����ϵ��ǰ������ƽ�ƣ�
                                //��������ʱ���룬�Լ���bʱ����
                                // auto NUC������̨pitch������ת�򣺽Ƕ�PID���ƻ�õĽ��ٶȳ���ϵ���󽻸����̿�������
                                

    RobotState_e_Spinner=2,       //С����ģʽ����̨���ݻ�е�ṹ�Զ�ȷ����С���ݻ���׼С���ݡ�
                                //��������ʱ���룬�Լ���vʱ����
                                //ң������ҡ�˿�������̨����ϵ�µ�ǰ������ƽ�ƣ���ҡ�˿�����̨��ת
                                // auto NUC������̨��������
};
typedef struct
{
	int16_t channel_0;
	int16_t channel_2;
	int16_t channel_3;
	enum RobotState_e mode;
	uint8_t keyboard;
}can_send_encode_data_s;
typedef struct
{
	int16_t channel_0;
	int16_t channel_2;
	int16_t channel_3;
	enum RobotState_e mode;
	bool_t W;
	bool_t A;
	bool_t S;
	bool_t D;
	bool_t E;
	bool_t F;
	bool_t G;
}can_send_data_channel_s;

typedef struct
{
	fp32 yaw;
	fp32 nuc_yaw;
}can_send_data_s;

//rm motor data
typedef struct
{
	int a;
}can_send_data_keyboard_s;
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;
typedef struct{
  fp32 nuc_yaw;
}can_send_data_nuc_yaw_s;
typedef struct
{
  int16_t current;
}can_send_data_trigger_s;
/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
extern const  can_send_data_s *get_yaw_measure_point();
extern const  can_send_data_channel_s *get_channel_measure_point();
//extern const can_send_data_nuc_yaw_s *get_nuc_yaw_measure_point();
extern const can_send_data_s *get_keyboard_measure_point(); 
extern const can_send_data_trigger_s *get_trigger_measure_point();
extern void CAN2_send_super_c_control(int16_t a,int16_t b,int16_t c);
extern void CAN2_send_super_c_buffer(int16_t buffer);
#endif
