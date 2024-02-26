#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H
#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2
#define SHOOT_CAN hcan2
//��̨�͵���ʹ�õ��ǲ�ͬ�����İ壬�����˲�ͬ��can�˿ڡ�
//�����ͻ�����������⣬����ʱ��Ҫע�⡣
//������ʹ��ͬһ���˿�ʱ�������޸�Ϊ
// #define GIMBAL_CAN hcan1

//��������ͺ�Լ����idö�����͡�ͨ���޸������ֵ��Ӧ���ĵ��
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

    CAN_SHOOT_ALL_ID=0X200,
    CAN_SHOOT_3508_ID_1=0X201,
    CAN_SHOOT_3508_ID_2=0X202,
    // ����Ħ����3508������can2��
    
} can_msg_id_e;

// ������ݽṹ�塣
typedef struct {
	int16_t shootspeed;
	uint8_t robot_color;
}can_send_referee_s;
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

//�˴��޸�Ϊ����ʹ�õĵ��
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
  * @brief          ����̷���IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          ���͵��̵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          ���Ͳ���������Ƶ���(0x205,0x206)
  * @param[in]      motor1: (0x205) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x206) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_shoot(int16_t motor5, int16_t motor6);


// ���ڸ�����ĺ����ṩ����

/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

//���ǿ�����Ҫ�޸Ĳ���
/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
  * @brief          ����Ħ���ֵ�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,2]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_shoot_motor_measure_point(uint8_t i);
extern void CAN1_send_yaw();
extern void CAN1_send_channel();
extern void CAN1_send_nuc_yaw();
extern const can_send_referee_s* get_refree_point();
extern void CAN1_send_keyboard();
#endif

/*ʾ��
        CAN_cmd_chassis(4000, 4000, 4000, 4000);
        HAL_Delay(2);
        CAN_cmd_gimbal(10000, 10000, 10000, 10000);
        HAL_Delay(2);
*/
