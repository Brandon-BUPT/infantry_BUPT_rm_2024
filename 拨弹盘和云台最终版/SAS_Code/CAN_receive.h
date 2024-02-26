#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H
#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2
#define SHOOT_CAN hcan2
//云台和底盘使用的是不同的中心板，连接了不同的can端口。
//解决冲突和吞吐率问题，接线时需要注意。
//当我们使用同一个端口时，可以修改为
// #define GIMBAL_CAN hcan1

//电机的类型和约定的id枚举类型。通过修改这里的值对应更改电机
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
    // 两个摩擦轮3508挂载在can2上
    
} can_msg_id_e;

// 电机数据结构体。
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

//此处修改为我们使用的电机
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          向底盘发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          发送底盘电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          发送拨弹电机控制电流(0x205,0x206)
  * @param[in]      motor1: (0x205) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x206) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_shoot(int16_t motor5, int16_t motor6);


// 用于给外面的函数提供数据

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

//我们可能需要修改参数
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
  * @brief          返回摩擦轮电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,2]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_shoot_motor_measure_point(uint8_t i);
extern void CAN1_send_yaw();
extern void CAN1_send_channel();
extern void CAN1_send_nuc_yaw();
extern const can_send_referee_s* get_refree_point();
extern void CAN1_send_keyboard();
#endif

/*示例
        CAN_cmd_chassis(4000, 4000, 4000, 4000);
        HAL_Delay(2);
        CAN_cmd_gimbal(10000, 10000, 10000, 10000);
        HAL_Delay(2);
*/
