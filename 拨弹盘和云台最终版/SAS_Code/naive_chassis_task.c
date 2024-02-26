/**
 * encoding:GBK
 * @file naive_chassis_task.c
 * @author 陈卓勋 电控组 UCAS-SAS 中国科学院大学机器人队
 * @brief 
 * @version 0.2
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022
 * 
 * 功能定义
 * 
 */
// 底盘电机ID和建系
/*      前X正
    2           1
左Y正                  右Y负
    3           4
        后X负
*/
// 编号
/*
1  0

2  3
*/

#include "remote_control.h"
#include "nucCommu.h"
#include "CAN_receive.h"
#include "pid.h"
#include "cmsis_os.h"
#include "robot_total_mode.h"
#include "RobotStructure.h"
#include "arm_math.h"
#include "OLED.h"
#include "naive_gimbal_task.h"
#include "user_lib.h"

//*****************************debug选项*******************//
#define GIMBAL_YAW_CHANNEL 2    //左边的左右摇杆，可以作为测试时使用。与底盘的旋转冲突
// #define POWERLESS_DEBUG_FOR_SAFETY
#define OLED_DEBUG_GIMBAL_TASK_ON


#ifdef OLED_DEBUG_GIMBAL_TASK_ON
#define OLED_REFRESH_TIME 300
#endif


//*************************按键定义********************//
// 机器人有四种模式。无力、普通车、卡yaw坏车、（准）小陀螺车（根据机械定义自动判断是否变化方向）。
//按键定义
#define HANDLE_LEFT_LR 2    //遥控器左边摇杆的左右方向  left right
#define HANDLE_LEFT_BF 3    //遥控器左边摇杆的前后方向  back forth
#define HANDLE_RIGHT_LR 0    //遥控器右边摇杆的左右方向  left right
#define HANDLE_RIGHT_BF 1    //遥控器右边摇杆的前后方向  back forth

//普通车模式    common car 按C或者遥控器右拨杆上方开启，（全能坦克）
//平移控制的坐标系为底盘坐标系
#define COMMON_FORTH            KEY_PRESSED_OFFSET_E    //前进
#define COMMON_BACK             KEY_PRESSED_OFFSET_D    //后退
#define COMMON_LEFT_ROTATE      KEY_PRESSED_OFFSET_S    //左转
#define COMMON_RIGHT_ROTATE     KEY_PRESSED_OFFSET_F    //右转
#define COMMON_LEFT_MOVE        KEY_PRESSED_OFFSET_A    //左平移
#define COMMON_RIGHT_MOVE       KEY_PRESSED_OFFSET_G    //右平移
//而遥控器左摇杆控制前后平移和左右旋转
#define COMMON_BF_RC            HANDLE_LEFT_BF      //前进,后退
#define COMMON_LR_ROTATE_RC     HANDLE_LEFT_LR      //左转,右转

//小陀螺模式    spinner，就像一个旋转的陀螺v，所以按v进入小陀螺模式。或者遥控器右拨杆在中间进入
//平移控制的坐标系为云台坐标系
#define SPINNER_FORTH            KEY_PRESSED_OFFSET_W    //前进
#define SPINNER_BACK             KEY_PRESSED_OFFSET_S    //后退
#define SPINNER_LEFT_MOVE        KEY_PRESSED_OFFSET_A    //左平移
#define SPINNER_RIGHT_MOVE       KEY_PRESSED_OFFSET_D    //右平移
//遥控器左摇杆控制前后平移和左右平移
#define SPINNER_BF_RC            HANDLE_LEFT_BF      //前进,后退
#define SPINNER_LR_MOVE_RC       HANDLE_LEFT_LR      //左转,右转


//卡yaw坏车模式    bad yaw，首字母B，所以按B进入卡yaw坏车模式。或者遥控器右拨杆在下进入
//此时，云台yaw相对于底盘不转动，所有云台目标转动量（包括自瞄）输出到底盘
//平移控制的坐标系为云台坐标系
#define BAD_YAW_FORTH            KEY_PRESSED_OFFSET_W    //前进
#define BAD_YAW_BACK             KEY_PRESSED_OFFSET_S    //后退
#define BAD_YAW_LEFT_MOVE        KEY_PRESSED_OFFSET_A    //左平移
#define BAD_YAW_RIGHT_MOVE       KEY_PRESSED_OFFSET_D    //右平移

#define BAD_YAW_BF_RC            HANDLE_LEFT_BF      //前进,后退
#define BAD_YAW_LR_MOVE_RC       HANDLE_LEFT_LR      //左转,右转

///////////////////////////////////////////////////////////////////////

//****************************机器人控制常量***************//
#define KEYBOARD_CONTROL_ROBOT_SPEED_X 0.6f      //键盘wasd控制的速度
#define KEYBOARD_CONTROL_ROBOT_SPEED_Y 0.6f
#define KEYBOARD_CONTROL_ROBOT_SPEED_W 0.5f
#define SPINNER_W   1.0f        //小陀螺运动旋转速度
#define SPINNER_MAX_ROUNDS  20   //小陀螺运动时，底盘和云台差最大圈数。甚至可以用小陀螺模式回正云台 233
#define ECD_FULL_ROUND 8192 //一圈的ECD值。实际取值0-8191


//****************************控制量计算系数**************//
#define CHASSIS_VX_RC_SEN 0.0005f   //把遥控器摇杆读数转化为速度的系数。遥控器-660~+660.可得最大0.33m/s
#define CHASSIS_VY_RC_SEN -0.0005f   //是负数，遥控器向左拉，向左平移，遥控器摇杆读数为负，在坐标系中为向y正方向
#define CHASSIS_WZ_RC_SEN -0.0005f   //最大旋转线速度。0.33m/s  //是负数。遥控器右拉，向右转

#define CHASSIS_RC_DEADLINE 10  //低于这个值的摇杆读数一律为0

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f   //电机旋线速度转化为机器人底盘速度。麦轮直径15cm，半径8cm
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f  
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f  

#define MOTOR_DISTANCE_TO_CENTER 0.27f           //电机到机器人中心的距离 27cm，可以在solidworks装配体中测量
// #define M3508_MOTOR_RPM_TO_VECTOR 0.0083f   //底盘电机转速到线速度（m/s）转化。计算：n*2*π*R/60
#define M3508_MOTOR_RPM_TO_VECTOR 0.0004f        //应该还要除以这里大约20.想一想应该是减速电机，转化为实际的速度还要除以减速比。
                                                //这个值还是

#define BAD_YAW_DATA_ROTATE_COEF 17.5f       //卡yaw模式下，从云台旋转PID获得的旋转速度，由于yaw pid的不完善，获得的
                                                //并非真实旋转速度,需要先乘以一个系数。

//***************************PID控制参数*****************//
#define MAX_MOTOR_CAN_CURRENT 15000.0f  //底盘3508最大can发送电流值
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 16000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f



#define MAX_WHEEL_SPEED 4.0f    //单个底盘电机最大速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f //底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f //底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_W 0.9f

//*************************任务相关常量*******************//
// #define AUTO_DETECT_INIT_ECD            //在任务开始时获取电机中央ECD。若开启，则覆盖get以下的预先测量值
#define INIT_PRE_GOT_MID_ECD    3557    //预先测量的云台在中心时的yawECD。在机器人重新安装云台后，可能导致ECD和这个不一样
#define CHASSIS_TASK_INIT_TIME 500      //电机初始化时间
#define CHASSIS_CONTROL_TIME_MS 5       //每5ms执行一次任务

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

//******************数字滤波所需常量****************// 系数和控制方法来自官方步兵
#define CHASSIS_ACCEL_X_NUM 150.1666f
#define CHASSIS_ACCEL_Y_NUM 150.1666f
#define CHASSIS_ACCEL_W_NUM 150.66f


//**************************控制量结构体*******************//

// 坐标系
enum MovingAxis_e{
    MovingAxis_e_GimbalAxis,
    MovingAxis_e_ChassisAxis,
};

//机器人控制量
struct RobotControl_s{
    fp32 vx,vy,w;
    first_order_filter_type_t vx_filter,vy_filter,w_filter;
    enum MovingAxis_e axis;
};

//云台yaw电机相对于底盘角度结构体
struct Angle_s{
    int16_t initGimbalYawECD;       //开机时的ECD
    int16_t nowGimbalYawECD;        //目前的ECD
    int32_t rotateRounds;           //目前旋转了多少圈，用于准小陀螺模式时，防止线断掉。
    fp32    gimbalAngleFromChassis; //云台相对于最初的角度，逆时针为正方向。单位弧度。
};

//底盘电机控制
struct MotorControl_s{
    pid_type_def    vpid;
    fp32            presentMotorSpeed;
    fp32            wantedMotorSpeed;
    int16_t         giveCurrent;
};


//************************用于控制的全局变量**********//
static int zeroCurrentMark;    //当离线时，直接发送零电流的时候
static struct Angle_s someAngle;   //用于记录所有的云台、底盘角度相关值
static struct MotorControl_s driveMotor[4];
static const RC_ctrl_t *rc_p;   //遥控器位置指针，需要初始化
static const toSTM32_t *nuc_p;  //NUC数据位置。未来制造全自动机器人时需要
const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD}; //PID 初始化参数
static const enum RobotState_e *robotMode;   //机器人模式
static struct RobotControl_s robotTotalSpeedControl;
static int16_t vx_channel,vy_channel,w_channel; //遥控器deadband limit后输出的值

const struct GimbalToChassis_s * badYawData;    //卡yaw模式下，旋转数据
const int16_t * gimbalRounds;


//***************调试所用变量**************************//
#ifdef OLED_DEBUG_GIMBAL_TASK_ON
static int time_passed_debug=0;
static fp32 pidOUT;
#endif
#ifdef TEST_TASK_TIME
static uint32_t taskTestStartTime,timeConsume;
#endif

//************************各个功能函数*************//
static void initPIDs(void)
{
    int i;
    for(i=0;i<4;i++)
        PID_init(&(driveMotor[i].vpid),PID_POSITION,motor_speed_pid,M3505_MOTOR_SPEED_PID_MAX_OUT,M3505_MOTOR_SPEED_PID_MAX_IOUT);
}

static void initFilters(void)
{
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    const static fp32 chassis_w_order_filter[1] = {CHASSIS_ACCEL_W_NUM};
    
    first_order_filter_init(&(robotTotalSpeedControl.vx_filter), CHASSIS_CONTROL_TIME_MS, chassis_x_order_filter);
    first_order_filter_init(&(robotTotalSpeedControl.vy_filter), CHASSIS_CONTROL_TIME_MS, chassis_y_order_filter);
    first_order_filter_init(&(robotTotalSpeedControl.w_filter), CHASSIS_CONTROL_TIME_MS, chassis_w_order_filter);

}

static void setChassisMode(void)
{
    zeroCurrentMark=0;
    if(RobotState_e_Powerless== *robotMode)
        zeroCurrentMark=1;
}

static void analyseTotalControl(void)    //通过遥控器和键盘参数设置速度。当键盘输出量为0时，通过遥控器摇杆确定速度。
{
    // int keyBoardMove=0;
    //注意建系的方向
    robotTotalSpeedControl.axis=MovingAxis_e_GimbalAxis;    //默认云台坐标系，有助于操作手的控制
    robotTotalSpeedControl.vx=0;robotTotalSpeedControl.vy=0;robotTotalSpeedControl.w=0;
    
    if(RobotState_e_Powerless==*robotMode)
    {
        return;
    }
    else if(RobotState_e_CommonCar==*robotMode)
    {
        //底盘坐标系
        robotTotalSpeedControl.axis=MovingAxis_e_ChassisAxis;
        if(rc_p->key.v & COMMON_FORTH)  //前
            robotTotalSpeedControl.vx-=KEYBOARD_CONTROL_ROBOT_SPEED_X;
        if(rc_p->key.v & COMMON_BACK)  //后
            robotTotalSpeedControl.vx+=KEYBOARD_CONTROL_ROBOT_SPEED_X;
        if(rc_p->key.v & COMMON_LEFT_MOVE)  //左
            robotTotalSpeedControl.vy-=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
        if(rc_p->key.v & COMMON_RIGHT_MOVE)  //右
            robotTotalSpeedControl.vy+=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
        if(rc_p->key.v & COMMON_LEFT_ROTATE)    //左旋 从上向下看为逆时针
            robotTotalSpeedControl.w+=KEYBOARD_CONTROL_ROBOT_SPEED_W;
        if(rc_p->key.v & COMMON_RIGHT_ROTATE)   //右旋
            robotTotalSpeedControl.w-=KEYBOARD_CONTROL_ROBOT_SPEED_W;

        rc_deadband_limit(rc_p->rc.ch[COMMON_BF_RC],vx_channel,CHASSIS_RC_DEADLINE);
        rc_deadband_limit(rc_p->rc.ch[COMMON_LR_ROTATE_RC],w_channel,CHASSIS_RC_DEADLINE);
        robotTotalSpeedControl.vx +=vx_channel*CHASSIS_VX_RC_SEN;
        robotTotalSpeedControl.w +=w_channel*CHASSIS_WZ_RC_SEN;

    }
    else if(RobotState_e_BadYawCar==*robotMode)
    {
        //云台坐标系
        robotTotalSpeedControl.axis=MovingAxis_e_GimbalAxis;

        if(rc_p->key.v & BAD_YAW_FORTH)  //前
            robotTotalSpeedControl.vx-=KEYBOARD_CONTROL_ROBOT_SPEED_X;
        if(rc_p->key.v & BAD_YAW_BACK)  //后
            robotTotalSpeedControl.vx+=KEYBOARD_CONTROL_ROBOT_SPEED_X;
        if(rc_p->key.v & BAD_YAW_LEFT_MOVE)  //左
            robotTotalSpeedControl.vy-=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
        if(rc_p->key.v & BAD_YAW_RIGHT_MOVE)  //右
            robotTotalSpeedControl.vy+=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
        rc_deadband_limit(rc_p->rc.ch[BAD_YAW_BF_RC],vx_channel,CHASSIS_RC_DEADLINE);
        rc_deadband_limit(rc_p->rc.ch[BAD_YAW_LR_MOVE_RC],vy_channel,CHASSIS_RC_DEADLINE);
        robotTotalSpeedControl.vx += vx_channel*CHASSIS_VX_RC_SEN;
        robotTotalSpeedControl.vy += vy_channel*CHASSIS_VY_RC_SEN;
        if(badYawData->effective)
            robotTotalSpeedControl.w += badYawData->w *BAD_YAW_DATA_ROTATE_COEF;
    }
    else if(RobotState_e_Spinner==*robotMode)
    {
        //云台坐标系
        robotTotalSpeedControl.axis=MovingAxis_e_GimbalAxis;
        if(rc_p->key.v & SPINNER_FORTH)  //前
            robotTotalSpeedControl.vx+=KEYBOARD_CONTROL_ROBOT_SPEED_X;
        if(rc_p->key.v & SPINNER_BACK)  //后
            robotTotalSpeedControl.vx-=KEYBOARD_CONTROL_ROBOT_SPEED_X;
        if(rc_p->key.v & SPINNER_LEFT_MOVE)  //左
            robotTotalSpeedControl.vy+=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
        if(rc_p->key.v & SPINNER_RIGHT_MOVE)  //右
            robotTotalSpeedControl.vy-=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
        rc_deadband_limit(rc_p->rc.ch[SPINNER_BF_RC],vx_channel,CHASSIS_RC_DEADLINE);
        rc_deadband_limit(rc_p->rc.ch[SPINNER_LR_MOVE_RC],vy_channel,CHASSIS_RC_DEADLINE);
        robotTotalSpeedControl.vx += vx_channel*CHASSIS_VX_RC_SEN;
        robotTotalSpeedControl.vy += vy_channel*CHASSIS_VY_RC_SEN;
        //内置旋转速度控制程序
        //最简单的，单级spinner 速度
        // 从云台控制那里获取云台相对于底盘转过的圈数，云台逆时针转为正圈数。若到达顶峰，则让底盘也逆时针转动
        static fp32 nowSpinnerW=SPINNER_W; //初始化底盘逆时针旋转 (奇怪，一次弄错，后面得跟着反过来)
        if(*gimbalRounds>=SPINNER_MAX_ROUNDS)   //云台逆时针转多了
            nowSpinnerW=SPINNER_W;  //底盘逆时针跟上
        else if((*gimbalRounds)<=-SPINNER_MAX_ROUNDS) //云台顺时针转多了
            nowSpinnerW=-SPINNER_W;  //底盘顺时针跟上
        robotTotalSpeedControl.w=nowSpinnerW;
    }    
}

static void refreshECD(void)   //更新当前云台yawECD值和弧度制的值，实现云台坐标系控制以及小陀螺
{
    someAngle.nowGimbalYawECD=get_yaw_gimbal_motor_measure_point()->ecd;
    someAngle.gimbalAngleFromChassis=(someAngle.nowGimbalYawECD-someAngle.initGimbalYawECD)*2*PI/ECD_FULL_ROUND;
}

static void calcWheelVelocityInChassisAxis(void)   //将控制量中的坐标系速度转化为底盘坐标系的速度，并计算轮子转动速度
{
    fp32 sin_yaw,cos_yaw;
    fp32 vx,vy,w;
    sin_yaw=arm_sin_f32(someAngle.gimbalAngleFromChassis);
    cos_yaw=arm_cos_f32(someAngle.gimbalAngleFromChassis);
    w=robotTotalSpeedControl.w;
    // robotTotalSpeedControl.axis=MovingAxis_e_GimbalAxis;
    if(robotTotalSpeedControl.axis==MovingAxis_e_GimbalAxis)
    {
        //将云台坐标系速度转化为底盘坐标系速度。坐标系变换
        vx=cos_yaw*robotTotalSpeedControl.vx-sin_yaw*robotTotalSpeedControl.vy;
        vy=sin_yaw*robotTotalSpeedControl.vx+cos_yaw*robotTotalSpeedControl.vy;
        
    }
    else if(robotTotalSpeedControl.axis==MovingAxis_e_ChassisAxis)
    {
        vx=robotTotalSpeedControl.vx;
        vy=robotTotalSpeedControl.vy;
    }
    driveMotor[0].wantedMotorSpeed=-vx-vy-w;    //麦轮速度计算公式
    driveMotor[1].wantedMotorSpeed=vx-vy-w;
    driveMotor[2].wantedMotorSpeed=vx+vy-w;
    driveMotor[3].wantedMotorSpeed=-vx+vy-w;

    #ifdef OLED_DEBUG_CHASSIS_TASK_ON
    debug_vx=vx;
    debug_vy=vy;
    debug_w=w;
    #endif
}

static void firstOrderFilt()
{
    first_order_filter_cali(&robotTotalSpeedControl.vx_filter,robotTotalSpeedControl.vx);
    first_order_filter_cali(&robotTotalSpeedControl.vy_filter,robotTotalSpeedControl.vy);
    first_order_filter_cali(&robotTotalSpeedControl.w_filter,robotTotalSpeedControl.w);
    robotTotalSpeedControl.vx=robotTotalSpeedControl.vx_filter.out;
    robotTotalSpeedControl.vy=robotTotalSpeedControl.vy_filter.out;
    robotTotalSpeedControl.w=robotTotalSpeedControl.w_filter.out;
}

static void calcGiveCurrent(void) //pid 计算电流
{
    uint8_t i;
    for(i=0;i<4;i++)
    {
        driveMotor[i].presentMotorSpeed=get_chassis_motor_measure_point(i)->speed_rpm*M3508_MOTOR_RPM_TO_VECTOR;    //获取电机转速
        PID_calc(&(driveMotor[i].vpid),driveMotor[i].presentMotorSpeed,driveMotor[i].wantedMotorSpeed);   //pid计算
        driveMotor[i].giveCurrent=driveMotor[i].vpid.out;    //pid的输出就是驱动电流
    }
}


void chassis_task(void const *pvParameters)
{
    osDelay(CHASSIS_TASK_INIT_TIME);    //等待电机初始化
    initPIDs();                         //初始化PID
    initFilters();
    rc_p=get_remote_control_point();    //获取遥控器数据和NUC数据指针
    nuc_p=get_nuc_control_point();
    robotMode=getRobotPresentMode();
    badYawData=getBadYawWantedSpd();    //初始化badYaw传来的数据
    gimbalRounds=getGimbalNowRoundsPoint(); //获取云台旋转圈数指针
    #ifdef AUTO_DETECT_INIT_ECD     //分情况载入初始ECD
    someAngle.initGimbalYawECD=get_yaw_gimbal_motor_measure_point()->ecd;
    #else
    someAngle.initGimbalYawECD=INIT_PRE_GOT_MID_ECD;
    #endif


    while(1)
    {
        setChassisMode();
        analyseTotalControl();
        refreshECD();	
        firstOrderFilt();
        calcWheelVelocityInChassisAxis();
        calcGiveCurrent();

        if(zeroCurrentMark) 
            CAN_cmd_chassis(0,0,0,0);       //发送零电流
        else
            CAN_cmd_chassis(driveMotor[0].giveCurrent,driveMotor[1].giveCurrent,
                driveMotor[2].giveCurrent,driveMotor[3].giveCurrent);   //正常控制电机
        

        osDelay(CHASSIS_CONTROL_TIME_MS);
    }
}

fp32 getChassisOLEDShowFp32(void)
{
    return robotTotalSpeedControl.w;
}
int16_t getChassisOLEDShowInt16(void)
{
    return *gimbalRounds;
}
void OLED_chassis(void)
{
    OLED_printf(0,0,"YAW_ECD:%d",get_yaw_gimbal_motor_measure_point()->ecd);
}
