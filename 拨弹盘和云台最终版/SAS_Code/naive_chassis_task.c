/**
 * encoding:GBK
 * @file naive_chassis_task.c
 * @author ��׿ѫ ����� UCAS-SAS �й���ѧԺ��ѧ�����˶�
 * @brief 
 * @version 0.2
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022
 * 
 * ���ܶ���
 * 
 */
// ���̵��ID�ͽ�ϵ
/*      ǰX��
    2           1
��Y��                  ��Y��
    3           4
        ��X��
*/
// ���
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

//*****************************debugѡ��*******************//
#define GIMBAL_YAW_CHANNEL 2    //��ߵ�����ҡ�ˣ�������Ϊ����ʱʹ�á�����̵���ת��ͻ
// #define POWERLESS_DEBUG_FOR_SAFETY
#define OLED_DEBUG_GIMBAL_TASK_ON


#ifdef OLED_DEBUG_GIMBAL_TASK_ON
#define OLED_REFRESH_TIME 300
#endif


//*************************��������********************//
// ������������ģʽ����������ͨ������yaw��������׼��С���ݳ������ݻ�е�����Զ��ж��Ƿ�仯���򣩡�
//��������
#define HANDLE_LEFT_LR 2    //ң�������ҡ�˵����ҷ���  left right
#define HANDLE_LEFT_BF 3    //ң�������ҡ�˵�ǰ����  back forth
#define HANDLE_RIGHT_LR 0    //ң�����ұ�ҡ�˵����ҷ���  left right
#define HANDLE_RIGHT_BF 1    //ң�����ұ�ҡ�˵�ǰ����  back forth

//��ͨ��ģʽ    common car ��C����ң�����Ҳ����Ϸ���������ȫ��̹�ˣ�
//ƽ�ƿ��Ƶ�����ϵΪ��������ϵ
#define COMMON_FORTH            KEY_PRESSED_OFFSET_E    //ǰ��
#define COMMON_BACK             KEY_PRESSED_OFFSET_D    //����
#define COMMON_LEFT_ROTATE      KEY_PRESSED_OFFSET_S    //��ת
#define COMMON_RIGHT_ROTATE     KEY_PRESSED_OFFSET_F    //��ת
#define COMMON_LEFT_MOVE        KEY_PRESSED_OFFSET_A    //��ƽ��
#define COMMON_RIGHT_MOVE       KEY_PRESSED_OFFSET_G    //��ƽ��
//��ң������ҡ�˿���ǰ��ƽ�ƺ�������ת
#define COMMON_BF_RC            HANDLE_LEFT_BF      //ǰ��,����
#define COMMON_LR_ROTATE_RC     HANDLE_LEFT_LR      //��ת,��ת

//С����ģʽ    spinner������һ����ת������v�����԰�v����С����ģʽ������ң�����Ҳ������м����
//ƽ�ƿ��Ƶ�����ϵΪ��̨����ϵ
#define SPINNER_FORTH            KEY_PRESSED_OFFSET_W    //ǰ��
#define SPINNER_BACK             KEY_PRESSED_OFFSET_S    //����
#define SPINNER_LEFT_MOVE        KEY_PRESSED_OFFSET_A    //��ƽ��
#define SPINNER_RIGHT_MOVE       KEY_PRESSED_OFFSET_D    //��ƽ��
//ң������ҡ�˿���ǰ��ƽ�ƺ�����ƽ��
#define SPINNER_BF_RC            HANDLE_LEFT_BF      //ǰ��,����
#define SPINNER_LR_MOVE_RC       HANDLE_LEFT_LR      //��ת,��ת


//��yaw����ģʽ    bad yaw������ĸB�����԰�B���뿨yaw����ģʽ������ң�����Ҳ������½���
//��ʱ����̨yaw����ڵ��̲�ת����������̨Ŀ��ת�������������飩���������
//ƽ�ƿ��Ƶ�����ϵΪ��̨����ϵ
#define BAD_YAW_FORTH            KEY_PRESSED_OFFSET_W    //ǰ��
#define BAD_YAW_BACK             KEY_PRESSED_OFFSET_S    //����
#define BAD_YAW_LEFT_MOVE        KEY_PRESSED_OFFSET_A    //��ƽ��
#define BAD_YAW_RIGHT_MOVE       KEY_PRESSED_OFFSET_D    //��ƽ��

#define BAD_YAW_BF_RC            HANDLE_LEFT_BF      //ǰ��,����
#define BAD_YAW_LR_MOVE_RC       HANDLE_LEFT_LR      //��ת,��ת

///////////////////////////////////////////////////////////////////////

//****************************�����˿��Ƴ���***************//
#define KEYBOARD_CONTROL_ROBOT_SPEED_X 0.6f      //����wasd���Ƶ��ٶ�
#define KEYBOARD_CONTROL_ROBOT_SPEED_Y 0.6f
#define KEYBOARD_CONTROL_ROBOT_SPEED_W 0.5f
#define SPINNER_W   1.0f        //С�����˶���ת�ٶ�
#define SPINNER_MAX_ROUNDS  20   //С�����˶�ʱ�����̺���̨�����Ȧ��������������С����ģʽ������̨ 233
#define ECD_FULL_ROUND 8192 //һȦ��ECDֵ��ʵ��ȡֵ0-8191


//****************************����������ϵ��**************//
#define CHASSIS_VX_RC_SEN 0.0005f   //��ң����ҡ�˶���ת��Ϊ�ٶȵ�ϵ����ң����-660~+660.�ɵ����0.33m/s
#define CHASSIS_VY_RC_SEN -0.0005f   //�Ǹ�����ң����������������ƽ�ƣ�ң����ҡ�˶���Ϊ����������ϵ��Ϊ��y������
#define CHASSIS_WZ_RC_SEN -0.0005f   //�����ת���ٶȡ�0.33m/s  //�Ǹ�����ң��������������ת

#define CHASSIS_RC_DEADLINE 10  //�������ֵ��ҡ�˶���һ��Ϊ0

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f   //��������ٶ�ת��Ϊ�����˵����ٶȡ�����ֱ��15cm���뾶8cm
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f  
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f  

#define MOTOR_DISTANCE_TO_CENTER 0.27f           //��������������ĵľ��� 27cm��������solidworksװ�����в���
// #define M3508_MOTOR_RPM_TO_VECTOR 0.0083f   //���̵��ת�ٵ����ٶȣ�m/s��ת�������㣺n*2*��*R/60
#define M3508_MOTOR_RPM_TO_VECTOR 0.0004f        //Ӧ�û�Ҫ���������Լ20.��һ��Ӧ���Ǽ��ٵ����ת��Ϊʵ�ʵ��ٶȻ�Ҫ���Լ��ٱȡ�
                                                //���ֵ����

#define BAD_YAW_DATA_ROTATE_COEF 17.5f       //��yawģʽ�£�����̨��תPID��õ���ת�ٶȣ�����yaw pid�Ĳ����ƣ���õ�
                                                //������ʵ��ת�ٶ�,��Ҫ�ȳ���һ��ϵ����

//***************************PID���Ʋ���*****************//
#define MAX_MOTOR_CAN_CURRENT 15000.0f  //����3508���can���͵���ֵ
//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 16000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f



#define MAX_WHEEL_SPEED 4.0f    //�������̵������ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f //�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f //�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_W 0.9f

//*************************������س���*******************//
// #define AUTO_DETECT_INIT_ECD            //������ʼʱ��ȡ�������ECD�����������򸲸�get���µ�Ԥ�Ȳ���ֵ
#define INIT_PRE_GOT_MID_ECD    3557    //Ԥ�Ȳ�������̨������ʱ��yawECD���ڻ��������°�װ��̨�󣬿��ܵ���ECD�������һ��
#define CHASSIS_TASK_INIT_TIME 500      //�����ʼ��ʱ��
#define CHASSIS_CONTROL_TIME_MS 5       //ÿ5msִ��һ������

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

//******************�����˲����賣��****************// ϵ���Ϳ��Ʒ������Թٷ�����
#define CHASSIS_ACCEL_X_NUM 150.1666f
#define CHASSIS_ACCEL_Y_NUM 150.1666f
#define CHASSIS_ACCEL_W_NUM 150.66f


//**************************�������ṹ��*******************//

// ����ϵ
enum MovingAxis_e{
    MovingAxis_e_GimbalAxis,
    MovingAxis_e_ChassisAxis,
};

//�����˿�����
struct RobotControl_s{
    fp32 vx,vy,w;
    first_order_filter_type_t vx_filter,vy_filter,w_filter;
    enum MovingAxis_e axis;
};

//��̨yaw�������ڵ��̽ǶȽṹ��
struct Angle_s{
    int16_t initGimbalYawECD;       //����ʱ��ECD
    int16_t nowGimbalYawECD;        //Ŀǰ��ECD
    int32_t rotateRounds;           //Ŀǰ��ת�˶���Ȧ������׼С����ģʽʱ����ֹ�߶ϵ���
    fp32    gimbalAngleFromChassis; //��̨���������ĽǶȣ���ʱ��Ϊ�����򡣵�λ���ȡ�
};

//���̵������
struct MotorControl_s{
    pid_type_def    vpid;
    fp32            presentMotorSpeed;
    fp32            wantedMotorSpeed;
    int16_t         giveCurrent;
};


//************************���ڿ��Ƶ�ȫ�ֱ���**********//
static int zeroCurrentMark;    //������ʱ��ֱ�ӷ����������ʱ��
static struct Angle_s someAngle;   //���ڼ�¼���е���̨�����̽Ƕ����ֵ
static struct MotorControl_s driveMotor[4];
static const RC_ctrl_t *rc_p;   //ң����λ��ָ�룬��Ҫ��ʼ��
static const toSTM32_t *nuc_p;  //NUC����λ�á�δ������ȫ�Զ�������ʱ��Ҫ
const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD}; //PID ��ʼ������
static const enum RobotState_e *robotMode;   //������ģʽ
static struct RobotControl_s robotTotalSpeedControl;
static int16_t vx_channel,vy_channel,w_channel; //ң����deadband limit�������ֵ

const struct GimbalToChassis_s * badYawData;    //��yawģʽ�£���ת����
const int16_t * gimbalRounds;


//***************�������ñ���**************************//
#ifdef OLED_DEBUG_GIMBAL_TASK_ON
static int time_passed_debug=0;
static fp32 pidOUT;
#endif
#ifdef TEST_TASK_TIME
static uint32_t taskTestStartTime,timeConsume;
#endif

//************************�������ܺ���*************//
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

static void analyseTotalControl(void)    //ͨ��ң�����ͼ��̲��������ٶȡ������������Ϊ0ʱ��ͨ��ң����ҡ��ȷ���ٶȡ�
{
    // int keyBoardMove=0;
    //ע�⽨ϵ�ķ���
    robotTotalSpeedControl.axis=MovingAxis_e_GimbalAxis;    //Ĭ����̨����ϵ�������ڲ����ֵĿ���
    robotTotalSpeedControl.vx=0;robotTotalSpeedControl.vy=0;robotTotalSpeedControl.w=0;
    
    if(RobotState_e_Powerless==*robotMode)
    {
        return;
    }
    else if(RobotState_e_CommonCar==*robotMode)
    {
        //��������ϵ
        robotTotalSpeedControl.axis=MovingAxis_e_ChassisAxis;
        if(rc_p->key.v & COMMON_FORTH)  //ǰ
            robotTotalSpeedControl.vx-=KEYBOARD_CONTROL_ROBOT_SPEED_X;
        if(rc_p->key.v & COMMON_BACK)  //��
            robotTotalSpeedControl.vx+=KEYBOARD_CONTROL_ROBOT_SPEED_X;
        if(rc_p->key.v & COMMON_LEFT_MOVE)  //��
            robotTotalSpeedControl.vy-=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
        if(rc_p->key.v & COMMON_RIGHT_MOVE)  //��
            robotTotalSpeedControl.vy+=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
        if(rc_p->key.v & COMMON_LEFT_ROTATE)    //���� �������¿�Ϊ��ʱ��
            robotTotalSpeedControl.w+=KEYBOARD_CONTROL_ROBOT_SPEED_W;
        if(rc_p->key.v & COMMON_RIGHT_ROTATE)   //����
            robotTotalSpeedControl.w-=KEYBOARD_CONTROL_ROBOT_SPEED_W;

        rc_deadband_limit(rc_p->rc.ch[COMMON_BF_RC],vx_channel,CHASSIS_RC_DEADLINE);
        rc_deadband_limit(rc_p->rc.ch[COMMON_LR_ROTATE_RC],w_channel,CHASSIS_RC_DEADLINE);
        robotTotalSpeedControl.vx +=vx_channel*CHASSIS_VX_RC_SEN;
        robotTotalSpeedControl.w +=w_channel*CHASSIS_WZ_RC_SEN;

    }
    else if(RobotState_e_BadYawCar==*robotMode)
    {
        //��̨����ϵ
        robotTotalSpeedControl.axis=MovingAxis_e_GimbalAxis;

        if(rc_p->key.v & BAD_YAW_FORTH)  //ǰ
            robotTotalSpeedControl.vx-=KEYBOARD_CONTROL_ROBOT_SPEED_X;
        if(rc_p->key.v & BAD_YAW_BACK)  //��
            robotTotalSpeedControl.vx+=KEYBOARD_CONTROL_ROBOT_SPEED_X;
        if(rc_p->key.v & BAD_YAW_LEFT_MOVE)  //��
            robotTotalSpeedControl.vy-=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
        if(rc_p->key.v & BAD_YAW_RIGHT_MOVE)  //��
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
        //��̨����ϵ
        robotTotalSpeedControl.axis=MovingAxis_e_GimbalAxis;
        if(rc_p->key.v & SPINNER_FORTH)  //ǰ
            robotTotalSpeedControl.vx+=KEYBOARD_CONTROL_ROBOT_SPEED_X;
        if(rc_p->key.v & SPINNER_BACK)  //��
            robotTotalSpeedControl.vx-=KEYBOARD_CONTROL_ROBOT_SPEED_X;
        if(rc_p->key.v & SPINNER_LEFT_MOVE)  //��
            robotTotalSpeedControl.vy+=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
        if(rc_p->key.v & SPINNER_RIGHT_MOVE)  //��
            robotTotalSpeedControl.vy-=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
        rc_deadband_limit(rc_p->rc.ch[SPINNER_BF_RC],vx_channel,CHASSIS_RC_DEADLINE);
        rc_deadband_limit(rc_p->rc.ch[SPINNER_LR_MOVE_RC],vy_channel,CHASSIS_RC_DEADLINE);
        robotTotalSpeedControl.vx += vx_channel*CHASSIS_VX_RC_SEN;
        robotTotalSpeedControl.vy += vy_channel*CHASSIS_VY_RC_SEN;
        //������ת�ٶȿ��Ƴ���
        //��򵥵ģ�����spinner �ٶ�
        // ����̨���������ȡ��̨����ڵ���ת����Ȧ������̨��ʱ��תΪ��Ȧ���������ﶥ�壬���õ���Ҳ��ʱ��ת��
        static fp32 nowSpinnerW=SPINNER_W; //��ʼ��������ʱ����ת (��֣�һ��Ū������ø��ŷ�����)
        if(*gimbalRounds>=SPINNER_MAX_ROUNDS)   //��̨��ʱ��ת����
            nowSpinnerW=SPINNER_W;  //������ʱ�����
        else if((*gimbalRounds)<=-SPINNER_MAX_ROUNDS) //��̨˳ʱ��ת����
            nowSpinnerW=-SPINNER_W;  //����˳ʱ�����
        robotTotalSpeedControl.w=nowSpinnerW;
    }    
}

static void refreshECD(void)   //���µ�ǰ��̨yawECDֵ�ͻ����Ƶ�ֵ��ʵ����̨����ϵ�����Լ�С����
{
    someAngle.nowGimbalYawECD=get_yaw_gimbal_motor_measure_point()->ecd;
    someAngle.gimbalAngleFromChassis=(someAngle.nowGimbalYawECD-someAngle.initGimbalYawECD)*2*PI/ECD_FULL_ROUND;
}

static void calcWheelVelocityInChassisAxis(void)   //���������е�����ϵ�ٶ�ת��Ϊ��������ϵ���ٶȣ�����������ת���ٶ�
{
    fp32 sin_yaw,cos_yaw;
    fp32 vx,vy,w;
    sin_yaw=arm_sin_f32(someAngle.gimbalAngleFromChassis);
    cos_yaw=arm_cos_f32(someAngle.gimbalAngleFromChassis);
    w=robotTotalSpeedControl.w;
    // robotTotalSpeedControl.axis=MovingAxis_e_GimbalAxis;
    if(robotTotalSpeedControl.axis==MovingAxis_e_GimbalAxis)
    {
        //����̨����ϵ�ٶ�ת��Ϊ��������ϵ�ٶȡ�����ϵ�任
        vx=cos_yaw*robotTotalSpeedControl.vx-sin_yaw*robotTotalSpeedControl.vy;
        vy=sin_yaw*robotTotalSpeedControl.vx+cos_yaw*robotTotalSpeedControl.vy;
        
    }
    else if(robotTotalSpeedControl.axis==MovingAxis_e_ChassisAxis)
    {
        vx=robotTotalSpeedControl.vx;
        vy=robotTotalSpeedControl.vy;
    }
    driveMotor[0].wantedMotorSpeed=-vx-vy-w;    //�����ٶȼ��㹫ʽ
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

static void calcGiveCurrent(void) //pid �������
{
    uint8_t i;
    for(i=0;i<4;i++)
    {
        driveMotor[i].presentMotorSpeed=get_chassis_motor_measure_point(i)->speed_rpm*M3508_MOTOR_RPM_TO_VECTOR;    //��ȡ���ת��
        PID_calc(&(driveMotor[i].vpid),driveMotor[i].presentMotorSpeed,driveMotor[i].wantedMotorSpeed);   //pid����
        driveMotor[i].giveCurrent=driveMotor[i].vpid.out;    //pid�����������������
    }
}


void chassis_task(void const *pvParameters)
{
    osDelay(CHASSIS_TASK_INIT_TIME);    //�ȴ������ʼ��
    initPIDs();                         //��ʼ��PID
    initFilters();
    rc_p=get_remote_control_point();    //��ȡң�������ݺ�NUC����ָ��
    nuc_p=get_nuc_control_point();
    robotMode=getRobotPresentMode();
    badYawData=getBadYawWantedSpd();    //��ʼ��badYaw����������
    gimbalRounds=getGimbalNowRoundsPoint(); //��ȡ��̨��תȦ��ָ��
    #ifdef AUTO_DETECT_INIT_ECD     //����������ʼECD
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
            CAN_cmd_chassis(0,0,0,0);       //���������
        else
            CAN_cmd_chassis(driveMotor[0].giveCurrent,driveMotor[1].giveCurrent,
                driveMotor[2].giveCurrent,driveMotor[3].giveCurrent);   //�������Ƶ��
        

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
