/**
  ********************************************************
	�����ʵ��ѧ-���ӽ�-ȫ���ֵ�������       ����޸����ڣ�2023.3.14
  ==============================================================================
	��������Ƶ������ֺ�YAW��
  �ұ߲��˿��Ƶ����˶�ģʽ��
	�ϲ�Ϊ������̨ģʽ��
	�в�Ϊ���̵����˶�ģʽ��
	�²�ΪС����ģʽ��
	ͨ�����ͨ��CANͨ�Ŵ���ģʽ�ͽǶ�����
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "referee.h"
#include "kalman.h"

//���Ͷ���
#define OMNI_CHASSIS 
//#define MEC_CHASSIS 

//ң������������
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
//�޷����� //test done
#define range_limit_inside(toBeLimited, range)                      \
    {                                                               \
         if((toBeLimited)>(range))                                  \
        {                                                           \
            (toBeLimited)=(range);                                  \
        }                                                           \
        else if((toBeLimited)<-(range))                             \
        {                                                           \
            (toBeLimited)=-(range);                                 \
        }                                                           \
    }
		//************************���ڿ��Ƶ���**********//
#define YAW_INIT_ECD 2500 //yaw�����ʱ��ĳ�ʼ�����͵����װ�йأ�������
//*************************��������********************//
// ʹ��ң�����ͼ��̿�����̨����������Ϊ�ٶ�
// ң�����ͼ��̿��ƶ����ٶȣ����������ٶ�
// NUC����ʱ��Ŀ�������Ϊ�Ƕ�
#define YAW_RC_SEN      -0.000004f
#define PITCH_RC_SEN    -0.000005f //0.005
#define YAW_MOUSE_SEN   -0.00005f
#define PITCH_MOUSE_SEN -0.00005f
// ������������ģʽ����������ͨ������̨������׼��С���ݳ������ݻ�е�����Զ��ж��Ƿ�仯���򣩡�
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
#define COMMON_LR_RC            HANDLE_LEFT_LR      //��ƽ��,��ƽ��
#define COMMON_LR_ROTATE_RC     HANDLE_RIGHT_LR     //��ת����ת

//С����ģʽ    spinner������һ����ת������v�����԰�v����С����ģʽ������ң�����Ҳ������м����
//ƽ�ƿ��Ƶ�����ϵΪ��̨����ϵ
#define SPINNER_FORTH            KEY_PRESSED_OFFSET_W    //ǰ��
#define SPINNER_BACK             KEY_PRESSED_OFFSET_S    //����
#define SPINNER_LEFT_MOVE        KEY_PRESSED_OFFSET_A    //��ƽ��
#define SPINNER_RIGHT_MOVE       KEY_PRESSED_OFFSET_D    //��ƽ��
//ң������ҡ�˿���ǰ��ƽ�ƺ�����ƽ��
#define SPINNER_BF_RC            HANDLE_LEFT_BF      //ǰ��,����
#define SPINNER_LR_MOVE_RC       HANDLE_LEFT_LR      //��ת,��ת
//****************************�����˿��Ƴ���***************//
#define KEYBOARD_CONTROL_ROBOT_SPEED_X 2.8f      //����wasd���Ƶ��ٶ�
#define KEYBOARD_CONTROL_ROBOT_SPEED_Y 2.8f
#define KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_X 4.5f
#define KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_Y 4.5f
#define KEYBOARD_CONTROL_ROBOT_SPEED_W 1.0f
#define SPINNER_W   6.5f        //С�����˶���ת�ٶ�
#define SPINNER_MAX_ROUNDS  20   //С�����˶�ʱ�����Ȧ�������������õ�
#define ECD_FULL_ROUND 8192 //һȦ��ECDֵ��ʵ��ȡֵ0-8191

//yaw����PID���˲�
#define YAW_SPD_KP 1560000.0f
#define YAW_SPD_KI 0.0f
#define YAW_SPD_KD 1000.0f

#define YAW_VOLT_MAX_OUT  30000.0f
#define YAW_VOLT_MAX_IOUT 2000.0f

// #define YAW_AGL_KP 0.07f
#define YAW_AGL_KP 0.03f
#define YAW_AGL_KI 0.0005f
#define YAW_AGL_KD 0.0f

#define YAW_AGL_SPD_MAX_OUT (50.0f)
#define YAW_AGL_SPD_MAX_IOUT (11.7f)

#define YAW_SPD_FILTER_NUM 0.001f
//************************�������ṹ�嶨��**********//
//���̵��ֱ�ӿ��ƣ�ֻ���ٶ�
struct MotorControl_s{
    pid_type_def    vpid;
    fp32            presentMotorSpeed;
    fp32            wantedMotorSpeed;
    int16_t         giveCurrent;
};
//����ϵ����
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
//��̨yaw����
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

    //PIDs
    pid_type_def spd_pid;       //���ڿ����ٶȵ�PID����������ٶ� rad/s, ���GM6020��ѹ int16
    pid_type_def agl_pid;       //���ڿ��ƽǶȵ�PID��������Ƕ� rad,������ٶ�rad/s

    //������
    int16_t giveVolt;       //GM6020���͵�ѹ
    uint8_t zeroVoltMark;

    // һ���˲��������˽��ٶ�
    first_order_filter_type_t spd_filter;
};
//��̨yaw�������ڵ��̽ǶȽṹ��
struct Angle_s{
    int16_t initGimbalYawECD;       //����ʱ��ECD
    int16_t nowGimbalYawECD;        //Ŀǰ��ECD
    int32_t rotateRounds;           //Ŀǰ��ת�˶���Ȧ��
    fp32    gimbalAngleFromChassis; //��̨���������ĽǶȣ���ʱ��Ϊ�����򡣵�λ���ȡ�
};

//************************���ڿ��Ƶ�ȫ�ֱ���**********//
const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD}; //PID ��ʼ������
static struct MotorControl_s driveMotor[4];
static struct Angle_s relativeAngle;   //���ڼ�¼��̨�����̽Ƕ����ֵ,��̨���ƺ�С����Ҫ��
static const RC_ctrl_t *rc_p;   //ң����λ��ָ�룬��Ҫ��ʼ��
static enum RobotState_e robotMode;   //������ģʽ
static struct RobotControl_s robotTotalSpeedControl;
static int16_t vx_channel,vy_channel,w_channel; //ң����deadband limit�������ֵ
static const motor_measure_t* yaw_measure;
const int16_t * gimbalRounds;
static int16_t vx_channel,vy_channel,w_channel; //ң����deadband limit�������ֵ
//yaw����
static struct gimbalMotorCtrl_s gimbalYawCtrl;
static struct gimbalMotorCtrl_s *gimbal_yaw_ctrl_point=&gimbalYawCtrl;
static fp32 yawSpdPIDco[3]={YAW_SPD_KP,YAW_SPD_KI,YAW_SPD_KD};
static fp32 yawAglPIDco[3]={YAW_AGL_KP,YAW_AGL_KI,YAW_AGL_KD};
extKalman_t spd_pid_kalman_out;
//************************���յ��Ŀ��Ʊ���**********//
static const can_send_data_s* YAW;
static const can_send_data_channel_s* RC_channel;
static const can_send_data_nuc_yaw_s* NUC_YAW;
static const can_send_data_s* keyboard;

enum RobotState_e getRobotPresentMode()
{
	if(toe_is_error(DBUS_TOE))
		return RobotState_e_Powerless;
	if(switch_is_up(rc_p->rc.s[CHASSIS_MODE_CHANNEL]))
		return RobotState_e_GimbalCar;
	else if(switch_is_mid(rc_p->rc.s[CHASSIS_MODE_CHANNEL]))
		return RobotState_e_CommonCar;
	else if(switch_is_down(rc_p->rc.s[CHASSIS_MODE_CHANNEL]))
		return RobotState_e_Spinner;
}
static void initChassis(void){
	//PID��ʼ��
	int i;
	for(i=0;i<4;i++)
		PID_init(&(driveMotor[i].vpid),PID_POSITION,motor_speed_pid,M3505_MOTOR_SPEED_PID_MAX_OUT,M3505_MOTOR_SPEED_PID_MAX_IOUT);
	//�˲���ʼ��
  const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  const static fp32 chassis_w_order_filter[1] = {CHASSIS_ACCEL_W_NUM}; 
  first_order_filter_init(&(robotTotalSpeedControl.vx_filter), CHASSIS_CONTROL_TIME_MS, chassis_x_order_filter);
  first_order_filter_init(&(robotTotalSpeedControl.vy_filter), CHASSIS_CONTROL_TIME_MS, chassis_y_order_filter);
	first_order_filter_init(&(robotTotalSpeedControl.w_filter), CHASSIS_CONTROL_TIME_MS, chassis_w_order_filter);
  //yaw�����ʼ��
	relativeAngle.initGimbalYawECD=YAW_INIT_ECD;
	yaw_measure=get_yaw_gimbal_motor_measure_point();
	relativeAngle.nowGimbalYawECD=yaw_measure->ecd;
	for(int i=0;i<4;i++){
	  driveMotor[i].presentMotorSpeed=get_chassis_motor_measure_point(i)->speed_rpm;
	}
}

//����ģʽ��ȡͨ��ֵ
static void analyseTotalControl(){
	robotTotalSpeedControl.axis=MovingAxis_e_ChassisAxis;//Ĭ�ϵ�������ϵ
	robotTotalSpeedControl.vx=robotTotalSpeedControl.vy=robotTotalSpeedControl.w=0;
	
	if(robotMode==RobotState_e_Powerless){
		return;
	}
	else if(RobotState_e_CommonCar==robotMode)
    {
        //��������ϵ
        robotTotalSpeedControl.axis=MovingAxis_e_ChassisAxis;
        

          if(RC_channel->E)  //ǰ
              robotTotalSpeedControl.vx-=KEYBOARD_CONTROL_ROBOT_SPEED_X;
          if(RC_channel->D)  //��
              robotTotalSpeedControl.vx+=KEYBOARD_CONTROL_ROBOT_SPEED_X;
          if(RC_channel->S)  //��
              robotTotalSpeedControl.vy-=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
          if(RC_channel->F)  //��
              robotTotalSpeedControl.vy+=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
					if(RC_channel->A)  //��
              robotTotalSpeedControl.w-=KEYBOARD_CONTROL_ROBOT_SPEED_W;
					if(RC_channel->G)  //��
              robotTotalSpeedControl.w+=KEYBOARD_CONTROL_ROBOT_SPEED_W;


					rc_deadband_limit(RC_channel->channel_3,vx_channel,CHASSIS_RC_DEADLINE);
					rc_deadband_limit(RC_channel->channel_2,vy_channel,CHASSIS_RC_DEADLINE);
					rc_deadband_limit(RC_channel->channel_0,w_channel,CHASSIS_RC_DEADLINE);
					robotTotalSpeedControl.vx -=vx_channel*CHASSIS_VX_RC_SEN;
          robotTotalSpeedControl.vy +=vy_channel*CHASSIS_VY_RC_SEN;
          robotTotalSpeedControl.w -=w_channel*CHASSIS_WZ_RC_SEN;
//				}
    }
	//��̨����С����ʹ����̨����ϵ����ʵ���Ǻ��ʼ�ĵ����й�
  //�������Ʒ�ʽ��ͬ
	else if(robotMode==RobotState_e_GimbalCar||robotMode==RobotState_e_Spinner)
   {
				robotTotalSpeedControl.axis=MovingAxis_e_GimbalAxis;
				
		 if (robotMode==RobotState_e_Spinner) {
				if(RC_channel->W)  //ǰ
						robotTotalSpeedControl.vx-=KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_X;
				if(RC_channel->S)  //��
						robotTotalSpeedControl.vx+=KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_X;
				if(RC_channel->A)  //��
						robotTotalSpeedControl.vy-=KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_Y;
				if(RC_channel->D)  //��
						robotTotalSpeedControl.vy+=KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_Y;
			} else {
				if(RC_channel->W)  //ǰ
						robotTotalSpeedControl.vx-=KEYBOARD_CONTROL_ROBOT_SPEED_X;
				if(RC_channel->S)  //��
						robotTotalSpeedControl.vx+=KEYBOARD_CONTROL_ROBOT_SPEED_X;
				if(RC_channel->A)  //��
						robotTotalSpeedControl.vy-=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
				if(RC_channel->D)  //��
						robotTotalSpeedControl.vy+=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
			}
			
			  rc_deadband_limit(RC_channel->channel_3,vx_channel,CHASSIS_RC_DEADLINE);
				rc_deadband_limit(RC_channel->channel_2,vy_channel,CHASSIS_RC_DEADLINE);
			  robotTotalSpeedControl.vx -= vx_channel*CHASSIS_VX_RC_SEN;
        robotTotalSpeedControl.vy += vy_channel*CHASSIS_VY_RC_SEN;

			if(robotMode==RobotState_e_Spinner){
				robotTotalSpeedControl.w=SPINNER_W;
			}
		
	 }
}
//�õ��µ�ECDֵ����������ԽǶ�
void refreshECD(){
  relativeAngle.nowGimbalYawECD=get_yaw_gimbal_motor_measure_point()->ecd;
	relativeAngle.gimbalAngleFromChassis=(relativeAngle.nowGimbalYawECD-relativeAngle.initGimbalYawECD)*2*PI/ECD_FULL_ROUND;
	//usart_printf("%d %f\r\n",relativeAngle.nowGimbalYawECD,relativeAngle.gimbalAngleFromChassis);
}
//ƽ��������������������
static void firstOrderFilt()
{
    first_order_filter_cali(&robotTotalSpeedControl.vx_filter
	,robotTotalSpeedControl.vx);
    first_order_filter_cali(&robotTotalSpeedControl.vy_filter,robotTotalSpeedControl.vy);
    first_order_filter_cali(&robotTotalSpeedControl.w_filter,robotTotalSpeedControl.w);
    robotTotalSpeedControl.vx=robotTotalSpeedControl.vx_filter.out;
    robotTotalSpeedControl.vy=robotTotalSpeedControl.vy_filter.out;
    robotTotalSpeedControl.w=robotTotalSpeedControl.w_filter.out;
}
static void calcWheelVelocity(){
	fp32 sin_yaw,cos_yaw;
	fp32 vx,vy,w;
	sin_yaw=arm_sin_f32(relativeAngle.gimbalAngleFromChassis);
	cos_yaw=arm_cos_f32(relativeAngle.gimbalAngleFromChassis);
	
	w=robotTotalSpeedControl.w;
	if(robotTotalSpeedControl.axis==MovingAxis_e_GimbalAxis){
		vx=cos_yaw*robotTotalSpeedControl.vx-sin_yaw*robotTotalSpeedControl.vy;
		vy=sin_yaw*robotTotalSpeedControl.vx+cos_yaw*robotTotalSpeedControl.vy;
	}
	else if(robotTotalSpeedControl.axis==MovingAxis_e_ChassisAxis)
    {
        vx=robotTotalSpeedControl.vx;
        vy=robotTotalSpeedControl.vy;
    }
		#ifdef OMNI_CHASSIS
	  driveMotor[0].wantedMotorSpeed=-vx+vy-w;
	  driveMotor[1].wantedMotorSpeed=-vx-vy-w;
	  driveMotor[2].wantedMotorSpeed=vx-vy-w;
		driveMotor[3].wantedMotorSpeed=vx+vy-w;
		#endif
		#ifdef MEC_CHASSIS
	  driveMotor[0].wantedMotorSpeed=-vx-vy-w;    //�����ٶȼ��㹫ʽ
    driveMotor[1].wantedMotorSpeed=vx-vy-w;
    driveMotor[2].wantedMotorSpeed=vx+vy-w;
    driveMotor[3].wantedMotorSpeed=-vx+vy-w;
		#endif
}
static void calcGivenCurrent(){
   uint8_t i;   
	 chassis_move_t chassis_power_control_data;

    for(i=0;i<4;i++)
    {
        driveMotor[i].presentMotorSpeed=get_chassis_motor_measure_point(i)->speed_rpm*M3508_MOTOR_RPM_TO_VECTOR;    //��ȡ���ת��
        PID_calc(&(driveMotor[i].vpid),driveMotor[i].presentMotorSpeed,driveMotor[i].wantedMotorSpeed);   //pid����
        driveMotor[i].giveCurrent=driveMotor[i].vpid.out;    //pid�����������������
    }
		for(i = 0; i < 4; i++)
    {
        chassis_power_control_data.motor_speed_pid[i].out = driveMotor[i].giveCurrent;
    }
		   chassis_power_control(&chassis_power_control_data);
    for(i = 0; i < 4; i++)
    {
        driveMotor[i].giveCurrent = chassis_power_control_data.motor_speed_pid[i].out;
    }
}
static void initGimbalYaw(void)
{
    uint8_t i;
    //��ʼ�����ԽǶ�λ��
    gimbalYawCtrl.anglePoint=&YAW->yaw; 

    // ��ʼ��ECD
    gimbalYawCtrl.ECDPoint=&(get_yaw_gimbal_motor_measure_point()->ecd);


    //��ʼ��ת���ĽǶ�
    gimbalYawCtrl.maxRounds=30000;


    // ��ʼ��PID����
	  PID_init(&(gimbal_yaw_ctrl_point->agl_pid),PID_POSITION,yawAglPIDco,YAW_AGL_SPD_MAX_OUT,YAW_AGL_SPD_MAX_IOUT);
    PID_init(&(gimbal_yaw_ctrl_point->spd_pid),PID_POSITION,yawSpdPIDco,YAW_VOLT_MAX_OUT,YAW_VOLT_MAX_IOUT);
    
    //�˲���ʼ��
    const static fp32 gimbal_yaw_order_filter[1] = {YAW_SPD_FILTER_NUM};

		KalmanCreate(&spd_pid_kalman_out,1,30);
    first_order_filter_init(&(gimbalYawCtrl.spd_filter), CHASSIS_CONTROL_TIME_MS, gimbal_yaw_order_filter);
    
    //��ʼ����ǰ�ǶȺ�Ŀ��Ƕȡ�pid��������̱�ջ������
		gimbal_yaw_ctrl_point->zeroVoltMark=0;
		gimbal_yaw_ctrl_point->nowAbsoluteAngle=*(gimbal_yaw_ctrl_point->anglePoint);
		gimbal_yaw_ctrl_point->nowTime=HAL_GetTick();
		gimbal_yaw_ctrl_point->wantedAbsoluteAngle=gimbal_yaw_ctrl_point->nowECD;
    
		
		PID_clear(&(gimbalYawCtrl.spd_pid));
    PID_clear(&(gimbalYawCtrl.agl_pid));
    
}
void refreshAngleStates(struct gimbalMotorCtrl_s * c)
{
    //��ǰ��̬�Ǻͽ��ٶ�
    c->lastAbsoluteAngle=c->nowAbsoluteAngle;
    c->lastTime=c->nowTime;

    c->nowAbsoluteAngle=*(c->anglePoint);
    c->nowTime=xTaskGetTickCount();

    c->radSpeed=(c->nowAbsoluteAngle-c->lastAbsoluteAngle)/(c->nowTime-c->lastTime);
    
    //��ǰECD�Ƕ�
    c->nowECD=*(c->ECDPoint);
}
/**
 * @brief ���ݻ�����״̬����pitch��yaw ��ת������ߴ������̵��ٶ�
 * 
 */
int y_data=0;
void getControlAngles(void)
{

    static fp32 selfTargetOffsetPitch=0,selfTargetOffsetYaw=0;
    static int16_t yaw_channel = 0, pitch_channel = 0;
    rc_deadband_limit(RC_channel->channel_0, yaw_channel, CHASSIS_RC_DEADLINE);
    //�����������ң����������̨rad�ٶȵĿ���

    //δ�������������Ժ������ƽ�����ӣ���������һ���Ƕȣ�����������ƫ������ƫ�Ƶ�����
    
    //������bad yaw carת������ģʽ����̨��ͻȻת��������Ϊ��̨û�е�����ϣ����λ�����ǣ�����һ��ת�����̡�
    // ����һ��ģʽ����һ��ģʽ��ͬʱ����ϣ���ĽǶ�����Ϊ��ǰ�Ƕȡ�
    static enum RobotState_e lastMode=RobotState_e_Powerless;   //��ʼ��Ϊ����
    
    lastMode=robotMode;
	  gimbalYawCtrl.wantedAbsoluteAngle=0.0f;
	  

    gimbalYawCtrl.zeroVoltMark=0;

//����ģʽѡ���־

    if(robotMode==RobotState_e_Powerless)
    {
        gimbalYawCtrl.zeroVoltMark=1;
    }    
    else 
    {
    
     	gimbalYawCtrl.wantedAbsoluteAngle=YAW->nuc_yaw;
      //usart_printf("%f,%f\r\n",gimbalYawCtrl.wantedAbsoluteAngle,gimbalYawCtrl.nowAbsoluteAngle);			
			
			
    }
    if(rc_p->key.v&KEY_PRESSED_OFFSET_Z)
			CAN_cmd_chassis_reset_ID();
    
    
}
/**
 * @brief ���ǶȻ�Ϊ(-PI,PI)��Χ��
 * 
 * @param rawAngle 
 * @return fp32 
 */
fp32 radFormat(fp32 rawAngle)   //test done
{
    while(rawAngle>PI)
        rawAngle-=(2*PI);
    while(rawAngle<(-PI))
        rawAngle+=(2*PI);
    return rawAngle;
}

/**
 * @brief ��ECD��ֵ�Ȼ�Ϊ(0,8191)��Χ
 * 
 * @param rawECD 
 * @return uint16_t 
 */
static uint16_t ECDFormat(int16_t rawECD)     //test done
{
    while(rawECD<0)
        rawECD+=ECD_FULL_ROUND;
    while(rawECD>=ECD_FULL_ROUND)
        rawECD-=ECD_FULL_ROUND;
    return (uint16_t)rawECD;
}

/**
 * @brief ��0-8191��ECDֵת��Ϊ-PI~PI�Ļ���ֵ,ֻ��ӳ��һһ��ȥ
 * 0->-PI,8192->PI
 * 
 * @param ecd 
 * @return fp32 
 */
fp32 ECD2Rad(uint16_t ecd)  //test done
{
    return ((fp32)ecd)*(2*PI)/ECD_FULL_ROUND-PI;
}
void limitAngles(struct gimbalMotorCtrl_s * c)
{
    // �������2PI�Ƕ�ʱ,���Ʒ���   //test done
    if(radFormat(c->wantedAbsoluteAngle - c ->nowECD) > 4096)
        c->wantedAbsoluteAngle = c ->nowECD + 4096;
    if(radFormat(c->wantedAbsoluteAngle - c ->nowECD) < -4096)
        c->wantedAbsoluteAngle = c ->nowECD - 4096;
    
#ifndef HAVE_SLIP_RING
    // �ﵽ�򳬹����Ȧ���һ�Ҫ��ǿԽ�����תʱ
    fp32 posiSafe=radFormat(ECD2Rad(c->maxECD)-ECD2Rad(c->nowECD));      //����0
    fp32 negSafe=radFormat(ECD2Rad(c->minECD)-ECD2Rad(c->nowECD));       //С��0
#endif

    //��������ת�ٶȡ���ת�ٶȽ������̿�������ȥ����
}
fp32 PID_spd_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        range_limit_inside(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        range_limit_inside(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        range_limit_inside(pid->out, pid->max_out);
    }
    return pid->out;
}
fp32 gimbal_PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = radFormat(set - ref);
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        range_limit_inside(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        range_limit_inside(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
			  if(pid->error[0]<=0){
					pid->error[0]=0;
				
				}
				else{
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
				}
    }
    return pid->out;
}
struct gimbalMotorCtrl_s c;

void calcPID(void)
{
    c=*gimbal_yaw_ctrl_point;
	   
      	gimbal_PID_calc(&(c.agl_pid),c.nowAbsoluteAngle,c.wantedAbsoluteAngle);  // �غ���ת�����PID������
        // �����������ת�ٶȡ�
        //���ٶȽ���һ���˲�
        first_order_filter_cali(&(c.spd_filter),(c.agl_pid).out);

        PID_spd_calc(&(c.spd_pid),c.radSpeed,(c.spd_filter).out);   // ��ͨ���ٶȿ��ƻ�
	      first_order_filter_cali(&(c.spd_filter),(c.spd_pid).out);
        gimbal_yaw_ctrl_point->giveVolt=c.spd_pid.out;     //����ѹ
	//����yaw��pid
	      // usart_printf("%f,%f,%f,%f,%f\r\n",c.nowAbsoluteAngle,c.wantedAbsoluteAngle,c.agl_pid.out,gimbal_yaw_ctrl_point->giveVolt,c.spd_pid.out);
}

//���̿�������
void chassis_task(void const *pvParameters)
{
	osDelay(CHASSIS_TASK_INIT_TIME);//�����ʼ��

	rc_p=get_remote_control_point();
	YAW=get_yaw_measure_point();
	RC_channel=get_channel_measure_point();
	
	initChassis();//��ʼ������PID���˲���������ֵ
	initGimbalYaw();
	gimbalYawCtrl.wantedAbsoluteAngle=0.0f;
	while(1){
	  //robotMode=getRobotPresentMode();
		robotMode=RC_channel->mode;
		  
		//usart_printf("%d\r\n",robotMode);
		//usart_printf("%d,%d,%d,%d\r\n",RC_channel->channel_0,RC_channel->channel_1,RC_channel->channel_2,RC_channel->channel_3);
		
		//���̿���
	  analyseTotalControl();
		refreshECD();
		//yaw�����
		getControlAngles();
		refreshAngleStates(gimbal_yaw_ctrl_point); 
		limitAngles(gimbal_yaw_ctrl_point);
    calcPID();

		firstOrderFilt();
		calcWheelVelocity();
		
		calcGivenCurrent();
		if(robotMode==RobotState_e_Powerless){
			CAN_cmd_chassis(0,0,0,0);
		  CAN_cmd_gimbal(0,0,0,0);
		}
		else
		{
		CAN_cmd_chassis(driveMotor[0].giveCurrent,driveMotor[1].giveCurrent,
		driveMotor[2].giveCurrent,driveMotor[3].giveCurrent);//���Ϳ��Ƶ���
		CAN_cmd_gimbal(gimbal_yaw_ctrl_point->giveVolt,0,0,0);
		}
    fp32 power, buffer;
		get_chassis_power_and_buffer(&power, &buffer);
		//usart_printf("%f, %f\r\n", power, buffer);

		usart_printf("%f,%f,%f,%f,%d,%d\r\n",driveMotor[0].presentMotorSpeed,driveMotor[0].wantedMotorSpeed,power,buffer,gimbalYawCtrl.nowECD,get_robot_chassis_power_limit());
		osDelay(CHASSIS_CONTROL_TIME_MS);
		
	}
}
