/**
 * encoding:GBK
 * @file naive_chassis_task.c
 * @author 
 * @brief 
 * @version 0.2
 * @date 2023-03-26
 * 
 * @copyright Copyright (c) 2022
 * 
 * ���ܶ���
 * 
 */


#include "cmsis_os.h"
#include "pid.h"
#include "bsp_laser.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "INS_task.h"
#include "main.h"
#include "remote_control.h"
#include "nucCommu.h"
#include "bsp_usart.h"
#include "nucCommu.h"
#include "naive_gimbal_task.h"
#include "robot_total_mode.h"
#include "RobotStructure.h"
#include "usb_task.h"
#include "OLED.h"
#include "user_lib.h"
#include "detect_task.h"
#include "usart.h"
#include "can.h"
#include <cmath>
#include "RobotStructure.h"
#define GIMBAL_TASK_INIT_TIME 500
#define GIMBAL_TASK_CTRL_TIME 4

// TO DO: ��������¼��ЩECD

// #define ZERO_CURRENT_SAFE

#define SAFE_AUTO_SHIFT_TIME 1500   //�Զ�ģʽʱ����װ�װ嶪ʧʱ��ʹ���ֶ����ƣ�
                //������ղ��������ƫ�����������´��ֶ������Զ�ģʽģʽʱ�������ֿ�����Ϊ��������Ӧ
                //����ƫ�����ƻ��������ư�ȫʱ�䣬�����½���ʱ����һ��ʱ�����Ӳ���������ƫ������
                //���ʱ��ΪSAFE_AUTO_SHIFT_TIME ��λms
//���������
#define CHANNEL_MAX 660
#define K_CTRL_EXP_X 1.5
#define K_CTRL_EXP_Y 2.5
#define K_CTRL_EXP_MOUSE_X 1
#define K_CTRL_EXP_MOUSE_Y 1
#define K_CTRL_PROP_X 20.0
#define K_CTRL_PROP_Y 20.0
#define K_CTRL_MOUSE_X 40
#define K_CTRL_MOUSE_Y 5


//****************************�����˲��Ի��ֵ*************//
#define PITCH_TEST_INIT_ANGLE_MIDDLE_ECD 4628
#define PITCH_TEST_MAX_ANGLE_ECD 4228
#define PITCH_TEST_MIN_ANGLE_ECD 3828
// #define PITCH_TEST_ANGLE_ECD_LENGTH 1039

#define PITCH_TEST_RAD_MIN (-0.39f)
#define PITCH_TEST_RAD_MAX  (0.20f)

#define YAW_TEST_INIT_ANGLE_MIDDLE_ECD 3557
#define YAW_TEST_MAX_ANGLE_ECD 5616
#define YAW_TEST_MIN_ANGLE_ECD 1534
// #define YAW_TEST_ANGLE_ECD_LENGTH 1039

#define NO_SLIP_RING_MAX_ROUNDS 0

// // ������ٶ� rad/s �������ѹ int16_t ��PIDϵ��
// #define PITCH_SPD_KP 300000.0f
// #define PITCH_SPD_KI 30.0f
// #define PITCH_SPD_KD 0.0f

// #define PITCH_VOLT_MAX_OUT  30000.0f
// #define PITCH_VOLT_MAX_IOUT 5000.0f

// // #define YAW_SPD_KP 90000.0f
// // // #define YAW_SPD_KI 300000.0f
// // // #define YAW_SPD_KD 90000.0f

// // #define YAW_SPD_KI 8000.0f
// // #define YAW_SPD_KD 000.0f

// #define YAW_SPD_KP 30000.0f
// // #define YAW_SPD_KI 300000.0f
// // #define YAW_SPD_KD 90000.0f

// #define YAW_SPD_KI 00.0f
// #define YAW_SPD_KD 000.0f

// #define YAW_VOLT_MAX_OUT  30000.0f
// #define YAW_VOLT_MAX_IOUT 5000.0f


// //����Ƕ� rad ��������ٶ�rad/s ��PIDϵ��
// #define AGL_KP 0.07f
// #define AGL_KI 0.0f
// #define AGL_KD 0.0f

// #define AGL_SPD_MAX_OUT (1.0f)
// #define AGL_SPD_MAX_IOUT (0.7f)

/*****
 * Try again at 2023/3/16 21:00
 */

// ������ٶ� rad/s �������ѹ int16_t ��PIDϵ��
#define PITCH_SPD_KP 10000000.0f
// #define PITCH_SPD_KP 10000000.0f

#define PITCH_SPD_KI 200.0f
#define PITCH_SPD_KD 0.0f
                            
#define PITCH_VOLT_MAX_OUT  30000.0f    // prev ent overflow of control control volt
                                        // because the control volt is -30000~30000
#define PITCH_VOLT_MAX_IOUT 1000.0f

//����Ƕ� rad ��������ٶ�rad/s ��PIDϵ��
#define PITCH_AGL_KP 0.009f
#define PITCH_AGL_KI 0.0001f
#define PITCH_AGL_KD 0.00f

#define PITCH_AGL_SPD_MAX_OUT (2.0f)
#define PITCH_AGL_SPD_MAX_IOUT (1.0f)


#define YAW_SPD_KP 20000.0f
#define YAW_SPD_KI 0.0f
#define YAW_SPD_KD 400.0f

#define YAW_VOLT_MAX_OUT  29998.0f
#define YAW_VOLT_MAX_IOUT 1000.0f

// #define YAW_AGL_KP 0.07f
#define YAW_AGL_KP 0.007f

#define YAW_AGL_KI 0.0f
#define YAW_AGL_KD 0.0f

#define YAW_AGL_SPD_MAX_OUT (50.0f)
#define YAW_AGL_SPD_MAX_IOUT (11.7f)







#define ECD_FULL_ROUND 8192

#define GIMBAL_FILTER_ON

//*****************************ң����ͨ��ѡ��**************//
#define GIMBAL_YAW_OR_CHASSIS_W_CHANNEL 0     //�ұߵ�����ҡ��
#define GIMBAL_PITCH_CHANNEL 1  //�ұߵ�ǰ��ҡ��
#define RC_DEADLINE 10  //�������ֵ��ҡ�˶���һ��Ϊ0
#define TO_CHASSIS_COEF 1

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


//****************************����������ϵ��**************//
// ʹ��ң�����ͼ��̿�����̨����������Ϊ�ٶ�
// ң�����ͼ��̿��ƶ����ٶȣ����������ٶ�
// NUC����ʱ��Ŀ�������Ϊ�Ƕ�
#define YAW_RC_SEN      -0.000004f
#define PITCH_RC_SEN    -0.000005f //0.005
#define YAW_MOUSE_SEN   -0.000005f
#define PITCH_MOUSE_SEN -0.000005f

//����������

// ң�������ֵ660
// 0.5��(500ms)�Ƕ����� 1.7rad
//5.1515151515151515151515151515152e-6

//������ֵ��32767
//0.5��(500ms)�Ƕ����� 1.7rad
// 1.0376293221839045381023590807825e-7


//*************************��̱�ջ���Ŀ��Ʋ���***********//
#define MILESTONE_NEAR_THRESHHOLD 200
#define MILESTONE_NUMBER 3

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

//һ���˲����Ĳ���,ר�Ÿ���������ֵԽ��������Խ˿��
// #define PITCH_SPD_FILTER_NUM 15.0f
// #define YAW_SPD_FILTER_NUM 15.0f

#define PITCH_SPD_FILTER_NUM 0.001f
#define YAW_SPD_FILTER_NUM 0.001f


 #define LimitMax(input, max)   \
     {                          \
         if (input > max)       \
         {                      \
             input = max;       \
         }                      \
         else if (input < -max) \
         {                      \
             input = -max;      \
         }                      \
     }


// Ϊ�˵��ԣ���ҪһЩ���ݽṹ,��usb�������
// 
// ��̨��һ��ʱ���ڿ����޷�����ָ��λ�ã��ͷ�������
struct milestoneStack_s {
    uint8_t head;
    uint8_t stack[MILESTONE_NUMBER+1];
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

static struct gimbalMotorCtrl_s gimbalPitchCtrl,gimbalYawCtrl;
static struct gimbalMotorCtrl_s *gimbalCtrl[2]={&gimbalPitchCtrl,&gimbalYawCtrl};
static fp32 yawSpdPIDco[3]={YAW_SPD_KP,YAW_SPD_KI,YAW_SPD_KD};
static fp32 pitchSpdPIDco[3]={PITCH_SPD_KP,PITCH_SPD_KI,PITCH_SPD_KD};
static fp32 pitchAglPIDco[3]={PITCH_AGL_KP,PITCH_AGL_KI,PITCH_AGL_KD};
static fp32 yawAglPIDco[3]={YAW_AGL_KP,YAW_AGL_KI,YAW_AGL_KD};
static const enum RobotState_e *robotMode;
static enum RobotState_e lastMode;

//ң��������,�����ۼ�
static int16_t rc_mouse_x;
static int16_t rc_mouse_y;
static int16_t press_x=0;

//����ָ��
static const RC_ctrl_t *rc_p;   //ң����λ��ָ�룬��Ҫ��ʼ��
static const toSTM32_t *nuc_p;  //NUC����λ��

// ���������ͨ�ŵ�����.ֻ����badyawģʽ������ת���ٶ�
static struct GimbalToChassis_s toChassis={0,0};
static uint8_t GimbalAngleMsg[34];
static int is_auto=0;
static void initGimbalCtrls(void)
{
    uint8_t i;
    //��ʼ�����ԽǶ�λ��
    gimbalYawCtrl.anglePoint=get_INS_angle_point();         //��һ���Ƕ�
    gimbalPitchCtrl.anglePoint=(&(get_INS_angle_point()[2])); //�������Ƕ�
    // ��ʼ��ECD
    gimbalYawCtrl.ECDPoint=&(get_yaw_gimbal_motor_measure_point()->ecd);//��̨������Ҳ����ν
    gimbalPitchCtrl.ECDPoint=&(get_pitch_gimbal_motor_measure_point()->ecd);

    gimbalPitchCtrl.maxECD=PITCH_TEST_MAX_ANGLE_ECD;
    gimbalPitchCtrl.minECD=PITCH_TEST_MIN_ANGLE_ECD;
    gimbalPitchCtrl.middleECD=PITCH_TEST_INIT_ANGLE_MIDDLE_ECD;

    gimbalYawCtrl.maxECD=YAW_TEST_MAX_ANGLE_ECD;
    gimbalYawCtrl.minECD=YAW_TEST_MIN_ANGLE_ECD;
    gimbalYawCtrl.middleECD=YAW_TEST_INIT_ANGLE_MIDDLE_ECD;
    //��ʼ��ת���ĽǶ�
    gimbalPitchCtrl.maxRounds=0;
    #ifdef HAVE_SLIP_RING
    gimbalYawCtrl.maxRounds=30000;
    #else
    gimbalYawCtrl.maxRounds=NO_SLIP_RING_MAX_ROUNDS;
    #endif

    // ��ʼ��PID����
    PID_init(&(gimbalPitchCtrl.spd_pid),PID_POSITION,pitchSpdPIDco,PITCH_VOLT_MAX_OUT,PITCH_VOLT_MAX_IOUT);
    PID_init(&(gimbalYawCtrl.spd_pid),PID_POSITION,yawSpdPIDco,YAW_VOLT_MAX_OUT,YAW_VOLT_MAX_IOUT);
    
    //�˲���ʼ��
    const static fp32 gimbal_pitch_order_filter[1] = {PITCH_SPD_FILTER_NUM};
    const static fp32 gimbal_yaw_order_filter[1] = {YAW_SPD_FILTER_NUM};

    first_order_filter_init(&(gimbalPitchCtrl.spd_filter), GIMBAL_TASK_CTRL_TIME, gimbal_pitch_order_filter);
    first_order_filter_init(&(gimbalYawCtrl.spd_filter), GIMBAL_TASK_CTRL_TIME, gimbal_yaw_order_filter);
    
    //��ʼ����ǰ�ǶȺ�Ŀ��Ƕȡ�pid��������̱�ջ������
    for(i=0;i<2;i++)
    {
        (gimbalCtrl[i])->zeroVoltMark=0;
        (gimbalCtrl[i])->nowAbsoluteAngle=*((gimbalCtrl[i])->anglePoint);
        (gimbalCtrl[i])->nowTime=HAL_GetTick();
        gimbalCtrl[i]->wantedAbsoluteAngle=(gimbalCtrl[i])->nowAbsoluteAngle;
        (gimbalCtrl[i]->mstack).head=0;
        (gimbalCtrl[i]->mstack).stack[0]=0;
    }
    PID_init(&(gimbalCtrl[0]->agl_pid),PID_POSITION,pitchAglPIDco,PITCH_AGL_SPD_MAX_OUT,PITCH_AGL_SPD_MAX_IOUT);
    PID_init(&(gimbalCtrl[1]->agl_pid),PID_POSITION,yawAglPIDco,YAW_AGL_SPD_MAX_OUT,YAW_AGL_SPD_MAX_IOUT);
    
    // ��ʼ��toChassis������
    toChassis.effective=0;
    toChassis.w=0;
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

/**
 * @brief ����תȦ��PID���ڲ�error��ѭ��ת��Ϊ(-PI,PI)��Χ
 * 
 * @param pid 
 * @param ref ����ֵ
 * @param set �趨ֵ
 * @return fp32 
 */
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
			if(toe_is_error(DBUS_TOE))
				pid->Iout=0.0f;
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
			  if(pid->error[0]<100){
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
int adjust_channel(int16_t channel, float ctrl_prop, float ctrl_exp)
{
	float a = powf((fabs(channel)/CHANNEL_MAX), ctrl_exp) * CHANNEL_MAX * (channel > 0 ? 1.f : -1.f) * ctrl_prop;
	//usart_printf("%f, %f\r\n", (float)channel, a);
	return a;
	
}
void getControlAngles(void)
{

    fp32 selfTargetOffsetPitch=0,selfTargetOffsetYaw=0;
    int16_t yaw_channel = 0, pitch_channel = 0;
    rc_deadband_limit(rc_p->rc.ch[GIMBAL_YAW_OR_CHASSIS_W_CHANNEL], yaw_channel, RC_DEADLINE);
    rc_deadband_limit(rc_p->rc.ch[GIMBAL_PITCH_CHANNEL], pitch_channel, RC_DEADLINE);
    //�����������ң����������̨rad�ٶȵĿ���

    //δ�������������Ժ������ƽ�����ӣ���������һ���Ƕȣ�����������ƫ������ƫ�Ƶ�����
    
    //������bad yaw carת������ģʽ����̨��ͻȻת��������Ϊ��̨û�е�����ϣ����λ�����ǣ�����һ��ת�����̡�
    // ����һ��ģʽ����һ��ģʽ��ͬʱ����ϣ���ĽǶ�����Ϊ��ǰ�Ƕȡ�
    
	  

//����ģʽѡ���־
//�Ӿ����԰汾
     
		//usart_printf("%f,%f\r\n",gimbalPitchCtrl.wantedAbsoluteAngle,gimbalPitchCtrl.nowAbsoluteAngle);
    if(*robotMode==RobotState_e_Powerless)
    {
        toChassis.effective=0;
        toChassis.w=0;
        gimbalYawCtrl.zeroVoltMark=1;
        gimbalPitchCtrl.zeroVoltMark=1;
    }    
    else 
    {
				if(lastMode==RobotState_e_Powerless)
				{
						//usart_printf("OK\r\n");
						gimbalPitchCtrl.wantedAbsoluteAngle = gimbalPitchCtrl.nowAbsoluteAngle;
						gimbalYawCtrl.wantedAbsoluteAngle = gimbalYawCtrl.nowAbsoluteAngle;
						gimbalYawCtrl.zeroVoltMark=0;
						gimbalPitchCtrl.zeroVoltMark=0;
				}
        
			  
				//pitch��Ŀ��ֵ����
				gimbalPitchCtrl.wantedAbsoluteAngle+=20*pitch_channel*PITCH_RC_SEN-20*rc_p->mouse.y*PITCH_MOUSE_SEN;
//				//gimbalPitchCtrl.wantedAbsoluteAngle += adjust_channel(pitch_channel, K_CTRL_PROP_Y, K_CTRL_EXP_Y) * PITCH_RC_SEN - adjust_channel(rc_p->mouse.y, K_CTRL_MOUSE_Y, K_CTRL_EXP_Y) * PITCH_MOUSE_SEN ;
//				if(gimbalPitchCtrl.wantedAbsoluteAngle>=0.2)
//					gimbalPitchCtrl.wantedAbsoluteAngle=0.2;
//				if(gimbalPitchCtrl.wantedAbsoluteAngle<=-0.4)
//					gimbalPitchCtrl.wantedAbsoluteAngle=-0.4;
				
				//yaw��Ŀ��ֵ����
        gimbalYawCtrl.wantedAbsoluteAngle+=20*yaw_channel*YAW_RC_SEN+50*rc_p->mouse.x * YAW_MOUSE_SEN;
				//gimbalYawCtrl.wantedAbsoluteAngle +=  adjust_channel(yaw_channel, K_CTRL_PROP_X, K_CTRL_EXP_X) * YAW_RC_SEN + adjust_channel(rc_p->mouse.x,K_CTRL_MOUSE_X,K_CTRL_EXP_MOUSE_X)*YAW_MOUSE_SEN;
				//usart_printf("%d\r\n",rc_p->mouse.x);
				if(gimbalYawCtrl.wantedAbsoluteAngle>=3.086)
					gimbalYawCtrl.wantedAbsoluteAngle-=2*PI;
				if(gimbalYawCtrl.wantedAbsoluteAngle<=-3.196)
					gimbalYawCtrl.wantedAbsoluteAngle+=2*PI;
				
			if(rc_p->mouse.press_r||rc_p->rc.ch[4]>500)
			{
				is_auto=1;
				gimbalPitchCtrl.wantedAbsoluteAngle=gimbalPitchCtrl.nowAbsoluteAngle;
				gimbalYawCtrl.wantedAbsoluteAngle=gimbalYawCtrl.nowAbsoluteAngle;
			  gimbalPitchCtrl.wantedAbsoluteAngle +=nuc_p->pitch.data;
			  gimbalYawCtrl.wantedAbsoluteAngle -=nuc_p->yaw.data;

			}
			else
			{
				is_auto=0;
			}
    }
    
    lastMode=*robotMode;
    
}

void monitorRounds(struct gimbalMotorCtrl_s * c)
{
    uint8_t j;
    for(j=0;j<MILESTONE_NUMBER;j++)    //ö��ÿһ����̱�����λ��
    {
        fp32 relativeRealECD;
        relativeRealECD=ECDFormat((int16_t)c->nowECD-(int16_t)c->middleECD);
        //ʧ��ԭ����0�ıȽϳ���������
        
        if(ECDFormat(relativeRealECD-j*ECD_FULL_ROUND/MILESTONE_NUMBER)<MILESTONE_NEAR_THRESHHOLD)
                //��ǰλ��������Ӧ��̱�������������
        {
            if(j!=(c->mstack.stack[c->mstack.head]))
               //����˵��������һ����λ�ã�������λ�ü���ջ��
            {
                c->mstack.head++;
                #ifdef WATCH_ARRAY_OUT
                if(c->mstack.head>=MILESTONE_NUMBER)
                {
                    itHappens();    //  ��usb task���������Խ����Ϣ
                    c->mstack.head=MILESTONE_NUMBER-1;
                }
                    
                #endif
                c->mstack.stack[c->mstack.head]=j;
            }
        }
    }
    if(((c->mstack.head)-2)>=0)
    {
        if(c->mstack.stack[c->mstack.head]==c->mstack.stack[c->mstack.head-2])
            (c->mstack.head)-=2;
    }
    if(((c->mstack.head)-3)>=0)
    {
        if(c->mstack.stack[c->mstack.head]==c->mstack.stack[c->mstack.head-3])
        {//������һȦ
            if(c->mstack.stack[1]==1)//������ת����ʱ�룩
                c->nowRounds +=1;
            else
                c->nowRounds -=1;
            c->mstack.head=0;     // ���ջ���ص���ʼΪ0��ʱ��
        }
    }
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

/**
 * @brief ��ECD����pitch�Ƕȣ�һ��Ҫ����ȷ��pitch����ECD����0
 * 
 */
void limitAnglesSecond(void)
{
     
    if(gimbalPitchCtrl.wantedAbsoluteAngle>PITCH_TEST_RAD_MAX)
        gimbalPitchCtrl.wantedAbsoluteAngle=PITCH_TEST_RAD_MAX;
    if(gimbalPitchCtrl.wantedAbsoluteAngle<PITCH_TEST_RAD_MIN)
        gimbalPitchCtrl.wantedAbsoluteAngle=PITCH_TEST_RAD_MIN;
    
}

void calcPID(void)
{
    uint8_t i;
    struct gimbalMotorCtrl_s * c;
    for(i=0;i<2;i++)
    {
        c=gimbalCtrl[i];
        gimbal_PID_calc(&(c->agl_pid),c->nowAbsoluteAngle,c->wantedAbsoluteAngle);  // �غ���ת�����PID������
        // �����������ת�ٶȡ�

        //���ٶȽ���һ���˲�
        #ifdef GIMBAL_FILTER_ON
        first_order_filter_cali(&(c->spd_filter),(c->agl_pid).out);

        #endif
        PID_calc(&(c->spd_pid),c->radSpeed,(c->spd_filter).out);   // ��ͨ���ٶȿ��ƻ�
        c->giveVolt=c->spd_pid.out;     //����ѹ
				//usart_printf("%d\r\n",c->giveVolt);
        if(c->zeroVoltMark)
        {
            c->giveVolt=0;
        }
    }
    // ���������bad yawʱ��������ٶȸ����̵��
    if(*robotMode==RobotState_e_BadYawCar)
        toChassis.w=gimbalYawCtrl.agl_pid.out;
}

int16_t * getTriggerCurrentP(void); //����һ���ļ���ȡ�����ֵ���ĵ���

void gimbal_task(void const *pvParameters)
{
    uint8_t i;
    osDelay(GIMBAL_TASK_INIT_TIME);
    initGimbalCtrls();
    robotMode=getRobotPresentMode();
    rc_p=get_remote_control_point();    //��ȡң�������ݺ�NUC����ָ��
    nuc_p=get_nuc_control_point();
    osDelay(GIMBAL_TASK_INIT_TIME);
    PID_clear(&(gimbalYawCtrl.spd_pid));
    PID_clear(&(gimbalYawCtrl.agl_pid));
    PID_clear(&(gimbalPitchCtrl.spd_pid));
    PID_clear(&(gimbalPitchCtrl.agl_pid));
    int16_t *triggerCurrentP=getTriggerCurrentP();//��ȡshoot_taskԴ�ļ�������M2006�ĵ�����
		lastMode=RobotState_e_Powerless;  //��һ�γ�ʼ��Ϊ����
		gimbalYawCtrl.zeroVoltMark=0;
		gimbalPitchCtrl.zeroVoltMark=0;
		gimbalYawCtrl.wantedAbsoluteAngle = gimbalYawCtrl.nowAbsoluteAngle;
		gimbalPitchCtrl.wantedAbsoluteAngle = gimbalPitchCtrl.nowAbsoluteAngle;
    while(1)
    {
			  laser_on();
        for(i=0;i<2;i++)
            refreshAngleStates(gimbalCtrl[i]);   // �Ӵ�������õ�ǰ�Ƕ����� //test done
        getControlAngles();                  // ��ң�������Ŀ��Ƕ�     //test half done
        for(i=0;i<2;i++)
        {
            monitorRounds(gimbalCtrl[i]);        // ���ת����Ȧ��           //test
        }
				limitAnglesSecond();
        calcPID();              // ���ǿ��ƽǶȣ�˫����������ʱ��Ҫ
        CAN_cmd_gimbal(gimbalYawCtrl.giveVolt,-gimbalPitchCtrl.giveVolt,*triggerCurrentP,0);       
				CAN1_send_yaw();
				CAN1_send_channel();
			  //����λ����ͨ�����԰棬�����ƶ������λ��
//				usart_printf("%d\n",*gimbalPitchCtrl.ECDPoint);
//				usart_printf("%f,%f,%f,%f,%f\r\n",gimbalYawCtrl.wantedAbsoluteAngle,gimbalYawCtrl.nowAbsoluteAngle,
//				gimbalPitchCtrl.wantedAbsoluteAngle,gimbalPitchCtrl.nowAbsoluteAngle,get_INS_angle_point()[1]);
//				Encode(GimbalAngleMsg,gimbalYawCtrl.nowAbsoluteAngle,gimbalPitchCtrl.nowAbsoluteAngle,get_INS_angle_point()[1],3,get_refree_point()->robot_color,1,is_auto);
				Encode(GimbalAngleMsg,gimbalYawCtrl.nowAbsoluteAngle,gimbalPitchCtrl.nowAbsoluteAngle,get_INS_angle_point()[1],0,3,1,is_auto);
				HAL_UART_Transmit(&huart1, GimbalAngleMsg, 34, 100);
				usart1_tx_dma_enable(GimbalAngleMsg, 34);
//				usart_printf("%d\n",is_auto);
				//usb_printf("���\r\n");
				osDelay(GIMBAL_TASK_CTRL_TIME);
				

    }
}
const struct GimbalToChassis_s * getBadYawWantedSpd(void)
{
    return &toChassis;
}

const struct gimbalMotorCtrl_s* get_wantedyaw_point()
{
	return &gimbalYawCtrl;
}

const int16_t * getGimbalNowRoundsPoint(void)
{
    return &(gimbalYawCtrl.nowRounds);
}
//to do :
// ���ٶ���������
// 

void OLED_gimbal(void)
{
    // OLED_printf(4,0,"WantP:%.2f,wantY%.2f",gimbalPitchCtrl.wantedAbsoluteAngle,gimbalYawCtrl.wantedAbsoluteAngle);
    OLED_printf(0,0,"pitch ECD:%d,pitch rad:%f",gimbalCtrl[0]->nowECD,gimbalCtrl[0]->nowAbsoluteAngle);
}


