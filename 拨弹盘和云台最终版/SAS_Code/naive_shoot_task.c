#include "cmsis_os.h"
#include "pid.h"
#include "remote_control.h"
#include "nucCommu.h"
#include "robot_total_mode.h"
#include "CAN_receive.h"
#include "main.h"
#include "detect_task.h"
#include "OLED.h"

//注意：当遥控器右拨杆指向中央的时候，才允许鼠标操控

// #define ZERO_CURRENT_SAFE

//*************任务设置
#define SHOOT_TASK_INIT_TIME 500
#define SHOOT_CTRL_TIME 5



//**************射击电机控制常量 15 0.40 0.24 0.53   1.0 0.6 1.72 1.0 18 0.45
#define SHOOT_MULTI_FREQUENCY   20      //射击频率：个/s
#define SHOOT_MULTI_TIME_GAP    (1000/SHOOT_MULTI_FREQUENCY)    //连发射击时间间隔
#define SHOOT_SPEED_LIMIT 0.420f          //摩擦轮速度。未来可以测试摩擦轮速度和射速的关系
#define SHOOT_TRIGGER_SPEED_LIMIT 2.5f   //拨弹轮开启时速度

 //**************射击电机控制常量 
 #define SHOOT_MULTI_30_FREQUENCY   6       //射击频率：个/s
 #define SHOOT_MULTI_TIME_30_GAP    (1000/SHOOT_MULTI_FREQUENCY)    //连发射击时间间隔
 #define SHOOT_SPEED_30_LIMIT 1.0f          //摩擦轮速度。未来可以测试摩擦轮速度和射速的关系
 #define SHOOT_TRIGGER_SPEED_30_LIMIT 0.75f   //拨弹轮开启时速度


#define PRESS_LONG_TIME     700    //长时间按住的定义为700ms，700个时钟周期

//**********单发圈数控制相关
#define TRIGGER_ROUNDS_FOR_A_BULLET_A 4   //M2006减速电机在减速前的轴转过的圈数，对应一发
#define TRIGGER_ROUNDS_FOR_A_BULLET_B 5     //实际测试出来36发为144圈，应为4.5圈一发。因此4和5交替来

#define ECD_FULL_ROUND 8192

//************遥控器和键鼠设置
#define SHOOT_MODE_CHANNEL 1    //遥控器左拨杆控制发射
// #define MOUSE_SHOOT  鼠标左键按下抬起——单发，按下不放——连发，右键按下：允许NUC控制发射       
#define KEY_FRIC_ON     KEY_PRESSED_OFFSET_SHIFT    //按shift开启摩擦轮

// #define TEAMER_ALLOW_SHOOT  //按下鼠标右键不放，允许nuc控制发射

//*************电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 60000.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
//老拨弹盘 20000
#define M2006_MOTOR_SPEED_PID_KP 20000.0f

#define M2006_MOTOR_SPEED_PID_KI 0.0f   //不要这个积分项，否则连续发射停止后还会转
#define M2006_MOTOR_SPEED_PID_KD 0.0f

#define MAX_MOTOR_CAN_CURRENT 1000000.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
//70000
#define MAX_MOTOR_CAN_CURRENT_M2006 70000.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT_M2006
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 2000.0f


#define SHOOT_M3508_MOTOR_RPM_TO_VECTOR (0.000415809748903494517209f/5)
#define SHOOT_M2006_MOTOR_RPM_TO_VECTOR (0.000415809748903494517209f/5)

//*************************里程碑栈法的控制参数***********//
#define MILESTONE_NEAR_THRESHHOLD 2000
#define MILESTONE_NUMBER 3

//**********************卡弹相关***********************//
#define STUCK_TIME_LIMIT 1000   //当在发射一发的模式中持续一段时间不退出后，进入卡弹解决模式
#define SOLVING_TIME_LIMIT 500  //在尝试卡弹解决模式中停留的时间。



const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
const static fp32 trigger_motor_speed_pid[3] = {M2006_MOTOR_SPEED_PID_KP, M2006_MOTOR_SPEED_PID_KI, M2006_MOTOR_SPEED_PID_KD};

//***********拨弹轮电机模式枚举类型
enum TriggerMode_e{
    TriggerMode_e_Stop,         //停止发射
    TriggerMode_e_ShootOne,     //发射一个，靠拨弹轮减速前的电机转4圈。对应弹盘轮子转一格
    TriggerMode_e_ShootMulti,   //多发，其实是等待进入发射一个的状态
    TriggerMode_e_StuckSolve,   //尝试解决卡弹
};

//**********拨弹轮ECD圈数监控器
struct milestoneStack_s {
    uint8_t head;
    uint8_t stack[MILESTONE_NUMBER+1];
};

struct TriggerControl_s{
    struct milestoneStack_s mstack; //里程碑栈
    const uint16_t * ECDPoint;           //电机ECD所在位置
    uint16_t    initECD;                //初始ECD
    uint16_t    nowECD;                 //现在的ECD
    int16_t     nowRounds;              //现在转过的圈数
};

//**********缓冲指令
struct InstructBuff_s{
    uint8_t turnOnFric  ; //为1时表示指令开启摩擦轮，为0时表示指令关闭摩擦轮
    uint8_t shootOne    ;
    uint8_t shootMulti  ;
};


//*******************全局变量********//
// 摩擦轮开启
uint8_t fricOn=0;

// 拨弹轮开启
uint8_t triggerOn=0;
uint8_t reverse = 0;

static struct InstructBuff_s insBuff;   //全局发射机构控制指令缓冲
static enum TriggerMode_e triggerMode;  //拨弹轮电机状态
// 在云台任务中控制速度

static int zeroCurrentMark;    //当离线时，直接发送零电流的时候
const static RC_ctrl_t *rc_p;
static const enum RobotState_e *robotMode;   //机器人模式
static const toSTM32_t *nuc_p;  //NUC数据位置。未来制造全自动机器人时需要

static int highspeedshoot=0;
static int16_t giveShootCurrent[2];
int16_t giveTriggerCurrent;
static pid_type_def shootMotorPIDs[2],triggerMotorPID;
static fp32 wantedVShootMotor[2],wantedVTriggerMotor;
// int16_t motor_speed_rpm;
static fp32 presentVShootMotor[2],presentVTriggerMotor;

static struct TriggerControl_s triggerCtrl;
static enum TriggerMode_e triggerMode;
static const enum RobotState_e *robotMode;
/////////////for debug///////////

static uint32_t pressTimeDebug;
// static uint8_t upDownDebug;
/////////////for sub task trigger monitor/////
// static uint8_t triggerMonitorSubOn=0;

static void initShootPIDs(void)
{
    uint8_t i;
    for(i=0;i<2;i++)
        PID_init(&shootMotorPIDs[i],PID_POSITION,motor_speed_pid,M3505_MOTOR_SPEED_PID_MAX_OUT,M3505_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&triggerMotorPID,PID_POSITION,trigger_motor_speed_pid,M2006_MOTOR_SPEED_PID_MAX_OUT,M2006_MOTOR_SPEED_PID_MAX_IOUT);
}

/**
 * @brief 将ECD差值等化为(0,8191)范围
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

static void initTriggerECDRoundsMonitor()  //初始化拨弹轮圈数监控
{
    triggerCtrl.ECDPoint=&(get_trigger_motor_measure_point()->ecd);
    triggerCtrl.initECD=*(triggerCtrl.ECDPoint);
}   

//暂未添加NUC控制发射
static void getInstructionAndBuff(void)
{
    static uint32_t pressTime=0;
    static uint8_t up=0,down=0;
    zeroCurrentMark=0;

    if(robotIsAuto())
    {
        #ifdef TEAMER_ALLOW_SHOOT
        if(nuc_p->nucSayWeShouldShootNow && rc_p->mouse.press_r)    // 按下右键，操作手允许NUC控制发射。
        #else
//        if(nuc_p->nucSayWeShouldShootNow)
        #endif
            insBuff.shootOne=1;
    }
            
    //通过按键开启摩擦轮（也许用处并不大，有了遥控器的就足够了，但还是用来确保开启了摩擦轮吧）
    if(rc_p->key.v & KEY_FRIC_ON)
		{
			if(insBuff.turnOnFric==1)
				insBuff.turnOnFric=0;
		}
    //当遥控器右拨杆指向中央的时候，允许鼠标操控
    if(switch_is_mid(rc_p->rc.s[SHOOT_MODE_CHANNEL]))
    {
        if(rc_p->mouse.press_l) //按下左键
        {
            if(pressTime<PRESS_LONG_TIME)
                pressTime+=SHOOT_CTRL_TIME;
            else    //已经按了足够长的时间了，可以不用再增加了，
            {
                insBuff.shootMulti=1;
            }
        }
        else    //左键抬起
        {
            if(pressTime<PRESS_LONG_TIME)   //拨下并很快抬起，发射一颗
                if(pressTime>0)
                {
                    insBuff.shootOne=1; 
                }
            pressTime=0;
            insBuff.shootMulti=0;
        }
    }

    if(switch_is_up(rc_p->rc.s[SHOOT_MODE_CHANNEL]))
    {
        up=1;
    }
    else if(switch_is_mid(rc_p->rc.s[SHOOT_MODE_CHANNEL]))
    {
        //通过上拨和下拨，实现摩擦轮状态的控制
        if(up==1)   // 从上到中 下拨
        {
            if(fricOn)
            {
                // fricOn=0;
                // triggerOn=0;
                //为了防止卡弹，使用缓冲指令设计。当拨弹轮停止时才能关闭摩擦轮。
                insBuff.shootMulti=0;
                insBuff.shootOne=0;
                insBuff.turnOnFric=0;   //，希望关闭摩擦轮
            }    
            else
            {
                // fricOn=1;        
                insBuff.turnOnFric=1;   //希望开启摩擦轮
            }
            up=0;
        }
        if(down==1)  // 刚刚是下方，从下到中的上拨
        {
            insBuff.shootMulti=0;   //停止连发
            if(pressTime<PRESS_LONG_TIME)   //拨下并很快抬起，发射一颗
                if(pressTime>0)
                {
                    insBuff.shootOne=1; 
                }
            pressTime=0;
            down=0;
            insBuff.shootMulti=0;
        }
    }
    else if(switch_is_down(rc_p->rc.s[SHOOT_MODE_CHANNEL]))
    {
        down=1;
        if(pressTime<PRESS_LONG_TIME)
            pressTime+=SHOOT_CTRL_TIME;
        else    //已经按了足够长的时间了，可以不用再增加了，
        {
            insBuff.shootMulti=1;
        }
    }
    pressTimeDebug=pressTime;

    

    if(toe_is_error(DBUS_TOE))
    {
        zeroCurrentMark=1;
        triggerOn=0;
        fricOn=0;
        triggerMode=TriggerMode_e_Stop;
        insBuff.turnOnFric=0;
        insBuff.shootOne=0;
        insBuff.shootMulti=0;
    }
}
//需要测试能否解析控制.

///////////////////////////
// void OLED_Shoot(void)
// {
//     OLED_printf(0,0,"nucSay:%d,t%d,presl:%d,swh:%d,Ins:fric=%d,one=%d,mul=%d,press:%d,trgMd:%d,fricOn:%d,trgOn:%d,round:%d,cur:%d",nuc_p->nucSayWeShouldShootNow,HAL_GetTick(), rc_p->mouse.press_l, rc_p->rc.s[SHOOT_MODE_CHANNEL],insBuff.turnOnFric,insBuff.shootOne,insBuff.shootMulti,pressTimeDebug,triggerMode,fricOn,triggerOn,triggerCtrl.nowRounds,giveShootCurrent[0]);
// }

//void OLED_Shoot(void)
//{
//    OLED_printf(0,0,"auto:%d,distance:%d,nucSay:%d,time:%d,shootOne:%d,toe_err:%d,phi:%f",robotIsAuto(),nuc_p->r, nuc_p->nucSayWeShouldShootNow,HAL_GetTick(),insBuff.shootOne,toe_is_error(MINI_PC_TOE),nuc_p->pitch);
//}

///////////////////////////

static void monitorTriggerECDRound(void)
{
    uint8_t j;
    // 更新ECD
    triggerCtrl.nowECD=*(triggerCtrl.ECDPoint);

    for(j=0;j<MILESTONE_NUMBER;j++)    //枚举每一个里程碑所在位置
    {
        fp32 relativeRealECD;
        relativeRealECD=ECDFormat((int16_t)triggerCtrl.nowECD-(int16_t)triggerCtrl.initECD);
        //失败原因是0的比较出现了问题
        
        if(ECDFormat(relativeRealECD-j*ECD_FULL_ROUND/MILESTONE_NUMBER)<MILESTONE_NEAR_THRESHHOLD)
                //当前位置落在相应里程碑点所在区域内
        {
            if(j!=(triggerCtrl.mstack.stack[triggerCtrl.mstack.head]))
               //不等说明到达了一个新位置，将此新位置加入栈中
            {
                triggerCtrl.mstack.head++;
                #ifdef WATCH_ARRAY_OUT
                if(c->mstack.head>=MILESTONE_NUMBER)
                {
                    itHappens();    //  让usb task输出此数组越界信息
                    c->mstack.head=MILESTONE_NUMBER-1;
                }
                    
                #endif
                triggerCtrl.mstack.stack[triggerCtrl.mstack.head]=j;
            }
        }
    }
    if(((triggerCtrl.mstack.head)-2)>=0)
    {
        if(triggerCtrl.mstack.stack[triggerCtrl.mstack.head]==triggerCtrl.mstack.stack[triggerCtrl.mstack.head-2])
            (triggerCtrl.mstack.head)-=2;
    }
    if(((triggerCtrl.mstack.head)-3)>=0)
    {
        if(triggerCtrl.mstack.stack[triggerCtrl.mstack.head]==triggerCtrl.mstack.stack[triggerCtrl.mstack.head-3])
        {//到达了一圈
            if(triggerCtrl.mstack.stack[1]==1)//正向旋转（逆时针）
                triggerCtrl.nowRounds +=1;
            else
                triggerCtrl.nowRounds -=1;
            triggerCtrl.mstack.head=0;     // 清空栈，回到初始为0的时候
        }
    }
}
static void initShootModes(void)
{
    //初始化指令
    insBuff.shootMulti=0;
    insBuff.shootOne=0;
    insBuff.turnOnFric=0;
    triggerMode=TriggerMode_e_Stop;
    fricOn=0;
    triggerOn=0;
}

// static void initWantedSpeed(void)
// {
    
// }

static void fricModeChange(void)
{
    //等待拨弹轮关闭时再关闭摩擦轮
    if((!(insBuff.turnOnFric)) || (*robotMode)==RobotState_e_Powerless)
    {
        if(triggerMode==TriggerMode_e_Stop)
        {
            fricOn=0;
            insBuff.shootMulti=0;
            insBuff.shootOne=0;
            insBuff.turnOnFric=0;
        }
    }
    else
    {
        fricOn=1;
    }        
}

static void triggerModeChange(void)
{
    static uint32_t lastShootMultiTime=0;
    static uint32_t shootOneEnterTime = 0;  //进入发射一发的状态的时刻
    // static uint32_t shootOneStayTime = 0;   // 在发射一发的状态中停留的时间
    static uint32_t solveStuckEnterTime = 0;   // 在发射一发的状态中停留的时间
    // static uint32_t solveStuckNowTime = 0;   // 在尝试解决卡弹状态中的现在时间，直接使用nowTime 代替
    
    static uint32_t nowTime=0;
    nowTime=HAL_GetTick();  //每次进入此函数都会更新
    
    if(!fricOn) //摩擦轮关闭状态，始终让拨弹轮关闭
    {
        triggerMode=TriggerMode_e_Stop;
        return;
    }
    //对每个状态枚举
    if(triggerMode==TriggerMode_e_Stop)
    {
        //机器人不处于无力状态时，接收从stop状态转变为其他状态的指令
        if((*robotMode)!=RobotState_e_Powerless)
        {
            if(insBuff.shootOne)
            {
                triggerMode=TriggerMode_e_ShootOne;
                shootOneEnterTime = nowTime;
                insBuff.shootOne=0; //清除缓冲的指令
            }
            else if (insBuff.shootMulti)
            {
                triggerMode=TriggerMode_e_ShootOne;
                insBuff.shootMulti=0;   //清除缓冲的指令
            }
                
        }
    }
    else if(triggerMode==TriggerMode_e_ShootOne)
    {
        static uint8_t nowTimeRoundThreshold=TRIGGER_ROUNDS_FOR_A_BULLET_A;
        //让拨弹轮转动，当逆时针转到4圈时（这是由于机械结构的原因），清除圈数进入stop
        if(triggerCtrl.nowRounds<=(-nowTimeRoundThreshold)||triggerCtrl.nowRounds>=nowTimeRoundThreshold)
        {
            triggerMode=TriggerMode_e_Stop;
            triggerCtrl.nowRounds=0;
            if(nowTimeRoundThreshold==TRIGGER_ROUNDS_FOR_A_BULLET_A)
                nowTimeRoundThreshold=TRIGGER_ROUNDS_FOR_A_BULLET_B;
            else
                nowTimeRoundThreshold=TRIGGER_ROUNDS_FOR_A_BULLET_A;
        }else if(nowTime - shootOneEnterTime > STUCK_TIME_LIMIT)
        {
            triggerMode=TriggerMode_e_Stop;   // 到达了卡弹的临界
            solveStuckEnterTime = nowTime;
            triggerCtrl.nowRounds = 0;//清空旋转量
        }
        //否则留在这个状态

    }
    else if(triggerMode==TriggerMode_e_ShootMulti)
    {
        if((nowTime-lastShootMultiTime)>=SHOOT_MULTI_TIME_GAP)
        {
            lastShootMultiTime=nowTime;
            triggerMode=TriggerMode_e_ShootOne;
            shootOneEnterTime = nowTime;
        }
    }
    else if(triggerMode==TriggerMode_e_StuckSolve)
    {
        if((nowTime - solveStuckEnterTime)>=SOLVING_TIME_LIMIT)
        {
            triggerMode=TriggerMode_e_Stop;
        }
    }

}

static void setSpeedByMode(void)
{
    if(fricOn)
    {
        wantedVShootMotor[0]=-SHOOT_SPEED_LIMIT;//实验测试转动方向。反过来可以实验卡弹时的退弹
        wantedVShootMotor[1]=SHOOT_SPEED_LIMIT;
			  
    }
    else
    {
        wantedVShootMotor[0]=0;
        wantedVShootMotor[1]=0;
    }
    if(triggerMode==TriggerMode_e_ShootOne || triggerMode == TriggerMode_e_StuckSolve)
        triggerOn=1;
    else 
        triggerOn=0;
    
    reverse = 0;
    if(triggerMode == TriggerMode_e_StuckSolve)
        reverse = 1;

    if(triggerOn)
        wantedVTriggerMotor=-reverse?(SHOOT_TRIGGER_SPEED_LIMIT/4):(-SHOOT_TRIGGER_SPEED_LIMIT/4);//若正在解决stuck(reverse)，则以逆向v/4的速度转动
    else
        wantedVTriggerMotor=0;
}

static void calcGiveCurrent(void)
{
    int i;
    for(i=0;i<2;i++)
        PID_calc(&shootMotorPIDs[i],presentVShootMotor[i],-wantedVShootMotor[i]);
    for(i=0;i<2;i++)
        giveShootCurrent[i]=shootMotorPIDs[i].out;
    PID_calc(&triggerMotorPID,presentVTriggerMotor,wantedVTriggerMotor);
    giveTriggerCurrent=triggerMotorPID.out;
}

static void getShootMotorSpeed(void)
{
    uint8_t i;
    for(i=0;i<2;i++)
    {
        presentVShootMotor[i]=get_shoot_motor_measure_point(i)->speed_rpm*SHOOT_M3508_MOTOR_RPM_TO_VECTOR;
    }
    presentVTriggerMotor=get_trigger_motor_measure_point()->speed_rpm*SHOOT_M2006_MOTOR_RPM_TO_VECTOR;
}

void shoot_task(void const *pvParameters)
{
    osDelay(SHOOT_TASK_INIT_TIME);
    initShootPIDs();    //初始化PID
    initTriggerECDRoundsMonitor();  //初始化拨弹轮圈数监控
    initShootModes();   //初始化发射状态
    // initWantedSpeed();  //初始化电机速度和电流 全局变量自己就是0；
    robotMode=getRobotPresentMode();
    rc_p=get_remote_control_point();    //获取遥控器数据和NUC数据指针
    // triggerMonitorSubOn=1;
    while(1)
    {
        getInstructionAndBuff();    //获取指令并缓冲
        fricModeChange();           //摩擦轮状态改变
        //若摩擦轮关闭状态，则清除单发和连发指令

        triggerModeChange();        //拨弹轮状态转变
        setSpeedByMode();

        // monitorTriggerECDRound();        //监控拨弹轮ECD圈数
        getShootMotorSpeed();
        calcGiveCurrent();                   //计算PID
                                    //发送控制电流
        #ifdef ZERO_CURRENT_SAFE
        zeroCurrentMark=1;
        #endif
        
        if(zeroCurrentMark)
        {
            CAN_cmd_shoot(0,0);
            giveTriggerCurrent=0;
        }
        else
            CAN_cmd_shoot(giveShootCurrent[0], giveShootCurrent[1]);
        
        
        osDelay(SHOOT_CTRL_TIME);
    }
}

int16_t * getTriggerCurrentP(void)
{
    return &giveTriggerCurrent;
}

//为了减小延迟使用的trigger monitor，独立于shoot任务进行
void shootTaskTrggMonitor(void const *pvParameters)
{
    osDelay(SHOOT_TASK_INIT_TIME);
    while(1)
    {
        // if(triggerMonitorSubOn)
            monitorTriggerECDRound();
        osDelay(1);
    }
}

