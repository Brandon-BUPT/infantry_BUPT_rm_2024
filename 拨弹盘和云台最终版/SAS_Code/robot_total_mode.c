/**
 * @file robot_total_mode.c
 * @author 陈卓勋 电控组 UCAS-SAS 中国科学院大学机器人队
 * @brief 根据遥控器宏观判断步兵机器人的总状态（包括是否自动），让云台、底盘、发射控制任务根据总状态判断分状态，保证一致性
 *          另外，在.c文件中定义变量，记录robotAuto状态。
 * @version 0.1
 * @date 2022-01-28 
 * 
 * @version 0.2
 * @date 2022-02-05 修改为四个模式：无力、普通车、小陀螺车、坏yaw走位车
 *                                 掉线    C       V        B
 *                                          上      中      下
 * 当键盘开始控制后，遥控器拨杆就暂时失去控制了。当拨杆状态发生改变，那么清除键盘控制标记，遥控器重新获得更改状态的权限
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
//***************debug设置*******************************//
// #define MODE_SWITCH_VERSION_1       //第一版本：通过遥控器拨杆变换状态2022年1月28日 只有sticky yaw

//OLED调试相关
// #define OLED_DEBUG_TOTAL_MODE_ON
#ifdef OLED_DEBUG_TOTAL_MODE_ON
#define OLED_REFRESH_TIME 300
#endif

//***************模式设置相关宏定义***********************//
#define RC_ROBOT_MODE_CHANNEL 0     //右边的拨杆控制模式选择。第一版本里，通过位置的停放实现模式改变，之后可以用位置总变化来控制

//***************任务相关宏定义***************************//
#define ROBOT_MODE_TASK_TIME 10    //10ms执行一次，因为涉及鼠标和键盘状态的读取，采样需要快。

//***************调试所用变量**************************//
#ifdef OLED_DEBUG_TOTAL_MODE_ON
static int time_passed_debug=0;
#endif

//***************全局变量*******************************//
static int robotAuto=0;     //用于记
static const RC_ctrl_t *rc_p;   //遥控器位置指针，需要初始化
static const toSTM32_t *nuc_p;  //NUC数据位置 
static enum RobotState_e robotState=RobotState_e_Powerless;  //机器人初始状态
static uint8_t keyBoardAndMouseHasChanged=0;        //记录是否有键盘鼠标操作
                                                //它的功能是避免遥控器的指令把键盘鼠标的指令覆盖了
                                                //因此只需要关注与遥控器控制量冲突的状态控制的几个键

/* 当键盘有任何按键开始按下后，就转换到键盘和鼠标控制了，遥控器控制失效。当遥控器右上拨杆发生
状态改变或者遥控器离线时，清除键鼠记录，重新按遥控器控制为准，
*/ 

void robot_total_mode_task(void const *pvParameters)
{
    //初始化数据指针
    rc_p=get_remote_control_point();
    nuc_p=get_nuc_control_point();
    int8_t lastMode,nowMode;
    nowMode=rc_p->rc.s[RC_ROBOT_MODE_CHANNEL];
    robotAuto=0;
    while (1)
    {
        //*****************************OLED 显示调试******************//
        #ifdef OLED_DEBUG_TOTAL_MODE_ON
        time_passed_debug+=ROBOT_MODE_TASK_TIME;
        if(time_passed_debug>OLED_REFRESH_TIME)
        {
            time_passed_debug=0;
            //*********输出和更新******************
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

        //*****************************第一版本：通过遥控器拨杆变换状态**//
        // NUC掉线时，强制变为手动模式
        // 遥控器掉线时，强制回到PowerLess状态
        // 当键盘有任何按键开始按下后，就转换到键盘和鼠标控制了
        // 键盘Z键按下，开启自动瞄准模式（拼音zi“自”），
        // 键盘X键按下，关闭自动瞄准模式（叉叉把自动瞄准叉去了）

        lastMode=nowMode;
        nowMode=rc_p->rc.s[RC_ROBOT_MODE_CHANNEL];
        if(lastMode!=nowMode)
            keyBoardAndMouseHasChanged=0;
        if(!keyBoardAndMouseHasChanged) //键鼠状态清除
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
        //探测键盘鼠标状态
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
        if(rc_p->key.v & KEY_PRESSED_OFFSET_C)  //c开启普通车
        {
            robotState=RobotState_e_CommonCar;
            keyBoardAndMouseHasChanged=1;
        }
        if(rc_p->key.v & KEY_PRESSED_OFFSET_V)  //v开启小陀螺
        {
            robotState=RobotState_e_Spinner;
            keyBoardAndMouseHasChanged=1;
        }
        if(rc_p->key.v & KEY_PRESSED_OFFSET_B)  //b开启卡yaw坏车
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


        //*********************掉线检测*******************************//
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
