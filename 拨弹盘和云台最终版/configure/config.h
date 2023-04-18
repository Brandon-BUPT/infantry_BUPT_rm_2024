#ifndef CONFIG_H
#define CONFIG_H

// 关于校准任务的修改
#define NO_CHASSIS_ID_RESET
// 比赛地点 和校准姿态解算算法有关
#define SHENZHEN_LATITUDE 22.0f
#define BEIJING_LATITUDE 39.5f

#define COMPETITION_LATITUDE BEIJING_LATITUDE
//**********************************************//
//can总线上的电极控制。需要更改ID、控制方法




//**********************************************//

//先实现改装的官方代码在机器人上运行
//而后继续移植自己的算法实现准小陀螺
//而后实现自瞄和小陀螺

//改装发射装置
#define EIGHT_BULLET_PER_ROUND
#define NO_CALI_GIMBAL

//更改遥控器usart1输出debug
#define RC_NO_USART1_OUT
#endif // !CONFIG_H
