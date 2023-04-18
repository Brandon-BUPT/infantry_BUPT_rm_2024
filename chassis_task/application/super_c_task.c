#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "CAN_receive.h"
#include "super_c_task.h"

#include "referee.h"


void super_c_task(void const * arguement)
{
	  fp32 power, buffer;
		get_chassis_power_and_buffer(&power, &buffer);
	while(1)
	{		
		CAN2_send_super_c_buffer((int16_t)60);
		CAN2_send_super_c_control(get_robot_chassis_power_limit(),100,100);
		osDelay(10);
	}
}