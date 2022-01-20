/****************************************************************************
 *  RoboMentors www.robomentors.com.
 *	wechat:superzz8080
 *	基于RoboMaster二次开发
 ***************************************************************************/


//实验：执行器 电机正转、反转、加速、减速的控制
#include "rm_hal_lib.h"
#include "cmsis_os.h"

uint8_t usart3_recv[4];
uint8_t a=0,b=0,c=0,d=0;
uint8_t can1_send_data[8];
int16_t current=0;


void Usart3_callback(void){
	a=usart3_recv[0];
	b=usart3_recv[1];
	c=usart3_recv[2];
	d=usart3_recv[3];
}


void study_task(const void*argu){
	can_device_init();
	uart_init(USER_UART3,9600,WORD_LEN_8B,STOP_BITS_1,PARITY_NONE);
	uart_recv_callback_register(USER_UART3,Usart3_callback);
	uart_receive_start(USER_UART3,usart3_recv,4);//从uart3接收，把接收到的数据放到usart3_recv

	while(1){
		if(a=='1'){
			write_led_io(LED_IO1,LED_ON);
			osDelay(100);
			write_led_io(LED_IO1,LED_OFF);
		}
		//加速过程
		for(current=0;current<30000;current+=100){
			can1_send_data[0]=current>>8;
			can1_send_data[1]=current;
			write_can(USER_CAN1,0x1ff,can1_send_data);
			osDelay(10);
		}
		
		//反向加速过程
		for(current=10000;current>-30000;current-=100){
			can1_send_data[0]=current>>8;
			can1_send_data[1]=current;
			write_can(USER_CAN1,0x1ff,can1_send_data);
			osDelay(10);
		}
		
		//反向减速过程
		for(current=-30000;current<0;current+=100){
			can1_send_data[0]=current>>8;
			can1_send_data[1]=current;
			write_can(USER_CAN1,0x1ff,can1_send_data);
			osDelay(10);
		}
	}
}


////实验：控制技术 PID运算方法
//#include "rm_hal_lib.h"
//#include "cmsis_os.h"

//static void abs_limit_test(float *a,float ABS_MAX);

//typedef struct{
//	float p;
//	float i;
//	float d;

//	float set;
//	float get;
//	float err[2];

//	float pout;
//	float iout;
//	float dout;

//	float out;  
//	uint32_t max_output;
//	uint32_t integral_limit;   
// 
//}pid_t;

//enum{
//	LAST	= 0,
//	NOW 	= 0,       
//};

//void pid_init_test(pid_t *pid,uint32_t max_out,uint32_t intergral_limit,float kp,float ki,float kd);

//float pid_calc_test(pid_t *pid,float get,float set);

//uint8_t can1_send_data[8];

//uint8_t can1_received[8];

//int16_t speed,current;

//pid_t pid_test  = { 0 };

//void can1_recv_callback_test(uint32_t recv_id,uint8_t data[]){
//	if(recv_id==0x201){
//		can1_received[0]=data[0];
//		can1_received[1]=data[1];
//		can1_received[2]=data[2];
//		can1_received[3]=data[3];
//		can1_received[4]=data[4];
//		can1_received[5]=data[5];
//		can1_received[6]=data[6];
//		can1_received[7]=data[7];

//		speed=(int16_t)(data[2]>>8|data[3]);
//	}
//}

//void pid_init_test(pid_t *pid,uint32_t max_out,uint32_t intergral_limit,float kp,float ki,float kd){
//	pid->integral_limit =intergral_limit;
//	pid->max_output =max_out;

//	pid->p = kp;
//	pid->i = ki;
//	pid->d = kd;
//}

//static void abs_limit_test(float *a,float ABS_MAX){
//	if(*a >ABS_MAX)
//		*a=ABS_MAX;
//	if(*a <-ABS_MAX)
//		*a=-ABS_MAX;
//}

//float pid_calc_test(pid_t *pid,float get,float set){
//	pid->get=get;
//	pid->set=set;
//	pid->err[NOW]=set - get;

//	pid->pout =pid->p * pid->err[NOW];
//	pid->pout =pid->i * pid->err[NOW];
//	pid->pout =pid->d * pid->err[NOW] - pid->err[LAST];

//	abs_limit_test(&(pid->iout),pid->integral_limit);
//	pid->out =pid->pout + pid->iout + pid->dout;
//	abs_limit_test(&(pid->out), pid->max_output);

//	pid->err[LAST] = pid->err[NOW];

//	return pid->out;
//}

//void study_task(const void*argu){
//	
//	osDelay(2000);
//	
//	pid_init_test(&pid_test,3000,0,8.0f,0.0f,0.0f);
//	
//	can_device_init();
//	
//	can_recv_callback_register(USER_CAN1,can1_recv_callback_test);
//	
//	can_receive_start();
//	
//	while(1){
//		current=pid_calc_test(&pid_test,speed,500);
//		can1_send_data[0]=current>>8;
//		can1_send_data[1]=current;
//		write_can(USER_CAN1,0x200,can1_send_data);
//		osDelay(5);
//	}
//}


////实验：使用遥控器控制灯效
//#include "rm_hal_lib.h"
//#include "cmsis_os.h"
//#include "uart_device.h"

//void study_task(const void*argu){
//	
//	while(1){
//		
//		//当遥控器右上角拨档开关推到上
//		if(rc.sw2==1){
//			write_led_io(LED_IO1,LED_ON);
//			write_led_io(LED_IO2,LED_OFF);
//			write_led_io(LED_IO3,LED_OFF);
//		}
//		
//		//当遥控器右上角的拨档开关拨到中
//		if(rc.sw2==3){
//			write_led_io(LED_IO1,LED_OFF);
//			write_led_io(LED_IO2,LED_ON);
//			write_led_io(LED_IO3,LED_OFF);
//		}
//		
//		//当遥控器右上角的拨档开关拨到下
//		if(rc.sw2==2){
//			write_led_io(LED_IO1,LED_OFF);
//			write_led_io(LED_IO2,LED_OFF);
//			write_led_io(LED_IO3,LED_ON);
//		}
//		
//		//当遥控器左上角的拨档开关拨到上
//		if(rc.sw1==1){
//			write_led_io(LED_IO6,LED_ON);
//			write_led_io(LED_IO7,LED_OFF);
//			write_led_io(LED_IO8,LED_OFF);
//		}
//		
//		//当遥控器左上角的拨档开关拨到中
//		if(rc.sw1==3){
//			write_led_io(LED_IO6,LED_OFF);
//			write_led_io(LED_IO7,LED_ON);
//			write_led_io(LED_IO8,LED_OFF);
//		}
//		
//		//当遥控器左上角的拨档开关拨到下
//		if(rc.sw1==2){
//			write_led_io(LED_IO6,LED_OFF);
//			write_led_io(LED_IO7,LED_OFF);
//			write_led_io(LED_IO8,LED_ON);
//		}
//	}
//}


////实验：使用遥控器控制灯效、电机
//#include "rm_hal_lib.h"
//#include "cmsis_os.h"
//#include "uart_device.h"

//uint8_t can1_send_data[8];

//int16_t current=0;

//uint8_t n;

//void study_task(const void*argu){
//	
//	can_device_init();
//	
//	while(1){
//		
//		//将遥控器通道2的值赋予电机ID为1的电机
//		current=rc.ch2;
//		
//		can1_send_data[0]=current>>8;
//		can1_send_data[1]=current;
//		write_can(USER_CAN1,0x200,can1_send_data);
//		
//		write_led_io(LED_IO1,LED_OFF);
//		write_led_io(LED_IO2,LED_OFF);
//		write_led_io(LED_IO3,LED_OFF);
//		write_led_io(LED_IO4,LED_OFF);
//		write_led_io(LED_IO5,LED_OFF);
//		write_led_io(LED_IO6,LED_OFF);
//		write_led_io(LED_IO7,LED_OFF);
//		write_led_io(LED_IO8,LED_OFF);
//		
//		//因为我们的摇杆会产生一个-660~660的连续的值，所以我们分段来控制灯效
//		if(rc.ch2>400&&rc.ch2<660)
//			write_led_io(LED_IO1,LED_ON);
//		if(rc.ch2>200&&rc.ch2<440)
//			write_led_io(LED_IO2,LED_ON);
//		if(rc.ch2>0&&rc.ch2<200)
//			write_led_io(LED_IO3,LED_ON);
//		if(rc.ch2>-200&&rc.ch2<-100)
//			write_led_io(LED_IO4,LED_ON);
//		if(rc.ch2>-400&&rc.ch2<-200)
//			write_led_io(LED_IO5,LED_ON);
//		if(rc.ch2>-660&&rc.ch2<-400)
//			write_led_io(LED_IO6,LED_ON);
//		
//		osDelay(10);
//	}
//}


////实验：使用遥控器控制底盘的前进后退左转右转
//#include "rm_hal_lib.h"
//#include "cmsis_os.h"
//#include "uart_device.h"
//#include "can_device.h"

////初始化速度 speed_x=前进后退 speed_y=左转右转
//int16_t speed_x=0,speed_y=0;

//int16_t current[4]={0,0,0,0};

//void study_task(const void*argu){
//	
//	while(1){
//		
//		//右侧拨杆拨到上
//		if(rc.sw2==1){
//			
//			speed_x = rc.ch2*3;//遥控器摇杆放大3倍
//			speed_y = rc.ch1*3;//遥控器摇杆放大3倍
//			
//			current[0]= -speed_x + speed_y;
//			current[1]= speed_x + speed_y;
//			current[2]= speed_x + speed_y;
//			current[3]= -speed_x + speed_y;
//			
//			send_chassis_moto_current(current);
//			
//			osDelay(10);
//		}else{
//
//			send_chassis_moto_zero_current();
//
//			osDelay(10);
//		}
//	}
//}


////实验：使用遥控器控制底盘的前进后退左转右转,加入PID算法
//#include "rm_hal_lib.h"
//#include "cmsis_os.h"
//#include "uart_device.h"
//#include "can_device.h"
//#include "pid.h"

//int16_t speed_x=0,speed_y=0;

//int16_t current[4]={0,0,0,0};

//float speed[4];

//void study_task(const void*argu){
//	
//	for(int k=0;k<4;k++){
//		pid_init(&pid_wheel_spd[k],10000,1000,3.0f,0.1,0);
//	}
//	
//	while(1){
//		
//		speed_x = rc.ch2*2;
//		speed_y = rc.ch1*2;
//		
//		speed[0]=-speed_x + speed_y;
//		speed[1]= speed_x + speed_y;
//		speed[2]= speed_x + speed_y;
//		speed[3]=-speed_x + speed_y;
//		
//		current[0] = pid_calc(&pid_wheel_spd[0], moto_chassis[0].speed_rpm, speed[0]);
//		current[1] = pid_calc(&pid_wheel_spd[1], moto_chassis[1].speed_rpm, speed[1]);
//		current[2] = pid_calc(&pid_wheel_spd[2], moto_chassis[2].speed_rpm, speed[2]);
//		current[3] = pid_calc(&pid_wheel_spd[3], moto_chassis[3].speed_rpm, speed[3]);
//		
//		send_chassis_moto_current(current);
//		
//		osDelay(10);
//	}
//}


////实验：使用遥控器控制底盘的前进、后退、左转、右转、左平移、右平移
//#include "rm_hal_lib.h"
//#include "cmsis_os.h"
//#include "uart_device.h"
//#include "can_device.h"
//#include "pid.h"

////初始化速度 speed_x=前进后退 speed_y=左转右转 speed_turn= 左平移右平移
//int16_t speed_x=0,speed_y=0,speed_turn=0;

//int16_t current[4]={0,0,0,0};

//float speed[4];

//void study_task(const void*argu){
//	
//	for(int k=0;k<4;k++){
//		
//		//PID算法，确保四个轮子的速度一致
//		pid_init(&pid_wheel_spd[k],16000,1000,3.0f,0.1,0);
//		
//	}
//	
//	while(1){
//		
//		//遥控器通道信号放大4倍，影响机器人运动速度
//		speed_x = rc.ch2*4;
//		speed_y = rc.ch1*4;
//		speed_turn = rc.ch3*4;
//		
//		speed[0] = -speed_x + speed_y + speed_turn;
//		speed[1] = speed_x + speed_y + speed_turn;
//		speed[2] = speed_x - speed_y + speed_turn;
//		speed[3] = -speed_x - speed_y + speed_turn;

//		current[0] = pid_calc(&pid_wheel_spd[0], moto_chassis[0].speed_rpm, speed[0]);
//		current[1] = pid_calc(&pid_wheel_spd[1], moto_chassis[1].speed_rpm, speed[1]);
//		current[2] = pid_calc(&pid_wheel_spd[2], moto_chassis[2].speed_rpm, speed[2]);
//		current[3] = pid_calc(&pid_wheel_spd[3], moto_chassis[3].speed_rpm, speed[3]);

//		send_chassis_moto_current(current);
//		
//		osDelay(10);
//	}
//}
