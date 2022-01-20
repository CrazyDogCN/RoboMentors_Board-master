/****************************************************************************
 *  RoboMentors www.robomentors.com.
 *	wechat:superzz8080
 *	����RoboMaster���ο���
 ***************************************************************************/


//ʵ�飺ִ���� �����ת����ת�����١����ٵĿ���
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
	uart_receive_start(USER_UART3,usart3_recv,4);//��uart3���գ��ѽ��յ������ݷŵ�usart3_recv

	while(1){
		if(a=='1'){
			write_led_io(LED_IO1,LED_ON);
			osDelay(100);
			write_led_io(LED_IO1,LED_OFF);
		}
		//���ٹ���
		for(current=0;current<30000;current+=100){
			can1_send_data[0]=current>>8;
			can1_send_data[1]=current;
			write_can(USER_CAN1,0x1ff,can1_send_data);
			osDelay(10);
		}
		
		//������ٹ���
		for(current=10000;current>-30000;current-=100){
			can1_send_data[0]=current>>8;
			can1_send_data[1]=current;
			write_can(USER_CAN1,0x1ff,can1_send_data);
			osDelay(10);
		}
		
		//������ٹ���
		for(current=-30000;current<0;current+=100){
			can1_send_data[0]=current>>8;
			can1_send_data[1]=current;
			write_can(USER_CAN1,0x1ff,can1_send_data);
			osDelay(10);
		}
	}
}


////ʵ�飺���Ƽ��� PID���㷽��
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


////ʵ�飺ʹ��ң�������Ƶ�Ч
//#include "rm_hal_lib.h"
//#include "cmsis_os.h"
//#include "uart_device.h"

//void study_task(const void*argu){
//	
//	while(1){
//		
//		//��ң�������Ͻǲ��������Ƶ���
//		if(rc.sw2==1){
//			write_led_io(LED_IO1,LED_ON);
//			write_led_io(LED_IO2,LED_OFF);
//			write_led_io(LED_IO3,LED_OFF);
//		}
//		
//		//��ң�������ϽǵĲ������ز�����
//		if(rc.sw2==3){
//			write_led_io(LED_IO1,LED_OFF);
//			write_led_io(LED_IO2,LED_ON);
//			write_led_io(LED_IO3,LED_OFF);
//		}
//		
//		//��ң�������ϽǵĲ������ز�����
//		if(rc.sw2==2){
//			write_led_io(LED_IO1,LED_OFF);
//			write_led_io(LED_IO2,LED_OFF);
//			write_led_io(LED_IO3,LED_ON);
//		}
//		
//		//��ң�������ϽǵĲ������ز�����
//		if(rc.sw1==1){
//			write_led_io(LED_IO6,LED_ON);
//			write_led_io(LED_IO7,LED_OFF);
//			write_led_io(LED_IO8,LED_OFF);
//		}
//		
//		//��ң�������ϽǵĲ������ز�����
//		if(rc.sw1==3){
//			write_led_io(LED_IO6,LED_OFF);
//			write_led_io(LED_IO7,LED_ON);
//			write_led_io(LED_IO8,LED_OFF);
//		}
//		
//		//��ң�������ϽǵĲ������ز�����
//		if(rc.sw1==2){
//			write_led_io(LED_IO6,LED_OFF);
//			write_led_io(LED_IO7,LED_OFF);
//			write_led_io(LED_IO8,LED_ON);
//		}
//	}
//}


////ʵ�飺ʹ��ң�������Ƶ�Ч�����
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
//		//��ң����ͨ��2��ֵ������IDΪ1�ĵ��
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
//		//��Ϊ���ǵ�ҡ�˻����һ��-660~660��������ֵ���������Ƿֶ������Ƶ�Ч
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


////ʵ�飺ʹ��ң�������Ƶ��̵�ǰ��������ת��ת
//#include "rm_hal_lib.h"
//#include "cmsis_os.h"
//#include "uart_device.h"
//#include "can_device.h"

////��ʼ���ٶ� speed_x=ǰ������ speed_y=��ת��ת
//int16_t speed_x=0,speed_y=0;

//int16_t current[4]={0,0,0,0};

//void study_task(const void*argu){
//	
//	while(1){
//		
//		//�Ҳದ�˲�����
//		if(rc.sw2==1){
//			
//			speed_x = rc.ch2*3;//ң����ҡ�˷Ŵ�3��
//			speed_y = rc.ch1*3;//ң����ҡ�˷Ŵ�3��
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


////ʵ�飺ʹ��ң�������Ƶ��̵�ǰ��������ת��ת,����PID�㷨
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


////ʵ�飺ʹ��ң�������Ƶ��̵�ǰ�������ˡ���ת����ת����ƽ�ơ���ƽ��
//#include "rm_hal_lib.h"
//#include "cmsis_os.h"
//#include "uart_device.h"
//#include "can_device.h"
//#include "pid.h"

////��ʼ���ٶ� speed_x=ǰ������ speed_y=��ת��ת speed_turn= ��ƽ����ƽ��
//int16_t speed_x=0,speed_y=0,speed_turn=0;

//int16_t current[4]={0,0,0,0};

//float speed[4];

//void study_task(const void*argu){
//	
//	for(int k=0;k<4;k++){
//		
//		//PID�㷨��ȷ���ĸ����ӵ��ٶ�һ��
//		pid_init(&pid_wheel_spd[k],16000,1000,3.0f,0.1,0);
//		
//	}
//	
//	while(1){
//		
//		//ң����ͨ���źŷŴ�4����Ӱ��������˶��ٶ�
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
