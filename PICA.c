#include <24FJ48GA002.h>
#include "BL_Support.h"
#use delay (internal = 8MHz, clock = 32MHz)
#PIN_SELECT U1RX = PIN_B12
#PIN_SELECT U1TX = PIN_B13
#PIN_SELECT OC1 = PIN_B3
#PIN_SELECT OC2 = PIN_B2
#PIN_SELECT OC3 = PIN_B0
#PIN_SELECT OC4 = PIN_B1
#PIN_SELECT OC5 = PIN_B14

#use rs232(UART1, BAUD = 9600, XMIT = PIN_B13, RCV = PIN_B12)
float e,s,p,theta_d,count =0,countz=0;
float ez,sz,pz,theta_z;
int u,uz,Kp,Ki,Kd;
int direction=1,direction_z=1,state,id,data,sign,Duty_Servo,action;
char SM_id = 1;
int getPackage = 0;
float array[8];
float A =0.000;
int Z;

#define DEVICE_ID   11

char* print_float(float data){
	int intDist = data / 1;
    int dotDist = (((intDist>>15)*-2)+1) * ((data * 1000.0f) - (intDist * 1000));
    char stringFloat[20];
    sprintf(stringFloat, "%d.%d", intDist, dotDist);
    return stringFloat;
}
void print_float(char* stringResult, float data){
	int intDist = data / 1;
    int dotDist = (((intDist>>15)*-2)+1) * ((data * 1000.0f) - (intDist * 1000));
    sprintf(stringResult, "%d.%d", intDist, dotDist);
}


#INT_EXT0
void INT_EXT_INPUT0(void){
	if(direction == 1){
	//	count+=1;
		count+=0.0056;
		//count+=0.005328;
	}else if(direction == 0){
		count-=0.0056;
		//count-=0.005328;
	}
}
#INT_TIMER1
void TIMER1_ist(){
	countz=countz+0.006;
}


void Init_Interrupts() {
disable_interrupts(GLOBAL);
setup_timer1(TMR_INTERNAL | TMR_DIV_BY_256,625); //100 HZ
setup_timer2(TMR_INTERNAL | TMR_DIV_BY_8,2000);
//setup_timer3(TMR_INTERNAL | TMR_DIV_BY_256,189); //330 HZ
setup_timer3(TMR_INTERNAL | TMR_DIV_BY_256,1250);
enable_interrupts(INT_EXT0);
ext_int_edge(0,L_TO_H); // Rising Edge
ext_int_edge(1,L_TO_H); // Rising Edge
clear_interrupt(INT_RDA);  
enable_interrupts(INT_RDA);
enable_interrupts(GLOBAL);
setup_compare(1,COMPARE_PWM|COMPARE_TIMER2);
setup_compare(2,COMPARE_PWM|COMPARE_TIMER2);
setup_compare(3,COMPARE_PWM|COMPARE_TIMER2);
setup_compare(4,COMPARE_PWM|COMPARE_TIMER2);
setup_compare(5,COMPARE_PWM|COMPARE_TIMER3);
}

void grip(int Servo_D)
{
	int  D = Servo_D;
	Duty_Servo = D*64 ;
	Duty_Servo = Duty_Servo/135;
	printf("%d\n" ,Duty_Servo);
	delay_ms(100);	
	Duty_Servo =  Duty_Servo+26;
	set_pwm_duty(5,Duty_Servo);
}	


void grab(int Servo_D)
{
	if( Servo_D == 1){
		set_pwm_duty(5,80);
}
	else if (Servo_D == 2){
		set_pwm_duty(5,100);
}
	else if (Servo_D == 3){
		set_pwm_duty(5,132);
}	
}




void driveMotor(int duty){
	if (duty>100){
		duty = 100;
	}
	else if (duty<-100){
		duty = -100;
	}
	duty = duty*20;
	if (duty < 0){
	set_pwm_duty(1,abs(duty));	
	set_pwm_duty(2,0);			
	direction=0;		
	}

	else if (duty>=0){
	set_pwm_duty(1,0);	
	set_pwm_duty(2,abs(duty));		
	direction = 1;
	}
}

void driveMotor_z(int duty){
	if (duty>100){
		duty = 100;
	}
	else if (duty<-100){
		duty = -100;
	}
	duty = duty*20;
	if (duty < 0){
	set_pwm_duty(3,abs(duty));	
	set_pwm_duty(4,0);			
	direction_z=0;		
	}

	else if (duty>=0){
	set_pwm_duty(3,0);	
	set_pwm_duty(4,abs(duty));		
	direction_z = 1;
	}
}


void setzero()
{
	do{		// Turn down motorA cw
		set_pwm_duty(1,2000);	
		set_pwm_duty(2,0);			
		}
	while(input_state(PIN_B5)==1);		//PIN_B4 = DI0
	do{		// Turn right motorA cw
		set_pwm_duty(1,2000);	
		set_pwm_duty(2,0);			
		}
	while(input_state(PIN_B4)==1);		//PIN_B5 = DI1
		set_pwm_duty(1,0);				//motorA stop (0,0)
		set_pwm_duty(2,0);			
		putc('k'); 
		//printf("setzero finish");
		count =0;
		u = 0;
}
void setzero_z()
{
	do{		// Turn up 
		set_pwm_duty(3,2000);	
		set_pwm_duty(4,0);			
		}
	while(input_state(PIN_A2)==1);		//PIN_B4 = DI0
		set_pwm_duty(3,0);				//motorA stop (0,0)
		set_pwm_duty(4,0);			
		putc('k');
		//printf("setzero z axis finish");
		count =0;
		u = 0;
}
void pid(float delta_a){
		e = 0;
		theta_d = delta_a;
		e = theta_d - count;
		s = s+e;
		if(abs(e)>0.15){
			u = Kp*e+Ki*s+Kd*(e-p);
		}
		else{
			u = 0;
		}
		driveMotor(u);
		p = e;
}

void pid_z(float delta_z){
		theta_z = delta_z;
		ez = theta_z - countz;
		sz =sz+ez;
		if(abs(ez)>0.3){
			uz = 20*ez+0*sz+0*(ez-pz);
		}
		else{
			uz = 0;
			printf("fail");
		}
		pz=ez;
		driveMotor_z(uz);
}

// Package
void SM_RxD(int c){
		if (SM_id <= 2){
			if (c == 255){ // bit 1-2 Check
				SM_id++;
		}else{
				SM_id = 1;
		}
		}
		else if (SM_id == 3){  // bit 3 Device id
			if (c == DEVICE_ID){
				id = c;
				SM_id++;
		}
		}
		else if (SM_id == 4){ // bit 4 Action
			SM_id++;
			if (c <= 5 ){
				state = c;
			}else if (c == 1){ // setzero
				getPackage = 1;
				SM_id = 1;
			}
		}
		else if (SM_id >= 5 && SM_id <= 7){ // bit 5 direction bit 6-7 data
			if(c>=0){ 
				array[SM_id - 5] = c;
				if (SM_id == 5){
					sign= array[0]; // value 1 = direction + , value 0 = direction -
				}	
				SM_id++;
			}
			else {
				SM_id = 1;
			}
		}
		else if (SM_id == 8){ // bit 8
			data = ~(id + state + (int)array[0] + (int)array[1] + (int)array[2]) + 1;
			data &= 0xff;
			getPackage = 1;
			SM_id = 1;
		}
		}	


#INT_RDA
void UART1_Isr() {
   	int c = getc();
   	SM_RxD(c);
}



void main(){
	Init_Interrupts();
	//setzero();
	//set_pwm_duty(5,100);
	float float_num;
	A =0.000;
	s = 0;
	p = 0;
	Kp = 200;
	Ki = 0.1;
	Kd = 0;
	while(TRUE){
	if (getPackage == 1 && state == 1){ // state 1 set zero
			getPackage = 0;
			//printf("checksum : %d",data);
			delay_ms(10);
			setzero();
			//putc('k'); 
			//printf("state%d\n",state);
			//Sent request
					}
	if (getPackage == 1 && state == 2){ // state 2 goto position
			e=0,u=0;
			delay_ms(10);
			//printf("state%d\n",state);
//Check direction		
			if(sign  == 1){ // If direction = 1 position A +
				A = array[1] + (array[2]/100);
				//printf("direction%d\n",sign);
				//printf("A%f\n",A);
			}
			else if(sign == 0){ // If direction = 0 position A -
				A = -1*(array[1] + (array[2]/100));
				//printf("direction%d\n",sign);
			}
//PID
			do{				
				pid(A);
				//printf("\nresult = %s\n", print_float(u));
				//printf("\nerror = %s\n", print_float(e));
			}while(abs(u)>0);
//Verify	
			do{
				delay_ms(1);
			}while(input_state(PIN_A1)==0);			
			putc('k');
			//printf("A and B Ready");
			
			getPackage = 0;
			// Verify
		}
	if (getPackage == 1 && state == 3){ // state 3 setzero z axis
				//printf("state%d\n",state);
				//printf("setzero_z");
				setzero_z(); 
				getPackage = 0;
					}
	if (getPackage == 1 && state == 4){ // state 4 go position z
				A = array[1] + (array[2]/100);
				countz=0;enable_interrupts(INT_TIMER1);
			do{
				set_pwm_duty(3,0);	
				set_pwm_duty(4,2000);
				//printf("\countzz = %s\n", print_float(countz));
			}while(countz<=A);
				set_pwm_duty(3,0);	
				set_pwm_duty(4,0);
				disable_interrupts(INT_TIMER1);
				putc('k');
				getPackage = 0;
				}
	if (getPackage == 1 && state == 5){ // state 5 degree z
			action=(int)array[0];
			grab(action);
			getPackage = 0;
			printf("%d %d %d \n",(int)array[0],array[1],array[2]);
			printf("state%d\n",state);
					}
	delay_ms(10);
	//	printf("\nresult = %s\n", print_float(u));
	//	printf("\nerror = %s\n", print_float(e));
}
}