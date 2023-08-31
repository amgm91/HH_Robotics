
#include <wiringPi.h>
#include <stdio.h>
#include <wiringPiSPI.h>
#include <math.h>
#include "spi_com.h"


MotorDataType MotorData;

static const int SPI_Channel = 1;

short Des_Speed = 3000;
int Select = 0;
int Counter = 0;




int main(void){

	wiringPiSetup(); 
	wiringPiSPISetup(SPI_Channel, 1000000);

	double er_old = MotorData.Encoder_M1;
	double el_old = MotorData.Encoder_M2;
	
	
	double er;
	double el;
	double x;
	double y;
	double a;
	double x_old = 1200;
	double y_old = 380;
	double a_old = (90 * M_PI) / 180;
	double mm_per_pulse = (2 * M_PI * 15) / (1024 * 6.6 * 32 *3);
	
	
	while(1){
		    MotorData.Set_Speed_M1=Des_Speed;
			MotorData.Set_Speed_M2=Des_Speed;
	
			Send_Read_Motor_Data(&MotorData);

		/*delay(50);
		Counter++;
		if(Counter > 40){
			Counter =0;
			switch(Select){
				case 0:
					Des_Speed = 3000;
					Select = 1;
				break;
				case 1:
					Des_Speed = -3000;
					Select = 2;
				break;
				case 2:
					Des_Speed = -3000;
					Select = 3;
				break;
				case 3:
					Des_Speed = 0;
					Select = 0;
				break;
			}*/
			er = MotorData.Encoder_M1;
			el = MotorData.Encoder_M2;
			
			double vr = (er - er_old) * mm_per_pulse * 20;
			double vl = (el - el_old) * mm_per_pulse * 20;
			
			er_old = er;
			el_old = el;
			
			double v = (vr + vl) / 2;
			double w = (vr - vl) / 130;
			
			double L = v * 0.05;
			double dA = w * 0.05;
			
			double dx = L * cos(dA/2);
			double dy = L * sin(dA/2);
			
			double a = a_old + dA;
			double x = x_old + (dx * cos(a)) - (dy * sin(a));
			double y = y_old + (dy * cos(a)) + (dx * sin(a));
			
			
			//double dx = dD * cos(a_old + dA/2);
			//double dy = dD * sin(a_old + dA/2);
			
			
			printf("dx = %f\n", x - x_old);
			printf("dy = %f\n", y - y_old);
			printf("dA = %f\n", dA);
			
			a_old = a;
			x_old = x;
			y_old = y;
		
		
			//printf("SpeedM1=%d Speed_M2=%d Enkoder_M1= %d Enkoder_M2= %d\n", MotorData.Act_Speed_M1,MotorData.Act_Speed_M2,MotorData.Encoder_M1,MotorData.Encoder_M2);
			//printf("Enkoder_M1= %d Enkoder_M2= %d\n", MotorData.Encoder_M1,MotorData.Encoder_M2);
			delay(50);
			//printf("Speed_M1=%d Speed_M2=%d Enkoder_M1_diff= %d Enkoder_M2_diff=  %d\n", MotorData.Act_Speed_M1,MotorData.Act_Speed_M2,diff_r, diff_l);
			
			
			
			
			
		}
	    //MotorData.Set_Speed_M1=1000;
		//MotorData.Set_Speed_M2=1000;
		//Send_Read_Motor_Data(&MotorData);
	
}
