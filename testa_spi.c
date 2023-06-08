
#include <wiringPi.h>
#include <stdio.h>
#include <wiringPiSPI.h>
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
			
			double diff_r = er - er_old;
			double diff_l = el - el_old;
		
			//printf("SpeedM1=%d Speed_M2=%d Enkoder_M1= %d Enkoder_M2= %d\n", MotorData.Act_Speed_M1,MotorData.Act_Speed_M2,MotorData.Encoder_M1,MotorData.Encoder_M2);
			printf("Enkoder_M1= %d Enkoder_M2= %d\n", MotorData.Encoder_M1,MotorData.Encoder_M2);
			delay(50);
			//printf("Speed_M1=%d Speed_M2=%d Enkoder_M1_diff= %d Enkoder_M2_diff=  %d\n", MotorData.Act_Speed_M1,MotorData.Act_Speed_M2,diff_r, diff_l);
			
			er_old = er;
			el_old = el;
			
			
			
		}
	    //MotorData.Set_Speed_M1=1000;
		//MotorData.Set_Speed_M2=1000;
		//Send_Read_Motor_Data(&MotorData);
	
}
