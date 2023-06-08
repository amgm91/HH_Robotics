#include <unistd.h>
#include <iostream>
#include <stdio.h>
//include "odometry.h"
#include "controller.h"
#include <sys/time.h>   // struct itimerval
#include <signal.h>  // struct sigaction
#include <wiringPiSPI.h>
#include <wiringPi.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include "global.h"
#include <time.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <eigen/Eigen/Dense>
#include "robot_spec.h"
#include "camera.h"
#include "spi_com.h"

#define SPEED 1000	


MotorDataType MotorData;

using namespace std;
using namespace Eigen;

VectorXd odometry(double encoder_newl,double encoder_newr,double encoder_oldl,double encoder_oldr, double xold,double yold,double aold){
	
	double wheel_base = 130;               // [mm]
        double wheel_diameter = 28;           // [mm]
        double pulses_per_revolution = 1024;        
    
	double sigma_wheel_encoder =0.1;
	double mm_per_pulse = wheel_diameter * M_PI / (pulses_per_revolution*4*32*6.6);   // [mm / pulse]

	
	double wheel1 = encoder_newl;
	double wheel2 = encoder_newr;


	double wheel1_mm = wheel1* mm_per_pulse;
	double wheel2_mm = wheel2* mm_per_pulse;
            
	double dr_old = encoder_oldr;
    	double dl_old = encoder_oldl;

	dr_old = dr_old * mm_per_pulse;
	dl_old = dl_old * mm_per_pulse;

	double dDr = wheel1_mm - dr_old;
        double dDl = wheel2_mm - dl_old;



	// Calculate change of relative movements
	double dD = (dDr + dDl) / 2;
	double dA = (dDr - dDl) / wheel_base;
	
	if(abs(dD) > 20 || abs(dA) > 20)
     	{
		 cout<< "Error" << endl;
	}

	
	// change in X and Y
	double dx = dD * cos(aold + dA/2);
	double dy = dD * sin(aold + dA/2);
	
	// Calculate new X, Y, A
	
	double x = xold + dx;
	double y = yold + dy;
	//a = fmod(a_old + dA, 2*M_PI);
	double a = (aold + dA);
	
	
	
	// Print the Values
	
	cout << "X: " << x << endl;
	cout << "Y: " << y << endl;
	cout << "A: " << (a * 180 / M_PI)  << endl;
	
	VectorXd xya(3);
	xya(0) = x;
	xya(1) = y;
	xya(2) = a;
	return(xya);
}

int main()
{

wiringPiSetup();
wiringPiSPISetup(1, 4000000);

double x_initial = 1200;
double y_initial = 250;
double a_initial = (45 * M_PI) / 180;
int Counter_walk = 0;

MotorData.Set_Speed_M1 = 3000;
MotorData.Set_Speed_M2 = 3000;
Send_Read_Motor_Data(&MotorData);

double E_right_old = MotorData.Encoder_M1;
double E_left_old = MotorData.Encoder_M2;

double E_right_new ,E_left_new;

while(1){
	
		if(Counter_walk < 5000){  // Todo! select time

			Counter_walk++;
			MotorData.Set_Speed_M1 = 3000;
			MotorData.Set_Speed_M2 = 3000;
			Send_Read_Motor_Data(&MotorData);

			E_right_new = MotorData.Encoder_M1;
                        E_left_new = MotorData.Encoder_M2;
		
			VectorXd xya = odometry(E_left_new,E_right_new,E_left_old,E_right_old,x_initial,y_initial,a_initial);

			E_right_old = E_right_new;
			E_left_old = E_left_new;
			
			x_initial = xya(0);
			y_initial = xya(1);
			a_initial = xya(2);
		}
				
	}


}


