#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
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
#include "scan_match.h"

#define SPEED 3000	


using namespace std;
using namespace Eigen;

MotorDataType MotorData;

// The Gearbox ratio is 32
// The wheel radius is 15 mm
int got_box = 0;
int first_run_flag = 1;
int wheelbase = 130;
int wheel_r = 15;
double mm_per_pulse = (2 * M_PI * wheel_r) / (1024 * 6.6 * 32 *4);
double dt = 0.05; // time sample
double sigma_wheel_encoder = 0.5 / 12;
double sigma_l = sigma_wheel_encoder;
double sigma_r = sigma_wheel_encoder;

double xe_new_location;
double ye_new_location;
double heading_new_location;

// intial position 
//int x_initial = 1200;
//int y_initial = 250;
//int a_initial = (90 * M_PI) / 180;
double x_initial = 1200;
double y_initial = 380;
double a_initial = (90 * M_PI) / 180;
MatrixXd p_initial(3,3);

// old position

double x_old;
double y_old;
double a_old;
MatrixXd p_old(3,3);

double dr_old;
double dl_old;

// current position 

double x;
double y;
double a;
MatrixXd p(3,3);


// M1 --> Right wheel 
// M2 --> Left wheel 

double E_right_old;
double E_left_old;
double E_right;
double E_left;

// initilize files to write location data to
//ofstream odo_file;
ofstream kalman_file;

int state = 1;
/*
 *  1 --> Start
 *  2 --> search for the box
 *  3 --> move closer to box
 *  4 --> Box is one
 *  5 --> Box is zero 
 * 	6 --> got 1st box
 * 	7 --> go home(got 2nd box)
 * 
*/
void odometry(void){
	
	// Read current encoder values
	E_right = MotorData.Encoder_M1;
	E_left = MotorData.Encoder_M2;
	
	
	// Transform pulses to mm
	double dDr = (E_right - E_right_old) * mm_per_pulse;
	double dDl = (E_left - E_left_old) * mm_per_pulse;
	
	// save the current encoder readings as old readings to be used in the next iteration
	E_right_old = E_right;
	E_left_old = E_left;
	
	// Make sure that the change in distance on both sides is not big because of the reset in encoder values
	if(abs(dDr) > 20 || abs(dDl) > 20)
	{
		return;
	}
	
	
	// Calculate change of relative movements
	double dD = (dDr + dDl) / 2;
	double dA = (dDr - dDl) / wheelbase;	
	
	// change in X and Y
	double dx = dD * cos(a_old + dA/2);
	double dy = dD * sin(a_old + dA/2);
	
	// Calculate new X, Y, A
	
	x = x_old + dx;
	y = y_old + dy;
	a = (a_old + dA);
	
	// Predict the uncertainty in the state variables
	
	MatrixXd Axya(3,3);
	Axya << 1, 0, -dD * sin(a_old + dA/2),
			0, 1, dD * cos(a_old + dA/2),
			0, 0, 1;
	
	MatrixXd Jrl(3,2);
	Jrl<< 0.5*cos(a_old+(dA/2))-((dD/(2*wheelbase))*sin(a+(dA/2))), 0.5*cos(a+(dA/2))+((dD/(2*wheelbase))*sin(a+(dA/2))),
		  0.5*sin(a+(dA/2))+((dD/(2*wheelbase))*cos(a+(dA/2))),     0.5*sin(a+(dA/2))-((dD/(2*wheelbase))*cos(a+(dA/2))),
		  (1/wheelbase),                                           (-1/wheelbase);
	
	double k = 0.01;
	
	MatrixXd Crl(2,2);
	Crl << k*dDr, 0, 
		   0, k*dDl;
		   
	MatrixXd Jb (3,1);
	Jb << (dD*(dA/pow(wheelbase, 2)))*sin(a_old+(dA/2*wheelbase)),
		  (-dD*(dA/pow(wheelbase,2)))*cos(a_old+(dA/2*wheelbase)),
		  -(dA/wheelbase);
		  
	double Cb = 0.01;
	
	MatrixXd Cxya_new = Axya * p_old * Axya.transpose() + Jrl * Crl * Jrl.transpose() + Jb * Cb * Jb.transpose();
	
	p = Cxya_new;
	
	// Print the Values
	
	//cout << "X: " << x << endl;
	//cout << "Y: " << y << endl;
	//cout << "A: " << (a * 180 / M_PI)  << endl;
	//cout << "P:  "  << p << endl;
	
	//odo_file << Local_Time() << " " << x << " " << y << " " << a << " ";
	//odo_file << p(0,0) << " " << p(0,1) << " " << p(0,2) << " ";
	//odo_file << p(1,0) << " " << p(1,1) << " " << p(1,2) << " ";
	//odo_file << p(2,0) << " " << p(2,1) << " " << p(2,2) << " " << " \n";
	
	// save the current as the old point for the next iteration
	x_old = x;
	y_old = y;
	a_old = a;
	p_old = p;
}

void Kalman(void){
	
	if(cox_complete == 1)
	{
		cout<<"enter Kalman"<<endl;
		
		// Covariance Matrices
		MatrixXd C_cox(3,3);
		MatrixXd C_odo(3,3);
	
		// Position Matrices 
		VectorXd X_cox(3);
		VectorXd X_odo(3);
	
		// Load Covariance Matrices 
		C_cox = c_cox;
		C_odo = p;
	
		// Load Positions
		X_odo << x, y, a;
		X_cox << x_cox, y_cox, a_cox;
		
		// Making sure that C_cox + C_odo is invertible 
		
		if(abs((C_cox + C_odo).determinant()) != 0)
		{
			// the matrix is invertible;
			cout<<"Matrix is invertable"<<endl;
			return;
		}
		/*
		if(abs((C_cox).determinant()) <= 0)
		{
			// the matrix is invertible;
			cout<<"cox Matrix is invertable"<<endl;
			return;
		}
		
		if(abs((C_odo).determinant()) <= 0)
		{
			// the matrix is invertible;
			cout<<"odo Matrix is invertable"<<endl;
			return;
		}
		*/
		cout<<"Start Calculations"<<endl;
		// Calculate new Positions 	
		MatrixXd new_X_odo;
		new_X_odo = C_cox * (C_cox + C_odo).inverse() * X_odo;
		
		MatrixXd new_X_cox;
		new_X_cox = C_odo * (C_cox + C_odo).inverse() * X_cox;
		
		MatrixXd new_X = new_X_odo + new_X_cox;
		
		// Calculate New C
		Eigen::MatrixXd new_C;
		new_C = (C_cox.inverse() + C_odo.inverse()).inverse();
		cout<<"finish calculation"<<endl;
		
		// Store current corrected position and covariance matrix 
		x = new_X(0, 0);
		y = new_X(1, 0);
		a = new_X(2, 0);
		
		p = new_C;
		
		cout << "X: " << x << endl;
		cout << "Y: " << y << endl;
		cout << "A: " << ((a) * 180 / M_PI)<< endl;
		//cout << "Started writing to kalman"<< endl;
		//kalman_file << Local_Time() << " " << new_X(0, 0) << " " << new_X(1, 0) << " " << new_X(2, 0) << " ";
		//kalman_file << new_C(0,0) << " " << new_C(0,1) << " " << new_C(0,2) << " ";
		//kalman_file << new_C(1,0) << " " << new_C(1,1) << " " << new_C(1,2) << " ";
		//kalman_file << new_C(2,0) << " " << new_C(2,1) << " " << new_C(2,2) << " " << " \n";
		
		//cout << "ended writing to kalman"<< endl;
	
	}
}

void SPI_Robot(void){
	
	unsigned char buffer[32];
/*	TYPE_SPI_TX *pSPI_TX;
	TYPE_SPI_RX *pSPI_RX;
	
	pSPI_RX = (TYPE_SPI_RX *)buffer;
	pSPI_TX = (TYPE_SPI_TX *)buffer;

	pSPI_TX->Control = 0;
	pSPI_TX->Current_Loop = 0;
	pSPI_TX->Speed_Loop = 0;
	pSPI_TX->Set_PWM_M0 = 0;
	pSPI_TX->Set_PWM_M1 = 0;
	pSPI_TX->Digital_Out = 0;
	pSPI_TX->Spare1 = 0;
	pSPI_TX->P_Value_Current = 0.0;
	pSPI_TX->P_Value_Speed = 0.0;
	pSPI_TX->I_Value_Speed = 0.0;
	pSPI_TX->Speed_Stepper = 0;
	pSPI_TX->Position_Servo = 0;
	pSPI_TX->Status = 1;
	pSPI_TX->Set_Speed_M0 = Black_Board_Desired_Speed_Right; //100
	pSPI_TX->Set_Speed_M1 = Black_Board_Desired_Speed_Left; //50
	pSPI_TX->Position_Servo = 0;
	pSPI_TX->Heart_Beat = Black_Board_Heart_Beat;
	wiringPiSPIDataRW(1, buffer, 32);
    Black_Board_Encoder_Right =	pSPI_RX->Encoder_M0;
    Black_Board_Encoder_Left =	pSPI_RX->Encoder_M1;*/
}



void Position_Feedback_Controller(void){


}


void Invers_Kinematic(void){

}

void server_time(int * hour, int * min, double * sec);

void forward()
{
	MotorData.Set_Speed_M1 = SPEED;
	MotorData.Set_Speed_M2 = SPEED;
}

void backward()
{
	MotorData.Set_Speed_M1 = -SPEED;
	MotorData.Set_Speed_M2 = -SPEED; 

}

void left()
{
	MotorData.Set_Speed_M1 = SPEED;
	MotorData.Set_Speed_M2 = -SPEED;

}

void right()
{
	MotorData.Set_Speed_M1 = -SPEED;
	MotorData.Set_Speed_M2 = SPEED;

}

void stop()
{
	MotorData.Set_Speed_M1 = 0;
	MotorData.Set_Speed_M2 = 0;

}

void *Pos_Controller(void* ){

	// initialize location with predetrmined initial on the arena
	p_initial << 1,0,0,0,1,0,0,0,pow(1*M_PI/180, 2);
	x_old = x_initial;
	y_old = y_initial;
	a_old = a_initial;
	
	wiringPiSetup();
	//wiringPiSPISetup(1, 4000000);
	wiringPiSPISetup(1, 1000000);
	int Counter_walk = 0;
	
	for(int i = 0; i < 10; i++)
	{
		MotorData.Set_Speed_M1 = 10;
		MotorData.Set_Speed_M2 = 10;
		Send_Read_Motor_Data(&MotorData);
		delay(50);
	}
	
	
	
	E_right_old = MotorData.Encoder_M1;
	E_left_old = MotorData.Encoder_M2;
	
	//odo_file.open("odometery_readings.txt", ios::trunc);
	//kalman_file.open("kalman_readings.txt", ios::trunc);
	/*
	if(!odo_file.is_open())
	{
		cout << "Odometry readings txt file failed to open" << endl;
		//return;
	}
	  
	if(!kalman_file.is_open())
	{
		cout << "kalman readings txt file failed to open" << endl;
		//return;
	}  
	*/
	//odo_file << "Time[s] X[mm] Y[mm] a[rad] c11 c12 c13 c21 c22 c23 c31 c32 c33 \n"; 
	//kalman_file << "Time[s] X[mm] Y[mm] a[rad] c11 c12 c13 c21 c22 c23 c31 c32 c33 \n"; 
	  
	
	while(1){
		
		//cout << "Time: " <<  Local_Time()<< endl;
		//forward();
		//delay(50);
		//Send_Read_Motor_Data(&MotorData);
		//E_left = MotorData.Encoder_M2;
		//cout << " E_new: "<< static_cast<int>(E_left) << endl;
		/*
		switch(state)
		{
			case 1:
			forward();
			Counter_walk++;
			if(Counter_walk > 200)
			{
				state = 2;
				Counter_walk = 0;
			}
			break;
			case 2:
			left();
			if(d != 0)
			state = 3;
			break;
			case 3:
			if( d > 0)
			{left();}
			if(d > 0)
			{right();}
			if(abs(d) < 15)
			{state = 4;}
			break;
			case 4:
			if( (dArea/1000000) > 20)
			{forward();}
			else
			{
				if(box_N == 1)
				{state = 5;}
				else
				{state = 6;}
			}
			break;
			case 5:
			if((dArea/1000000) > 2)
			{forward();}
			else
			{state = 7;}
			break;
			case 6:
			left();
			Counter_walk++;
			if(Counter_walk > 30)
			{
				
				forward();
			}
			else if(Counter_walk > 60)
			{
				state = 2;
				Counter_walk = 0;
			}
			break;
			case 7:
			if(got_box == 0)
			{
				state = 2;
				got_box = 1;
			}
			else
			{ state = 8;}
			break;
			case 8:
			xe_new_location = 1200 - x;
			ye_new_location = 380 - y;
			heading_new_location  = atan2(xe_new_location, ye_new_location);
			if( abs(heading_new_location) > 20)
			{
				left();
			}
			else if( abs(xe_new_location) > 10 && abs(ye_new_location) > 10)
			{
				forward();
			}
			else
			{state = 9;}
			break;	
		}
		delay(50);
		//std::this_thread::sleep_for(std::chrono::milliseconds(50));
		Send_Read_Motor_Data(&MotorData);
		odometry();
		Kalman();
		*/
		
		if(Counter_walk < 500)
		{  
			Counter_walk++;
			forward();
			delay(50);
			Send_Read_Motor_Data(&MotorData);
			odometry();
			Kalman();
			
			//cout << "X: " << x << endl;
			//cout << "Y: " << y << endl;
			//cout << "A: " << (a * 180 / M_PI)  << endl;
		}
		else 
		{
			stop();
			Send_Read_Motor_Data(&MotorData);
			delay(50);
		}	
	}
	
}



