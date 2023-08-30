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
double angle_diff;
double offset_x;
double offset_y;

// intial position 
//int x_initial = 1200;
//int y_initial = 250;
//int a_initial = (90 * M_PI) / 180;
//double x_initial = 1200;
//double y_initial = 380;

double x_initial = 1170;
double y_initial = 390;
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
ofstream odo_file("odometery_readings.txt");
ofstream kalman_file("kalman_readings.txt");

int state = 6;
int csase6_direction = 0;
int kalman_flag = 0;
int box_area_con = 3000;

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
	
	/*
	// Make sure that the change in distance on both sides is not big because of the reset in encoder values
	if(abs(dDr) > 10 || abs(dDl) > 10)
	{
		return;
	}
	*/
	
	// Calculate change of relative movements
	double dD = (dDr + dDl) / 2;
	double dA = (dDr - dDl) / wheelbase;	
	
	// change in X and Y
	double dx = dD * cos(a_old + dA/2);
	double dy = dD * sin(a_old + dA/2);
	
	//cout << "dx = " << dx <<endl;
	//cout << "dy = " << dy <<endl;
	//cout << "da/2 = " << ( dA/2 * M_PI /180) << endl;
	
	// Calculate new X, Y, A
	
	x = x_old + dx;
	y = y_old + dy;
	a = fmod((a_old + dA + (2 * M_PI)), 2 * M_PI);
	
	// (2 * M_PI) was added to ensure that the anglle is always in the range 0 - 360 and not negative
	
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
	
	// save the current as the old point for the next iteration
	x_old = x;
	y_old = y;
	a_old = a;
	p_old = p;

	
	// Print the Values
	
	cout << "Odometry Reading" << endl;
	cout << "X: " << x << endl;
	cout << "Y: " << y << endl;
	cout << "A: " << (a * 180 / M_PI)  << endl;
	//cout << "P:  "  << p << endl;
	
	odo_file << Local_Time() << " " << x << " " << y << " " << (a* 180 / M_PI) << " "<< " \n";
	//odo_file << Local_Time() << " " << x << " " << y << " " << a << " ";
	//odo_file << p(0,0) << " " << p(0,1) << " " << p(0,2) << " ";
	//odo_file << p(1,0) << " " << p(1,1) << " " << p(1,2) << " ";
	//odo_file << p(2,0) << " " << p(2,1) << " " << p(2,2) << " " << " \n";
	odo_file.flush();
	
	
}

void Kalman(void){
	
	kalman_flag = 0;
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
		
		//cout << "c_Cox: " << C_cox << endl;
		//cout << "c_odo: " << C_odo << endl;
		//cout << "c_Cox + C_odo" << C_cox + C_odo << endl;
		//cout << "Det:" << (C_cox + C_odo).determinant()<<endl;
		
		// Making sure that C_cox + C_odo is invertible 
		
		if(abs((C_cox + C_odo).determinant()) == 0)
		{
			// the matrix is not invertible;
			cout<<"Matrix is not invertable"<<endl;
			return;
		}
		
		
		double c_cox_det = C_cox.determinant();
		double c_odo_det = C_odo.determinant();
		
		//cout << "Inverse calculation c_cox: " << c_cox_det << endl;
		//cout << "Inverse calculation c_odo: " << c_odo_det << endl;
		
		//cout << "=!= odo " << (c_odo_det != c_odo_det) << endl;
		//cout << "=!= cox " << (c_cox_det != c_cox_det) << endl;
		
		if(c_cox_det != c_cox_det)
		{
			return;
		}
		
		if(c_odo_det != c_odo_det)
		{
			return;
		}
		
		
		//cout<<"Start Calculations"<<endl;
		// Calculate new Positions 	
		MatrixXd new_X_odo;
		new_X_odo = C_cox * (C_cox + C_odo).inverse() * X_odo;
		
		MatrixXd new_X_cox;
		new_X_cox = C_odo * (C_cox + C_odo).inverse() * X_cox;
		
		MatrixXd new_X = new_X_odo + new_X_cox;
		
		
		
		//cout << "Inverse calculation: " << (C_cox.inverse() + C_odo.inverse()).determinant() << endl;
		// Calculate New C
		Eigen::MatrixXd new_C;
		new_C = (C_cox.inverse() + C_odo.inverse()).inverse();
		
		// this line to insure that each cox calculation is used once for kalman
		cox_complete = 0;
		
		//cout<<"finish calculation"<<endl;
		/*
		if((abs(fmod(new_X(2, 0), 2 * M_PI) - a_old) * 180 / M_PI) > 10)
		{
			cout << "BIG ANGLE CHANGE KALMAN" << endl;
			kalman_file << Local_Time() << " " << (new_X(0, 0) - x_old) << " " << (new_X(1, 0) - y_old)  << " " << ((fmod(new_X(2, 0), 2 * M_PI) - a_old) * 180 / M_PI) << " ";
			kalman_file << new_C(0,0) << " " << new_C(0,1) << " " << new_C(0,2) << " ";
			kalman_file << new_C(1,0) << " " << new_C(1,1) << " " << new_C(1,2) << " ";
			kalman_file << new_C(2,0) << " " << new_C(2,1) << " " << new_C(2,2) << "            " << "BIG ANGLE CHANGE KALMAN"<< " \n";
		
			return;
		}
		*/
		
		int kalman_x_flag = 1;
		int kalman_y_flag = 1;
		
		if( abs(new_X(0, 0) - x_old) > 150)
		{
			kalman_x_flag = 0;
			cout << "BIG X CHANGE KALMAN" << endl;
			kalman_file << Local_Time() << " " << (new_X(0, 0) - x_old) << " " << (new_X(1, 0) - y_old)  << " " << ((fmod(new_X(2, 0), 2 * M_PI) - a_old) * 180 / M_PI) << " "<< "BIG X CHANGE KALMAN"<< " \n";
			//kalman_file << new_C(0,0) << " " << new_C(0,1) << " " << new_C(0,2) << " ";
			//kalman_file << new_C(1,0) << " " << new_C(1,1) << " " << new_C(1,2) << " ";
			//kalman_file << new_C(2,0) << " " << new_C(2,1) << " " << new_C(2,2) << "            " << "BIG ANGLE CHANGE KALMAN"<< " \n";
		
			//return;
		}
		
		
		if( abs(new_X(1, 0) - y_old) > 150)
		{
			kalman_y_flag = 0;
			cout << "BIG Y CHANGE KALMAN" << endl;
			kalman_file << Local_Time() << " " << (new_X(0, 0) - x_old) << " " << (new_X(1, 0) - y_old)  << " " << ((fmod(new_X(2, 0), 2 * M_PI) - a_old) * 180 / M_PI) << " "<< "BIG Y CHANGE KALMAN"<< " \n";
			//kalman_file << new_C(0,0) << " " << new_C(0,1) << " " << new_C(0,2) << " ";
			//kalman_file << new_C(1,0) << " " << new_C(1,1) << " " << new_C(1,2) << " ";
			//kalman_file << new_C(2,0) << " " << new_C(2,1) << " " << new_C(2,2) << "            " << "BIG ANGLE CHANGE KALMAN"<< " \n";
		
			return;
		}
		
		
		
		// Store current corrected position and covariance matrix 
		
		if(kalman_x_flag)
		{
			x = new_X(0, 0);
		}
		
		if(kalman_y_flag)
		{
			y = new_X(1, 0);
		}
		
		
		double old_kal_a = a;
		a = fmod(new_X(2, 0), 2 * M_PI);
		cout << "Kalman x diffrence= " << (x - x_old) << endl;
		cout << "Kalman y diffrence= " << (y - y_old) << endl;
		cout << "Kalman a diffrence= " << ((a - a_old) * 180 / M_PI) << endl;
		p = new_C;
		
		
		
		cout << "X: " << x << endl;
		cout << "Y: " << y << endl;
		cout << "A: " << ((a) * 180 / M_PI)<< endl;
		
		//cout << "Started writing to kalman"<< endl;
		kalman_file << Local_Time() << " " << (x - x_old) << " " << (y - y_old)  << " " << ((a - a_old) * 180 / M_PI) << " " << " \n";
		//kalman_file << Local_Time() << " " << new_X(0, 0) << " " << new_X(1, 0)  << " " << (new_X(2, 0) * 180 / M_PI) << " " << " \n";
		//kalman_file << new_C(0,0) << " " << new_C(0,1) << " " << new_C(0,2) << " ";
		//kalman_file << new_C(1,0) << " " << new_C(1,1) << " " << new_C(1,2) << " ";
		//kalman_file << new_C(2,0) << " " << new_C(2,1) << " " << new_C(2,2) << " " << " \n";
		kalman_file.flush();
		//cout << "ended writing to kalman"<< endl;
		kalman_flag = 1;
		
		// save the current as the old point for the next odometry iteration
		x_old = x;
		y_old = y;
		a_old = a;
		p_old = p;
	}
	cout << "Kalman Flag =  " << kalman_flag << endl;
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

void left(int speed)
{
	MotorData.Set_Speed_M1 = speed;
	MotorData.Set_Speed_M2 = -speed;

}

void right(int speed)
{
	MotorData.Set_Speed_M1 = -speed;
	MotorData.Set_Speed_M2 = speed;

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
	
	x = x_initial;
	y = y_initial;
	a = a_initial;
	
	
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
	//kalman_file.open("kalman_readings.txt", ios::out | ios::trunc);
	
	odo_file.close();
    kalman_file.close();
	
	if(!odo_file.is_open())
	{
		odo_file.open("odometery_readings.txt", std::ios::out | std::ios::trunc);
		odo_file.tie(nullptr);
		cout << "odo file opened" << endl;
		//cout << "Odometry readings txt file failed to open" << endl;
		//return;
	}
	else
	{
		cout << "odo file is already open" << endl;
	}
	  
	if(!kalman_file.is_open())
	{
		kalman_file.open("kalman_readings.txt", ios::out | ios::trunc);
		kalman_file.tie(nullptr);
		cout << "kalman file opened" << endl;
		//cout << "kalman readings txt file failed to open" << endl;
		//return;
	}  
	else
	{
		cout << "Kalman file is already open" << endl;
	}
	
	odo_file << "Time[s] X[mm] Y[mm] a[rad] c11 c12 c13 c21 c22 c23 c31 c32 c33 \n"; 
	kalman_file << "Time[s] X[mm] Y[mm] a[rad] c11 c12 c13 c21 c22 c23 c31 c32 c33 \n"; 
	odo_file.flush();
	kalman_file.flush();
	double angle_deg;
	int temp_counter = 0;
	
	while(1){
		
		
		Send_Read_Motor_Data(&MotorData);
		odometry();
		Kalman();
		delay(50);
		switch(state)
		{
			case 1: // Start state
			forward();
			Counter_walk++;
			//if(Counter_walk > 200)
			if(Counter_walk > 300)
			{
				//state = 2;
				state = 6;
				Counter_walk = 0;
			}
			break;
			case 2: // check the box if it is one or zero
			stop();
			if(box_N == 1)
			{
				state = 3;
			}
			else
			{
				state = 7;
			}
			break;
			case 3: // Turn toward the box if box = 1
			if(d < 0)
			{left(700);}
			if(d > 0)
			{right(700);}
			if(abs(d) < 80)
			{state = 5;}
			break;
			case 4: // turn away from box if box = 0
			if(d > 0)
			{left(2000);}
			if(d < 0)
			{right(2000);}
			if(abs(d) > 120 || box_N == 1)
			{
				//stop();
				state = 2;
			}
			if(d == 0)
			{
				state = 2;
			}
			break;
			case 5: // move toward box
			
			//cout << "Box Area = " << box_area << endl;
			//cout << "Box Area condition = " << box_area_con << endl;
 			if(box_area > box_area_con && abs(d) > 0)
			//if(abs(d) > 0)
			{
				if(got_box == 1)
				{
					box_area_con += 400;
				}
				forward();
				Counter_walk++;
				if(Counter_walk > 50)
				{
					state = 2;
					Counter_walk = 0;
					//stop();
				}
			}
			else 
			{
				//stop();
				//forward();
				//Counter_walk++;
				forward();
				Counter_walk++;
				if(Counter_walk > 100)
				{
					
					stop();
					if(got_box == 0) // got the first box
					{
						stop();
						got_box = 1;
						state = 8;
					}
					else
					{
						state = 8;
						cout<< "Got box 2" << endl;
						//stop();
					}
				}
			}
			break;
			
			case 6: // prepare for seaching for second box 
			forward();
			Counter_walk++;
			if(Counter_walk > 120)
			{
				angle_deg = ((a) * 180 / M_PI);
				cout << "Current angle: " << angle_deg << endl;
				//if(angle_deg > -270)
				if(angle_deg > 80 && angle_deg < 270)
				{
					//a = a - (3 * M_PI / 180);
					left(1000);
				}
				else
				{	
					//state = 7;
					state = 10;
					Counter_walk = 0;
				}
			}
			break;
			
			case 7:
			cout << "entered case 7" << endl;
			angle_deg = ((a) * 180 / M_PI);
			if(abs(d) == 0 || box_N == 0)
			{
				if(angle_deg >= 90)
				{
					left(1500);
					//right(3000);
				}
				else if (angle_deg < 90)
				{
					right(1500);
					//left(3000);
				}
			}
			else
			{
				state = 2;
			}
			//stop();
			break;
			
			case 8: 
			stop();
			if(kalman_flag == 1 || temp_counter < 2)
			//if(kalman_flag == 1)
			{
				cout << "Enter case 8" << endl;
				temp_counter++;
				if(kalman_flag == 1)
				{
					temp_counter = 0;
				}
				
				xe_new_location = 1200 - x;
				ye_new_location = 380 - y;
				//heading_new_location  = atan2(xe_new_location, ye_new_location);
				heading_new_location  = atan2(ye_new_location, xe_new_location);
				heading_new_location = ((heading_new_location) * 180 / M_PI);
				angle_deg = ((a) * 180 / M_PI);
				angle_diff = heading_new_location - angle_deg;
				
				// change this part to make the angle between 180 and -180
				//angle_diff = fmod(angle_diff + 180, 360) - 180;
				if(angle_diff > 10)
				{
					left(3000);
					//right(2000);
				}
				else if(angle_diff < -10)
				{
					right(3000);
					//left(2000);
				}
				else
				{
					state = 9;
				}
				cout << "xe_new_location" << xe_new_location << endl;
				cout << "ye_new_location: " << ye_new_location << endl;
				cout << "angle_deg = " << (angle_deg)<< endl;
				cout << "heading_new_location = " << (heading_new_location)<< endl;
				cout << "angle_diff = " << (angle_diff)<< endl;
			}
			
			break;
			
			case 9:
			ye_new_location = 480 - y;
			cout << "ye_new_location" << ye_new_location << endl;
			if(ye_new_location < 0)
			{
				forward();
				Counter_walk++;
				if(Counter_walk > 70)
				{
					state = 8;
					Counter_walk = 0;
				}
			}
			else
			{
				stop();
			}
				
			break;
			
			case 10:
			stop();
			break;
			
		}
		//cout << "d: " << d << endl;
		cout << "State: " << state << endl;
		//delay(50);
		//Send_Read_Motor_Data(&MotorData);
		//odometry();
		//Kalman();
		
		/*
		
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
		*/	
	}
	
	
	
}



