#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include "global.h"

#include <vector>
#include <eigen/Eigen/Dense>

//#include "cox.h"
#include "lidar.h"
#include "controller.h"

using namespace std;
using namespace Eigen;

MatrixXd lines(4,4);
MatrixXd ui(4,2);


int hello_flag = 1;
int cox_complete = 0;

// Output
int x_cox = 1200;
int y_cox = 380;
int a_cox = (90 * M_PI) / 180;
MatrixXd c_cox(3,3);

// Sensor to robot transformation values
double sALFA = 0;
double sBETA = 0;
double sGAMMA = 0;

// updated changes 
double ddx;
double ddy;
double dda;


// Initial position 

double Rx;
double Ry;
double Ra;

// open file to write readings
ofstream cox_file;


void itializeMatrix()
{
	
	// The Lines need to changed to the arena dimentions 
	// instead of the box dimentions
	// and ui recalculated 
	
	// this matrix intilizes the lines and unit vector matrices
	lines<<0,0,2420,0,
		2420,0,2420,3630,
		2420,3630,0,3630,
		0,3630,0,0;
		
		
	ui << 1,0,
		0,1,
		-1,0,
		0,-1;	
		
	
}


void cox(void){
	//cout<<"Hello Cox"<<endl;
	itializeMatrix();
	cox_complete = 0;
	ddx = 0;
	ddy = 0;
	dda = 0;
	
	//Rx = 1200;
	//Ry = 380;
	//Ra = (90 * M_PI) / 180;
	
	Rx = x;
	Ry = y;
	Ra = a;
	int max_iterations = 400;
	MatrixXd Zi (4,2);
	Zi.col(0) = lines.col(2);
	Zi.col(1) = lines.col(3);
	
	VectorXd Ri = (ui.array()* Zi.array()).rowwise().sum();
	 
		 
	 for(int i =0; i<max_iterations; i++)
	 {
		  
		 VectorXd disV(dis.cols());
		 VectorXd angV(ang.cols());
		 
		 for(size_t j = 0; j<dis.cols();++j)
		 {
			 disV(j) = dis(0,j);
			 angV(j) = ang(0,j);
		 }
		 
		 // Translate and rotate the data points to cartesian
		 VectorXd x = disV.array() * angV.array().cos();
		 VectorXd y = disV.array() * angV.array().sin();
		 
		 
		 // Sensor coordinates to robot coordinates
		 // Create the translation matrix R1
		 Matrix3d R_1;
		 R_1 << cos(sGAMMA), -sin(sGAMMA), sALFA,
			sin(sGAMMA), cos(sGAMMA), sBETA,
			0, 0, 1;
		 
		 MatrixXd points(3,x.size());
		 points << x.transpose(), y.transpose(), VectorXd::Ones(x.size()).transpose();
		 
		 // transform to robot coordinates
		 //MatrixXd transformed_points = R_1 * points;
		 
		 MatrixXd Xs = R_1 * points;
		 
		 
		 // Extract the transformed X
		 //VectorXd Xs = transformed.ponints.row(0);
		 
		 // Robot Coordinates to world coordinates
		 // Create the translation matrix R_2
		 Matrix3d R_2;
		 R_2 << cos(Ra + dda), -sin(Ra + dda), Rx + ddx,
				sin(Ra + dda), cos(Ra + dda), Ry + ddy,
				0, 0, 1;
		
		 
		 MatrixXd points2(3, Xs.cols());
		 points2 << Xs.row(0), Xs.row(1), VectorXd::Ones(Xs.cols()).transpose();
		 
		 MatrixXd Xw = R_2 * points2;
		 
		 //VectorXd Xw = transformed_points.row(0);
		 
		 
		 // calculate targets for data points 
		 
		MatrixXd vi = Xw.topRows(2);
		MatrixXd yi(4,400);
		for(int j = 0; j < vi.cols();j++)
		{
			
			for(int k = 0; k<ui.rows();k++)
			{
				yi(k,j) = Ri(k) - ui.row(k).dot(vi.col(j));
			}
		}
		
		// Calculating the smallest distance from each point to each line 
		// and the index of the line with the smallest distance for each point 
		VectorXd min_line_len(yi.cols());
		VectorXd min_line_id(yi.cols());
		
		for (int j = 0; j < yi.cols(); j++) 
		{
			double min_len = yi.col(j).cwiseAbs().minCoeff( & min_line_id (j));
			min_line_len(j) = min_len;
		}
		
		// Create yi_min_abs matrix
		MatrixXd yi_min_abs(3,yi.cols());
		yi_min_abs.row(0) = Eigen::VectorXd::LinSpaced(yi.cols(), 1, yi.cols());
		yi_min_abs.row(1) = min_line_id.cast<double>();
		yi_min_abs.row(2) = min_line_len;
		
		// yi_min is a vector to store the extracted values
		
		VectorXd yi_min(yi_min_abs.cols());
		
		// Extract values from yi  based on yi_min_abs(1) indicies 
		
		for(int k = 0; k < yi_min_abs.cols(); k++)
		{
			yi_min(k) = yi(static_cast <int> (yi_min_abs(1, k)), k);
		}
		
		
		// Calculate the median of yi_min_ab(2) 
		VectorXd column = yi_min_abs.row(2);
		// Sort the point in column to find the median 
		sort(column.data(), column.data()+ column.size());
		
		double median;
		if (column .size() % 2 == 0)
		{
			median = (column(column.size() / 2 - 1) + column(column.size() / 2)) / 2.0;
		}
		else
		{
			median	 = column(column.size() / 2);
		}
		
		// Create a Matrix non_out that contains non outlier values
		MatrixXd non_out(3, 0);
		for(int k = 0; k < yi_min_abs.cols(); k++)
		{
			if(yi_min_abs(2,k) <= median)
			{
				non_out.conservativeResize(non_out.rows() , non_out.cols()+ 1);
				non_out.col(non_out.cols() - 1) = yi_min_abs.col(k);
			}
		}
		
		int counter = 0;
		double avg1 = 0;
		double avg2 = 0;
		
		for(int k = 0; k<non_out.cols();k++)
		{
			counter+=1;
			avg1 += vi(0,non_out(0, k) - 1);
			avg2 += vi(1,non_out(0, k) - 1);
		}
		
		// Calculate Vm that contains the mean of the points 
		VectorXd vm(2);
		vm << avg1/counter, avg2/counter;
		
		// Extract Values from ui based on the line indices in non_out(1)
		VectorXd xi1(non_out.cols());
		VectorXd xi2(non_out.cols());
		
		for(int k = 0; k<non_out.cols();k++)
		{
			xi1(k) = ui(non_out(1,k), 0);
			xi2(k) = ui(non_out(1,k), 1);
		}
		
		// Calculate the values of xi3
		VectorXd xi3(non_out.cols());
		MatrixXd rotMatrix(2,2);
		rotMatrix << 0, -1, 1, 0;
		
		for(int k = 0; k<non_out.cols();k++)
		{
			VectorXd viDiff = vi.col(non_out(0, k) - 1) - vm;
			xi3(k) = ui.row(non_out(1, k)) * rotMatrix * viDiff;
		}
		
		
		// Constructing A matrix
		MatrixXd A (xi1.size(), 3);
		A << xi1, xi2, xi3;
		
		// Create matrix yi_min_non_out
		
		VectorXd yi_min_non_out(non_out.cols());
		
		for(int k = 0; k<non_out.cols();k++)
		{
			yi_min_non_out(k) = yi_min(non_out(0, k) - 1);
		}
		
		//VectorXd yi_min_non_out_1 = yi_min_non_out.segment(0, A.size());
		
		
		// Calculate B = inv(A'*A)*A'* yi_min_non_out
		VectorXd B = (A.transpose() * A).inverse() * A.transpose() * yi_min_non_out;
		
		
		
		// Calculating S2
		
		double max_size_A = max(A.rows(), A.cols());
		VectorXd diff = yi_min_non_out - A * B;
		double S2 = diff.squaredNorm() / (max_size_A - 4);
		
		// Calculating C covariance matrix
		
		MatrixXd C = S2 * (A.transpose() * A).inverse();
		c_cox = C;
		
		ddx = ddx + B(0);
		ddy = ddy + B(1);
		dda = fmod(dda + B(2), 2 * M_PI);
		//dda = (dda + B(2));
		//dda = -dda;
		
		//x_cox = Rx + ddx;
		//y_cox = Ry + ddy;
		//a_cox = Ra + dda;
		
		//x_cox = Rx + ddx;
		//y_cox = Ry + ddy;
		//a_cox = Ra - dda;
		
		
		if (sqrt(B(0) * B(0) + B(1) * B(1)) < 5 && fabs(B(2)) < 0.1 * M_PI / 180) 
		{
			
			x_cox = Rx + ddx;
			y_cox = Ry + ddy;
			a_cox = Ra + dda;
			cox_file << Local_Time() << " " << x_cox << " " << y_cox << " " << a_cox << " ";
			cox_file << c_cox(0,0) << " " << c_cox(0,1) << " " << c_cox(0,2) << " ";
			cox_file << c_cox(1,0) << " " << c_cox(1,1) << " " << c_cox(1,2) << " ";
			cox_file << c_cox(2,0) << " " << c_cox(2,1) << " " << c_cox(2,2) << " " << " \n";
			cout<< "X: "<< x_cox << endl;
			cout<< "Y: "<< y_cox << endl; 
			//cout<< "Ra degree: "<<(Ra * 180 / M_PI) << endl; 
			//cout<< "Ra radians: "<< Ra << endl; 
			//cout<< "dda degrees: "<< (dda * 180 / M_PI) << endl; 
			//cout<< "dda radians: "<< (dda) << endl; 
			cout<< "A degress: "<< ((Ra + dda) * 180 / M_PI) << endl;
			//cout<< "A radians: "<< ((Ra + dda)) << endl;
			break;
		}
		
	 }
	 
	 Array_full_flag = 0;
	 cox_complete = 1;
	    
	/*
	if(Array_full_flag == 0)
	{
		int i;
		cout<<"Array started"<<endl;
		for(i = 0;i<400;i++)
		{
			cout<<"Distance"<<dis(0, i)<<endl;
			cout<<"angle"<<ang(0, i)<<endl;
		
		}
		cout<<"Array ended "<< i <<endl;
		//Array_full_flag = 0;
		hello_flag == 0;
	}
	*/
}


void *Scan_Match(void *){
	
	cox_file.open("scan_match_readings.txt", ios::trunc);
	
	
	if(!cox_file.is_open())
	{
		cout << "Scan match readings txt file failed to open" << endl;
		//return;
	}
	  
	
	cox_file << "Time[s] X[mm] Y[mm] a[rad] c11 c12 c13 c21 c22 c23 c31 c32 c33 \n"; 
	
	while(1){
		if(Array_full_flag == 1)
		cox();
	}
}
