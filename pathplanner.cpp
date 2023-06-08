#include <math.h>
#include "global.h"
#include <stdio.h>
#include "robot_spec.h"
#include <iostream>
#include <stdlib.h> 


using namespace std;
/*
FILE *FilePointer_Enc;

	
void Speed_Profile( int straight,int EndPos,int StartPos ,int *Encoder_Left,int *Encoder_Right){

	int Old_Encoder_Speed = 0;
    int OldSpeed = 0;
	int BreakPos=0; 
	int Speed = 0;   
	int Max_Counter = 0;
		
	if(EndPos > StartPos){
		while ((EndPos > *Encoder_Right) && (Speed >= 0) && (Max_Counter++ < 100000)){
		//BreakTime = Speed/MinAcc; % Time for brake [Time = Speed/acc]
		//BreakPos = Breake Time * Speed/2; 
			if (Speed < MaxSpeed && (*Encoder_Right  < EndPos + BreakPos)){
				Speed = Old_Encoder_Speed + MaxAcc * SampleTime;
				BreakPos = Speed*Speed/(2*DecAcc);
			}
			else if(*Encoder_Right  >  EndPos + BreakPos){
				Speed = Old_Encoder_Speed + DecAcc * SampleTime;
			}
			else{
				Speed = MaxSpeed;
			}
		
			*Encoder_Right = *Encoder_Right + Speed * SampleTime;
			if(straight == 0){
				*Encoder_Left = *Encoder_Left - Speed * SampleTime;
			}
			else{
				*Encoder_Left = *Encoder_Left + Speed * SampleTime;
			}
			fprintf(FilePointer_Enc,"%d %d %d\n",*Encoder_Left, *Encoder_Right, straight);
			Old_Encoder_Speed = Speed;
		}
		
    }
    else if(EndPos < StartPos){ 
		while((*Encoder_Right > EndPos) && (Speed <= 0) && (Max_Counter++ < 100000)) {
			if(Speed > -MaxSpeed & *Encoder_Right > (EndPos + BreakPos)){
				Speed = (Old_Encoder_Speed - MaxAcc * SampleTime);
				BreakPos = -Speed*Speed/(2*DecAcc);
			}
			else if (*Encoder_Right < (EndPos + BreakPos)){
				Speed = (Old_Encoder_Speed - DecAcc * SampleTime);
			}
			else{
				Speed = -MaxSpeed;
			}
			*Encoder_Right = *Encoder_Right + Speed * SampleTime;
			if(straight == 0){
				*Encoder_Left = *Encoder_Left - Speed * SampleTime;
			}
			else{
				*Encoder_Left = *Encoder_Left + Speed * SampleTime;
			}            
			fprintf(FilePointer_Enc,"%d %d %d\n",*Encoder_Left, *Encoder_Right, straight);
			Old_Encoder_Speed = Speed;
		}
	}	
}

	//TYPE_POS Pos_Odometry = {300, 600, M_PI/2};	
	
void Create_WayPoints(void){
	
	int Dir;
	int Encoder_Left;
	int Encoder_Right;
	int Old_Encoder_Left = 0;
	int Old_Encoder_Right = 0;
	float Speed_Left_Wheel;
	float Speed_Right_Wheel;
	float Omega;
	float Delta_Theta,Robot_Speed,Distance,Delta_X,Delta_Y;
	
	TYPE_POS Pos_Kalman = {Black_Board_Pos_Kalman.X,Black_Board_Pos_Kalman.Y, Black_Board_Pos_Kalman.Theta};	
	//TYPE_POS Pos_Kalman;
	FILE *FilePointer_Pos ;
    FilePointer_Pos  = fopen("pos_points.txt","w");
	if(FilePointer_Pos == NULL){
		cout << "could't open file" << endl;
		exit(0);	
	}
	
    FilePointer_Enc  = fopen("encoder_points.txt","r");
	if(FilePointer_Enc == NULL){
		cout << "could't open file" << endl;
		exit(0);	
	}
	
	
		
	while(fscanf(FilePointer_Enc,"%d %d %d" ,&Encoder_Left, &Encoder_Right, &Dir)!=EOF){
		Speed_Right_Wheel = (Encoder_Right - Old_Encoder_Right)/(SampleTime*Nbr_Puls_MM);
		Speed_Left_Wheel = (Encoder_Left - Old_Encoder_Left)/(SampleTime*Nbr_Puls_MM);	
		Old_Encoder_Left = Encoder_Left;
		Old_Encoder_Right = Encoder_Right;		
		Omega = (Speed_Right_Wheel-Speed_Left_Wheel)/Robot_Width;
		Delta_Theta = Omega *  SampleTime;
		Robot_Speed = (Speed_Right_Wheel+Speed_Left_Wheel)/2;
		Distance = Robot_Speed * SampleTime;
		Delta_X = Distance * cos(Delta_Theta/2);
		Delta_Y = Distance * sin(Delta_Theta/2);
		Pos_Kalman.Theta = Pos_Kalman.Theta + Delta_Theta;
		Pos_Kalman.X = Pos_Kalman.X + Delta_X * cos(Pos_Kalman.Theta) - Delta_Y * sin(Pos_Kalman.Theta); 
		Pos_Kalman.Y = Pos_Kalman.Y + Delta_Y * cos(Pos_Kalman.Theta) + Delta_X * sin(Pos_Kalman.Theta);
		//cout << "Pos_Odometry.Theta" << Pos_Odometry.Theta  <<endl;
		//fprintf(FilePointer_Pos,"%f %f %f %d\n",Pos_Odometry.X + Black_Board_Pos_Kalman.X, Pos_Odometry.Y+Black_Board_Pos_Kalman.Y ,Pos_Odometry.Theta +Black_Board_Pos_Kalman.Theta, Dir);
		fprintf(FilePointer_Pos,"%f %f %f %d\n",Pos_Kalman.X , Pos_Kalman.Y ,Pos_Kalman.Theta, Dir);

	}
	for(int i=0;i<15;i++)
		fprintf(FilePointer_Pos,"%f %f %f %d\n",Pos_Kalman.X , Pos_Kalman.Y ,Pos_Kalman.Theta, Dir);
	fclose(FilePointer_Pos);
}



void pathplanner(){

 
	float X = Black_Board_Pos_Goal.X - Black_Board_Pos_Odometry.X; 
    float Y = Black_Board_Pos_Goal.Y - Black_Board_Pos_Odometry.Y;
    //float Theta = Black_Board_Pos_Home.Theta - Black_Board_Pos_Odometry.Theta ;   
    float Theta = Black_Board_Pos_Goal.Theta ;           
        
    int StartPos = 0;  //encoder value,Position
    int EndPos;
	float Distance;
	float Turn_deg;	
    float Alpha;
	int Encoder_Right=0;
	int Encoder_Left=0;


	cout << "X = " << X << "Y = " << Y << endl;
    FilePointer_Enc = fopen("encoder_points.txt","w");
	if(FilePointer_Enc == NULL){
		cout << "could't open file" << endl;
		exit(0);	
	}

    if(abs(Y) < 0.00001 && X >0){
		Alpha = 0;
	}
    else if(abs(Y) < 0.00001 && X < 0){
        Alpha = M_PI;
    }
    else if((X < 0 && Y < 0) ||( X < 0 && Y > 0) ){
        Alpha = M_PI+atan(Y/X);
	}
	else if(abs(X) < 0.00001 && Y>0){
        Alpha = M_PI/2;
    }
    else if(abs(X) < 0.00001 && Y<0){
        Alpha = -M_PI/2;
    }
     else if(abs(X) < 0.00001 && abs(Y) < 0.00001){
        Alpha = 0;
    }
    else{
		Alpha = atan(Y/X);
    }
   // cout << "ALPHA = " << Alpha << endl;
    Alpha = Alpha - Black_Board_Pos_Kalman.Theta; 
    //cout << "ALPHA2 = " << Alpha << endl;
        //-----turn----
	Distance = Alpha * Robot_Width/2;
    EndPos = (int)(Nbr_Puls_MM*Distance); 
    Speed_Profile(0, EndPos,StartPos,&Encoder_Left,&Encoder_Right);    
        //----straight-----
	StartPos = EndPos;
    Distance = sqrt(X*X+Y*Y);
    EndPos = StartPos +  (int)(Nbr_Puls_MM*Distance);        
    Speed_Profile(1, EndPos,StartPos,&Encoder_Left,&Encoder_Right);
       //----turn-------

    StartPos = EndPos;
    Turn_deg = -(Alpha - Theta) - Black_Board_Pos_Kalman.Theta;
    Distance = Turn_deg * Robot_Width/2;	
    EndPos = StartPos +  (int)(Nbr_Puls_MM*Distance); 
    Speed_Profile(0, EndPos,StartPos,&Encoder_Left,&Encoder_Right);
    //cout << "Black_Board_Pos_Kalman. = " << Black_Board_Pos_Kalman.X << "Black_Board_Pos_Kalman.y = " << Black_Board_Pos_Kalman.Y <<endl;
   // cout << "Black_Board_Pos_Kalman.Theta" << Black_Board_Pos_Kalman.Theta << "Theta = " << Theta << endl;
	fclose(FilePointer_Enc);
	
	Create_WayPoints();
}
*/
