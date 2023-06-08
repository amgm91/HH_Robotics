#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include "camera.h"
#include "global.h"
#include "controller.h"
#include "scan_match.h"
#include "lidar.h"
#include <time.h>

// g++ main.cpp camera.cpp -o main -lpthread $(pkg-config --libs --cflags opencv)

using namespace std;

TYPE_STATE Robot_State; // Statemachine is nice



int main(int argc, char **argv){
	
	pthread_t thread1, thread2, thread3, thread4;
	int p1,p2,p3,p4;
	 
   	p1 = pthread_create(&thread1, NULL, Pos_Controller, NULL);
	p2 = pthread_create(&thread2, NULL, Camera_Client, NULL);
   	p3 = pthread_create(&thread3, NULL, Scan_Match, NULL);
 	p4 = pthread_create(&thread4, NULL, Lidar_Server, NULL);
	
	//if(p1 | p2 | p4 |p3)
	 // cout << "threads missing";
   
   Robot_State = Start; 
  
   //double Start_Time = Local_Time();
   
  while(1){
		switch(Robot_State){
		case Start:
		break;
		case Move_2_Object:
		break;
		case Find_Obj:
		break;
		default:
			//Black_Board_Desired_Robot_Speed = 0.0;
			//Black_Board_Desired_Robot_Steer = 0.0;
		break;
		}
		
   }
   return 0;
   //exit(0); 
}


