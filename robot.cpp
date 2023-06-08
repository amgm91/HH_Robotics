
#include <iostream>
#include <errno.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "robot.h"
#include "global.h"
#include <time.h>
#include <sys/time.h>   // struct itimerval
#include <signal.h>  // struct sigaction

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE !FALSE
#endif

#define BUFFSIZE 1023

using namespace std;
static const int  CHANNEL=1;

FILE *pFd_t;
int Send_ch;

//void server_time(int *year,int *mon,int *day,int *hour,int *min,double *sec)
void server_time(int * hour, int * min, double * sec){

	struct timeval tv;
    struct timezone tz;
    struct tm *now;
    time_t tim;

    tim=time(NULL);
    now=localtime(&tim);
    gettimeofday(&tv, &tz);
 // *year = now->tm_year+1900;//*mon = now->tm_mon+1;//*day = now->tm_mday;
    *hour = now->tm_hour;
    *min = now->tm_min;
    *sec = (double)now->tm_sec + (double)tv.tv_usec/1000000.0;
}

void event_handler(int signum){

	int T_len;
	int t_hour, t_min;
	double t_sec;
    static char T_buff[BUFFSIZE];



	struct timeval tv;
    struct timezone tz;
    struct tm *now;
    time_t tim;

    tim=time(NULL);
    now=localtime(&tim);
    gettimeofday(&tv, &tz);
 // *year = now->tm_year+1900;//*mon = now->tm_mon+1;//*day = now->tm_mday;
    t_hour = now->tm_hour;
    t_min = now->tm_min;
    t_sec = (double)now->tm_sec + (double)tv.tv_usec/1000000.0;

//	server_time(&t_hour, &t_min, &t_sec);
//	sprintf(T_buff,"h=%3d min=%3d sec=%3.5f nbr_ch=%d",
//			t_hour, t_min, t_sec, Send_ch);
//	printf("%s",T_buff);
/*	T_len = strlen(T_buff)+1;
	T_buff[T_len] = 0;
	T_len = T_len + Send_ch;*/
//	puts(T_buff);
//	fprintf(pFd_t,"%s\n",T_buff);
//	if(ReceiveTimer)
//		ReceiveTimer--;
}


void *Robot(void *){
	
	struct sigaction sa;
    struct itimerval timer;
    const int iHz = 1; 
	int T_hour, T_min, Stop = FALSE;
	double T_sec;
    
	unsigned long micros;
	clock_t start, end;
	unsigned char buffer[100];
	TYPE_SPI_TX *pSPI_TX;
	TYPE_SPI_RX *pSPI_RX;
	
	pSPI_RX = (TYPE_SPI_RX *)buffer;
	pSPI_TX = (TYPE_SPI_TX *)buffer;
	wiringPiSPISetup(CHANNEL, 4000000);

  // Setup signal handler to create time interrupt
  memset (&sa, 0, sizeof (sa));
  sa.sa_handler = &event_handler;
  sigaction (SIGALRM, &sa, NULL);
  timer.it_value.tv_sec = 1/iHz;
  timer.it_value.tv_usec = 1000000/iHz - (int)(1/iHz)*1000000;
  timer.it_interval.tv_sec = 1/iHz;
  timer.it_interval.tv_usec = 1000000/iHz - (int)(1/iHz)*1000000;
  setitimer (ITIMER_REAL, &timer, NULL);

	while(1){
//		start = clock();
//		printf("while in robot");
		sleep(1);
/*		end = clock();
		//printf("sleep %d [ms] robot\n",start);
		if (Black_Board_Heart_Beat > 0){
			Black_Board_Heart_Beat--;
			// TODO fill buffer
			pSPI_TX->Set_Speed_M0 = (signed short)Black_Board_Speed_Left;
			wiringPiSPIDataRW(CHANNEL, buffer, 32);
			Black_Board_Encoder_Left = pSPI_RX->Encoder_M0;
			// TODO read buffer
		}*/
	}
}


