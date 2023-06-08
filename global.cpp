#include "global.h"
#include "controller.h"
#include "camera.h"
#include <sys/time.h>   // struct itimerval
#include <time.h>  



double Local_Time(void){

    struct timeval tv;
    struct tm *now;
    time_t tim;

    tim=time(NULL);
    now=localtime(&tim);

    return now->tm_hour*3600 + now->tm_min*60 + (double)now->tm_sec + (double)tv.tv_usec/1000000.0;
}
