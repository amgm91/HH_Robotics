#ifndef _ROBOT_H
#define _ROBOT_H



void * Robot(void *);

typedef struct{
  unsigned char Status;
  unsigned char Control;
  unsigned short Current_Loop;
  unsigned short Speed_Loop;
  signed short Set_PWM_M0;
  signed short Set_PWM_M1;
  signed short Set_Speed_M0;
  signed short Set_Speed_M1;
  unsigned char Digital_Out;
  unsigned char Spare1;
  float P_Value_Current;
  float P_Value_Speed;
  float I_Value_Speed;
  signed short Speed_Stepper;
  unsigned char Position_Servo;
  unsigned char Heart_Beat;
} TYPE_SPI_TX;


typedef struct{
  unsigned char Status;       // 1 byte
  unsigned char Digital_In;  // 1
  unsigned short Current_M0; // 2
  unsigned short Current_M1; // 2
  signed short Speed_M0;     // 2
  signed short Speed_M1;     // 2
  unsigned short Spare0;     // 2
  int Encoder_M0;           // 4
  int Encoder_M1;           // 4
} TYPE_SPI_RX;


#endif
