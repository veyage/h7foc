#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H
#include "main.h"
#include "tim.h"
extern float reload_value;

typedef struct {
    float Kp;                  // ����ϵ��
    float Ki;                  // ����ϵ��
    float Kd;                  // ΢��ϵ��
    float Last_Error;          // ��һ�����
    float Last_Integration;    // ��һ�λ���ֵ
    float Last_Output;         // ��һ�����ֵ
    unsigned long Timestamp_Last; // ��һ��ʱ���
} PID_Controller_t;
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd);
float PID_Controller(PID_Controller_t *pid, float Error);
float  _constrain(float amt, float low, float high);
float PID_Controller1(PID_Controller_t *pid, float Error);
#endif


