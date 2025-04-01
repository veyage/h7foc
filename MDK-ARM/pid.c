#include "pid.h"

#define  limit            5
#define  Output_ramp      10000

// �޷�
float _constrain(float amt, float low, float high)    
{
    return ((amt < low) ? (low) : ((amt > high) ? (high) : (amt)));
}

// ���� PID �������ṹ��


// ��ʼ�� PID ������
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Last_Error = 0.0f;
    pid->Last_Integration = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Timestamp_Last = 0;
}

// PID ����������
float PID_Controller(PID_Controller_t *pid, float Error) {
    float Ts = 0.0f;
    uint32_t Timestamp = SysTick->VAL;
    if (Timestamp > pid->Timestamp_Last) {
        Ts = (float)(Timestamp - pid->Timestamp_Last) / (reload_value / 1000) * 1e-3;
    } else {
        Ts = (reload_value - pid->Timestamp_Last + Timestamp) / (reload_value / 1000) * 1e-3;
    }

    float proportion = pid->Kp * Error; // P��

    float integration = pid->Last_Integration + pid->Ki * 0.5f * Ts * Error; // I��
    integration = _constrain(integration, -limit, limit);

    float differential = pid->Kd * (Error - pid->Last_Error) / Ts; // D��

    float Output = proportion + integration + differential;
    Output = _constrain(Output, -limit, limit);

    pid->Last_Error = Error;
    pid->Last_Integration = integration;
    pid->Last_Output = Output;
    pid->Timestamp_Last = Timestamp;

    return Output;
}
float PID_Controller1(PID_Controller_t *pid, float Error) {
    float Ts = 0.0f;
    uint32_t Timestamp = SysTick->VAL;
    if (Timestamp > pid->Timestamp_Last) {
        Ts = (float)(Timestamp - pid->Timestamp_Last) / (reload_value / 1000) * 1e-6;
    } else {
        Ts = (reload_value - pid->Timestamp_Last + Timestamp) / (reload_value / 1000) * 1e-6;
    }

    float proportion = pid->Kp * Error; // P��

    float integration = pid->Last_Integration + pid->Ki * 0.5f * Ts * Error; // I��
    integration = _constrain(integration, -limit, limit);

    float differential = pid->Kd * (Error - pid->Last_Error) / Ts; // D��

    float Output = proportion + integration + differential;
    Output = _constrain(Output, -limit, limit);

    pid->Last_Error = Error;
    pid->Last_Integration = integration;
    pid->Last_Output = Output;
    pid->Timestamp_Last = Timestamp;

    return Output;
}