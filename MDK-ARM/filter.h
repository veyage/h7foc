#ifndef FILTER_H
#define FILTER_H

#include "arm_math.h"

// �ⲿ��������
extern float reload_value;

// �������˲����ṹ��
typedef struct {
    float q; // ��������Э����
    float r; // ��������Э����
    float x; // ����ֵ
    float p; // �������Э����
    float k; // ����������
} KalmanFilter;

// ��ͨ�˲����ṹ��
typedef struct {
    float Tf;               // ʱ�䳣��
    float Last_y;           // ��һ�ε����ֵ
    uint32_t Last_Timesamp; // ��һ�ε�ʱ���
} LOWPASS;

// ��ͨ�˲�������
float Lowpassfilter(LOWPASS *lowpass, float x);


// �������˲�������
void KalmanFilter_Init(KalmanFilter *filter, float q, float r, float initial_value);
float KalmanFilter_Update(KalmanFilter *filter, float measurement);


#endif // FILTER_H