#include "filter.h"
#include "stm32h7xx_hal.h"

/////////////////�˲�

extern float reload_value;


KalmanFilter kalman_filter;

// ʹ�� LOWPASS �ṹ��ĵ�ͨ�˲���ʵ��
float Lowpassfilter( LOWPASS *lowpass, float x) {
    float dt = 0.0;

    uint32_t Timesamp = SysTick->VAL;
    if (Timesamp < lowpass->Last_Timesamp) {
        dt = (float)(lowpass->Last_Timesamp - Timesamp) / (reload_value / 1000) * 1e-3f;
    } else {
        dt = (float)(reload_value - Timesamp + lowpass->Last_Timesamp) / (reload_value / 1000) * 1e-3f;
    }

    if (dt <= 0.0f || dt > 0.005f) {
        lowpass->Last_y = x;
        lowpass->Last_Timesamp = Timesamp;
        return x;
    }

    float alpha = lowpass->Tf / (lowpass->Tf + dt);
    float y = alpha * lowpass->Last_y + (1.0f - alpha) * x;

    lowpass->Last_y = y;
    lowpass->Last_Timesamp = Timesamp;

    return y;
}



// �������˲�����ʼ��
void KalmanFilter_Init(KalmanFilter *filter, float q, float r, float initial_value) {
    filter->q = q;
    filter->r = r;
    filter->x = initial_value;
    filter->p = 1.0f;
    filter->k = 0.0f;
}

// �������˲�������
float KalmanFilter_Update(KalmanFilter *filter, float measurement) {
    // Ԥ�����
    filter->p = filter->p + filter->q;

    // ���㿨��������
    filter->k = filter->p / (filter->p + filter->r);

    // ���¹���ֵ
    filter->x = filter->x + filter->k * (measurement - filter->x);

    // ���¹������Э����
    filter->p = (1.0f - filter->k) * filter->p;

    return filter->x;
}


