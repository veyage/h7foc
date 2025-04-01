#ifndef CURRENT_H
#define CURRENT_H

#include "stm32h7xx_hal.h"
#include "adc.h"
// �����������ṹ��
struct Current_Sensor {
    int Sen_Num;          // ���������
    float I_a;            // ����� A
    float I_b;            // ����� B
    float offset_ia;      // ��Ʈƫ�� A
    float offset_ib;      // ��Ʈƫ�� B
    float current_a;      // ��ǰ���� A
    float current_b;      // ��ǰ���� B
    float current_c;      // ��ǰ���� C
};

// �ⲿ��������
extern uint16_t Samp_volts[4];

// ��������
void Set_Cur_Sensor(int Mot_num);
void DriftOffsets(struct Current_Sensor *Sensor);
void CurrSense_Init(struct Current_Sensor *Sensor);
void GetPhaseCurrent(struct Current_Sensor *Sensor);

#endif // CURRENT_H


