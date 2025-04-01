#ifndef AS5600_H
#define AS5600_H

#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include "i2c.h"

// AS5600 �������ṹ��
struct AS5600_Sensor {
    int Mot_num;           // ������
    float Angle;           // ��ǰ�Ƕ�
    float Last_Angle;      // ��һ�νǶ�
    int full_rotations;    // ������תȦ��
    float velocity;        // ��ǰ�ٶ�
    int Last_Vel_ts;       // ��һ���ٶ�ʱ���
    float Vel_Last_Angle;  // ��һ���ٶȽǶ�
};

// ��������
void AS5600_Write_Reg(uint16_t reg, uint8_t *value);
void AS5600_Write_Regs(uint16_t reg, uint8_t *value, uint8_t len);
void AS5600_Read_Reg(uint16_t reg, uint8_t *buf, uint8_t len);
float GetAngle_Without_Track(struct AS5600_Sensor *sensor);
float GetAngle(struct AS5600_Sensor *sensor);
float GetVelocity(struct AS5600_Sensor *sensor);

#endif // AS5600_H