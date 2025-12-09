#ifndef __ENCODER_H__

#define __ENCODER_H__

#include "ModbusMap.h"
#include "stdint.h"
#include "main.h"
#include "UartModbus.h"
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

#define DMA_BUFFER_SIZE 100
#define ENCODER_PULSES_PER_REV 24U

struct Encoder_t {
    uint16_t velocity;
    uint16_t pulse_count;
    uint16_t diameter_mm;
    uint16_t revolutions;
    bool reset_flag;
};

typedef struct Encoder_t Encoder_t;

extern Encoder_t encoder;

void Encoder_Init(void);
void Encoder_Update(void);
void Encoder_Reset(void);
void Encoder_Save_Velocity(void);
void Encoder_Save_Pulse_Count(void);
void Encoder_Load_Diameter(void);
void Encoder_Load_Reset_Flag(void);
void Encoder_Load_Revolutions(void);
void Encoder_Save_Reset_Flag(void);
void Encoder_Get_Velocity(uint16_t *velocity);
#ifdef __cplusplus
}
#endif

#endif