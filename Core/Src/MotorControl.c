#include "MotorControl.h"
#include "main.h"
#include "ModbusMap.h"
#include "UartModbus.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>  // ✅ DEBUG: Thêm cho printf

// Khởi tạo

MotorRegisterMap_t motor1;
MotorRegisterMap_t motor2;
SystemRegisterMap_t system;

MotionState_t m1_motion_state;
MotionState_t m2_motion_state;

uint16_t mapRegisterAddress(uint16_t modbusAddress) {
    // System registers (0x0000-0x0006)
    if (modbusAddress <= 0x0006) {
        return modbusAddress;
    }
    // Motor 1 registers (0x0010-0x001D)
    else if (modbusAddress >= 0x0010 && modbusAddress <= 0x001D) {
        return modbusAddress;
    }
    // Motor 2 registers (0x0020-0x002D)
    else if (modbusAddress >= 0x0020 && modbusAddress <= 0x002D) {
        return modbusAddress;
    }
    // Digital Input registers (0x0030-0x0034)
    else if (modbusAddress >= 0x0030 && modbusAddress <= 0x0034) {
        return modbusAddress;
    }
    // Digital Output registers (0x0040-0x0044)
    else if (modbusAddress >= 0x0040 && modbusAddress <= 0x0044) {
        return modbusAddress;
    }
    
    return modbusAddress; // Return as is if not in any range
}
// Load từ modbus registers
void MotorRegisters_Load(MotorRegisterMap_t* motor, uint16_t base_addr) {
    motor->Control_Mode = g_holdingRegisters[base_addr + 0x00];
    motor->Enable = g_holdingRegisters[base_addr + 0x01];
    motor->Command_Speed = g_holdingRegisters[base_addr + 0x02];
    motor->Actual_Speed = g_holdingRegisters[base_addr + 0x03];
    motor->Direction = g_holdingRegisters[base_addr + 0x04];
    motor->Max_Speed = g_holdingRegisters[base_addr + 0x05];
    motor->Min_Speed = g_holdingRegisters[base_addr + 0x06];
    motor->Vmax = g_holdingRegisters[base_addr + 0x07];
    motor->Amax = g_holdingRegisters[base_addr + 0x08];
    motor->Jmax = g_holdingRegisters[base_addr + 0x09];
    motor->Max_Acc = g_holdingRegisters[base_addr + 0x0A];
    motor->Max_Dec = g_holdingRegisters[base_addr + 0x0B];
    motor->Status_Word = g_holdingRegisters[base_addr + 0x0C];
    motor->Error_Code = g_holdingRegisters[base_addr + 0x0D];
}

void SystemRegisters_Load(SystemRegisterMap_t* sys, uint16_t base_addr){
    sys->Device_ID = g_holdingRegisters[base_addr + 0];
    sys->Firmware_Version = g_holdingRegisters[base_addr + 1];
    sys->System_Status = g_holdingRegisters[base_addr + 2];
    sys->System_Error = g_holdingRegisters[base_addr + 3];
    sys->Reset_Error_Command = g_holdingRegisters[base_addr + 4];
    sys->Config_Baudrate = g_holdingRegisters[base_addr + 5];
    sys->Config_Parity = g_holdingRegisters[base_addr + 6];
}

// Save lại vào modbus registers
void MotorRegisters_Save(MotorRegisterMap_t* motor, uint16_t base_addr){
    g_holdingRegisters[base_addr + 0x00] = motor->Control_Mode;
    g_holdingRegisters[base_addr + 0x01] = motor->Enable;
    g_holdingRegisters[base_addr + 0x02] = motor->Command_Speed;
    g_holdingRegisters[base_addr + 0x03] = motor->Actual_Speed;
    g_holdingRegisters[base_addr + 0x04] = motor->Direction;
    g_holdingRegisters[base_addr + 0x05] = motor->Max_Speed;
    g_holdingRegisters[base_addr + 0x06] = motor->Min_Speed;
    g_holdingRegisters[base_addr + 0x07] = (uint8_t)(motor->Vmax);
    g_holdingRegisters[base_addr + 0x08] = (uint8_t)(motor->Amax);
    g_holdingRegisters[base_addr + 0x09] = (uint8_t)(motor->Jmax);
    g_holdingRegisters[base_addr + 0x0A] = motor->Max_Acc;
    g_holdingRegisters[base_addr + 0x0B] = motor->Max_Dec;
    g_holdingRegisters[base_addr + 0x0C] = motor->Status_Word;
    g_holdingRegisters[base_addr + 0x0D] = motor->Error_Code;
}
void SystemRegisters_Save(SystemRegisterMap_t* sys, uint16_t base_addr){
    g_holdingRegisters[base_addr + 0] = sys->Device_ID;
    g_holdingRegisters[base_addr + 1] = sys->Firmware_Version;
    g_holdingRegisters[base_addr + 2] = sys->System_Status;
    g_holdingRegisters[base_addr + 3] = sys->System_Error;
    g_holdingRegisters[base_addr + 4] = sys->Reset_Error_Command;
    g_holdingRegisters[base_addr + 5] = sys->Config_Baudrate;
    g_holdingRegisters[base_addr + 6] = sys->Config_Parity;
}

// Xử lý logic điều khiển motor
void Motor_ProcessControl(MotorRegisterMap_t* motor){
    if(motor->Enable == 1){
        HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, GPIO_PIN_SET);
        switch(motor->Control_Mode){
            case CONTROL_MODE_ONOFF:
                Motor_HandleOnOff(motor);
                break;
            case CONTROL_MODE_RAMP:
                Motor_HandleRamp(motor);
                break;
            default:
                break;
        }   
    }
    else if(motor->Enable == 0){
        HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, GPIO_PIN_RESET);
        motor->Status_Word = 0x0000;
        g_holdingRegisters[REG_M1_STATUS_WORD] = 0x0000;
        motor->Direction = IDLE;
        motor->Actual_Speed = 0; // Reset actual speed when disabled
        
        if(motor == &motor1) {
            Motor1_OutputPWM(motor, 0);           // Stop PWM with 0% duty
            Motor1_Set_Direction(IDLE);           // Set direction to IDLE
        } else {
            Motor2_OutputPWM(motor, 0);           // Stop PWM with 0% duty  
            Motor2_Set_Direction(IDLE);           // Set direction to IDLE
        }
    }
}



void Motor_Set_Mode(MotorRegisterMap_t* motor, uint8_t mode){
    motor->Control_Mode = mode;
}
void Motor_Set_Enable(MotorRegisterMap_t* motor){
    motor->Enable = 1;
}
void Motor_Set_Disable(MotorRegisterMap_t* motor){
    motor->Enable = 0;
    motor->Actual_Speed = 0; // Reset actual speed when disabled
}



void Motor1_Set_Direction(uint8_t direction){
    if(direction == IDLE){
        motor1.Direction = IDLE;
        HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_RESET);
        motor1.Actual_Speed = 0; // Reset actual speed when idle
        
        // ✅ CRITICAL FIX: STOP ALL PWM CHANNELS WHEN IDLE
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    }else if(direction == FORWARD){
        motor1.Direction = FORWARD;
        HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_RESET);
        // motor1.Actual_Speed = motor1.Command_Speed; // Set actual speed to command speed
    }else if(direction == REVERSE){
        motor1.Direction = REVERSE;
        HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_SET);
        // motor1.Actual_Speed = motor1.Command_Speed; // Set actual speed to command speed
    }
}
void Motor2_Set_Direction(uint8_t direction){
    if(direction == IDLE){
        motor2.Direction = IDLE;
        HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_RESET);
        motor2.Actual_Speed = 0; // Reset actual speed when idle
        
        // ✅ CRITICAL FIX: STOP ALL PWM CHANNELS WHEN IDLE
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    }else if(direction == FORWARD){
        motor2.Direction = FORWARD;
        HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_SET);
        // motor2.Actual_Speed = motor2.Command_Speed; // Set actual speed to command speed
    }else if(direction == REVERSE){
        motor2.Direction = REVERSE;
        HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_RESET);
        // motor2.Actual_Speed = motor2.Command_Speed; // Set actual speed to command speed
    }
}

void Motor_Set_Speed(MotorRegisterMap_t* motor, uint8_t speed){
    motor->Command_Speed = speed;
    if(motor->Enable && motor->Direction != IDLE) {
        motor->Actual_Speed = speed; // Update actual speed when enabled and not idle
    }
}


void Motor_Set_Vmax(MotorRegisterMap_t* motor, uint8_t vmax){
    motor->Vmax = vmax;
}
void Motor_Set_Amax(MotorRegisterMap_t* motor, uint8_t amax){
    motor->Amax = amax;
}
void Motor_Set_Jmax(MotorRegisterMap_t* motor, uint8_t jmax){
    motor->Jmax = jmax;
}

static uint32_t GetTimerInputClockHz(TIM_HandleTypeDef *htim)
{
    uint32_t pclk;
    uint32_t tim_clk;

    /* TIM1, TIM8 on APB2; others on APB1 (STM32F1 family) */
    if (htim->Instance == TIM1) {
        pclk = HAL_RCC_GetPCLK2Freq();
        /* If APB2 prescaler != 1, timer clock = PCLK2 * 2 */
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) 
            tim_clk = pclk * 2;
        else 
            tim_clk = pclk;
    } else {
        pclk = HAL_RCC_GetPCLK1Freq();
        /* If APB1 prescaler != 1, timer clock = PCLK1 * 2 */
        if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) 
            tim_clk = pclk * 2;
        else 
            tim_clk = pclk;
    }
    return tim_clk;
}


// Xử lý ON/OFF mode (mode 1)
uint8_t Motor_HandleOnOff(MotorRegisterMap_t* motor) {
    uint8_t duty = 0;
    uint8_t motor_id = (motor == &motor1) ? 1 : 2;
    
    if(motor->Enable == 1 && motor->Direction != IDLE) {
        motor->Status_Word = 0x0001;
        g_holdingRegisters[REG_M1_STATUS_WORD] = 0x0001;
        // Xuất PWM theo tốc độ đặt
        duty = motor->Command_Speed;
        motor->Actual_Speed = duty; // Update actual speed in ON/OFF mode
        
        // ✅ CRITICAL FIX: OUTPUT PWM WHEN ENABLED
        if(motor_id == 1) {
            Motor1_OutputPWM(motor, duty);
        } else {
            Motor2_OutputPWM(motor, duty);
        }
    } else {
        motor->Status_Word = 0x0000;
        g_holdingRegisters[REG_M1_STATUS_WORD] = 0x0000;
        motor->Direction = IDLE;
        motor->Actual_Speed = 0;
        duty = 0;
        
        // ✅ CRITICAL FIX: STOP PWM WHEN DISABLED OR IDLE
        if(motor_id == 1) {
            Motor1_OutputPWM(motor, 0);
            Motor1_Set_Direction(IDLE);
        } else {
            Motor2_OutputPWM(motor, 0);
            Motor2_Set_Direction(IDLE);
        }
    }
    return duty;
}


// Function to simulate actual speed measurement (replace with real encoder reading)
uint8_t getActualSpeed(uint8_t motor_id) {
    // For now, simulate speed based on PWM duty with some delay/filtering
    // In real implementation, this should read from encoder or current sensor
    static uint8_t simulated_speed1 = 0;
    static uint8_t simulated_speed2 = 0;
    
    if (motor_id == 1) {
        // Simple first-order filter to simulate motor response
        uint8_t target_speed = motor1.Command_Speed;
        if (simulated_speed1 < target_speed) {
            simulated_speed1 += (target_speed > simulated_speed1 + 2) ? 2 : (target_speed - simulated_speed1);
        } else if (simulated_speed1 > target_speed) {
            simulated_speed1 -= (simulated_speed1 > target_speed + 2) ? 2 : (simulated_speed1 - target_speed);
        }
        return simulated_speed1;
    } else {
        uint8_t target_speed = motor2.Command_Speed;
        if (simulated_speed2 < target_speed) {
            simulated_speed2 += (target_speed > simulated_speed2 + 2) ? 2 : (target_speed - simulated_speed2);
        } else if (simulated_speed2 > target_speed) {
            simulated_speed2 -= (simulated_speed2 > target_speed + 2) ? 2 : (simulated_speed2 - target_speed);
        }
        return simulated_speed2;
    }
}

// Xử lý PID mode (mode 3)
uint8_t Motor_HandleRamp(MotorRegisterMap_t* motor) {
    uint8_t motor_id = (motor == &motor1) ? 1 : 2;
    MotionState_t *motion_state = (motor_id == 1) ? &m1_motion_state : &m2_motion_state;

    TIM_HandleTypeDef *tim;
    uint32_t ch_forward, ch_reverse;

    /* Map timers/channels theo file gốc */
    if (motor_id == 1) {
        tim = &htim3;
        ch_forward = TIM_CHANNEL_1;   // Motor1 uses TIM3 CH1 in your code
        ch_reverse = TIM_CHANNEL_2;   // if you used CH2 elsewhere you can keep it
    } else {
        tim = &htim1;
        ch_forward = TIM_CHANNEL_1;   // Motor2 uses TIM1 CH1 (forward)
        ch_reverse = TIM_CHANNEL_3;   // Motor2 uses TIM1 CH3 (reverse)
    }

    /* ------------------ 1) Điều kiện dừng / disable ------------------ */
    if (motor->Enable == 0 || motor->Control_Mode != CONTROL_MODE_RAMP || motor->Direction == IDLE) {
        /* reset motion state (khởi tạo an toàn) */
        MotionState_Init(motor_id);

        /* dừng PWM output cho timer tương ứng */
        if (motor_id == 1) {
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        } else {
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        }

        /* đảm bảo giá trị báo UI = 0 */
        motor->Actual_Speed = 0;
        return motor->Actual_Speed;
    }

    /* ------------------ 2) Tính v_target từ Command_Speed (0..100) ------------------ */
    /* Ánh xạ percent -> vận tốc (steps/s). Command_Speed là 0..100 (%). */
    motion_state->v_target = DEFAULT_VMIN + (motor->Vmax - DEFAULT_VMIN) * (motor->Command_Speed / 100.0f);

    /* ------------------ 3) Cập nhật jerk -> acceleration ------------------ */
    float dv = motion_state->v_target - motion_state->v_actual;
    if (fabsf(dv) < 1e-6f) {
        motion_state->j = 0.0f;
    } else if (dv > 0.0f) {
        motion_state->j = motor->Jmax * 100.0f;      /* tăng tốc */
    } else {
        motion_state->j = - (float)motor->Jmax * 100.0f; /* giảm tốc */
    }

    motion_state->a += motion_state->j * motion_state->dt;

    /* Giới hạn gia tốc */
    if (motion_state->a > motor->Amax * 100.0f) motion_state->a = motor->Amax * 100.0f;
    if (motion_state->a < -motor->Amax * 100.0f) motion_state->a = -motor->Amax * 100.0f;

    /* ------------------ 4) Cập nhật vận tốc (v_actual) ------------------ */
    motion_state->v_actual += motion_state->a * motion_state->dt;

    // Giới hạn trong [0, v_target]
    if (motion_state->v_actual > motion_state->v_target)
        motion_state->v_actual = motion_state->v_target;
    if (motion_state->v_actual < 0.0f)
        motion_state->v_actual = 0.0f;

    /* ------------------ 5) Cập nhật vị trí tích lũy ------------------ */
    motion_state->pos += motion_state->v_actual * motion_state->dt;

    /* ------------------ 6) Cập nhật giá trị tốc độ Actual_Speed (%) cho UI ------------------ */
    float percent_speed = 0.0f;
    if (motion_state->v_target > 0.0f) {
        // Tỷ lệ so với v_target, scale lên Command_Speed (%)
        percent_speed = (motion_state->v_actual / motion_state->v_target) * motor->Command_Speed;
    } else {
        percent_speed = 0.0f;
    }
    if (percent_speed < 0.0f) percent_speed = 0.0f;
    if (percent_speed > 100.0f) percent_speed = 100.0f;
    motor->Actual_Speed = (uint8_t)(percent_speed + 0.5f); // rounded

    /* Nếu tốc độ rất thấp (<= Vmin) thì tạm dừng PWM (không phát xung) */
    if (motion_state->v_actual <= DEFAULT_VMIN) {
        if (motor_id == 1) {
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        } else {
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        }
        return motor->Actual_Speed;
    }

    /* ------------------ 7) Tính PSC/ARR tối ưu và cập nhật timer (tần số step) ------------------ */
    float f_step = motion_state->v_actual; // step/s (Hz)
    if (f_step <= 0.0f) return motor->Actual_Speed;

    /* Lấy timer input clock (PCLKx với hệ số x2 nếu APB prescaler != 1) */
    uint32_t timer_clk = GetTimerInputClockHz(tim);

    /* Tính số tick mong muốn cho một chu kỳ: desired_counts = timer_clk / f_step */
    float desired_counts_f = (float)timer_clk / f_step; // > 0.0

    /* Giới hạn ARR max (16-bit timer trên F1) */
    const uint32_t MAX_ARR = 0xFFFFu;

    /* Tìm PSC hợp lý để ARR <= MAX_ARR.
       Đề xuất PSC = floor(desired_counts / (MAX_ARR+1)) nếu cần. */
    uint32_t psc_new = __HAL_TIM_GET_PRESCALER(tim);
    if (desired_counts_f > (float)(MAX_ARR + 1)) {
        uint32_t psc_candidate = (uint32_t)(desired_counts_f / (float)(MAX_ARR + 1));
        if (psc_candidate > 0xFFFFu) psc_candidate = 0xFFFFu;
        psc_new = psc_candidate;
    }

    /* Tính ARR từ PSC mới */
    uint32_t arr_new = 0;
    if (psc_new >= 0xFFFFFFFFu) {
        arr_new = MAX_ARR;
    } else {
        arr_new = (uint32_t)(desired_counts_f / (float)(psc_new + 1));
        if (arr_new > 0) arr_new = arr_new - 1;
        if (arr_new > MAX_ARR) arr_new = MAX_ARR;
    }

    /* Áp thay đổi PSC/ARR một cách an toàn: disable timer, cập nhật, UG, enable */
    __HAL_TIM_DISABLE(tim);
    __HAL_TIM_SET_PRESCALER(tim, psc_new);
    __HAL_TIM_SET_AUTORELOAD(tim, arr_new);
    /* pulse width: 50% duty (suitable for step pulse). Ensure at least 1 tick */
    uint32_t compare = arr_new / 2;
    if (compare < 1) compare = 1;

    /* Chọn channel phù hợp theo chiều (vì motor2 dùng CH1/CH3 để chọn chiều) */
    if (motor_id == 1) {
        /* Motor1 uses TIM3 CH1 as STEP output (consistent with Motor1_OutputPWM) */
        __HAL_TIM_SET_COMPARE(&htim3, ch_forward, compare);
        HAL_TIM_PWM_Start(&htim3, ch_forward);
    } else {
        /* Motor2: start channel according to direction, stop the other */
        if (motor->Direction == FORWARD) {
            HAL_TIM_PWM_Stop(&htim1, ch_reverse);
            __HAL_TIM_SET_COMPARE(&htim1, ch_forward, compare);
            HAL_TIM_PWM_Start(&htim1, ch_forward);
        } else { /* REVERSE */
            HAL_TIM_PWM_Stop(&htim1, ch_forward);
            __HAL_TIM_SET_COMPARE(&htim1, ch_reverse, compare);
            HAL_TIM_PWM_Start(&htim1, ch_reverse);
        }
    }

    /* generate update so shadow registers load immediately */
    tim->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_ENABLE(tim);

    /* (Tùy chọn) cập nhật holding register để Modbus/ UI đọc ngay */
    if (motor_id == 1) {
        g_holdingRegisters[REG_M1_ACTUAL_SPEED] = motor->Actual_Speed;
        g_holdingRegisters[REG_M1_STATUS_WORD]  = motor->Status_Word;
    } else {
        g_holdingRegisters[REG_M2_ACTUAL_SPEED] = motor->Actual_Speed;
        g_holdingRegisters[REG_M2_STATUS_WORD]  = motor->Status_Word;
    }

    return motor->Actual_Speed;
}

void MotionState_Init(uint8_t motor_id){
    MotionState_t* motor = (motor_id == 1) ? &m1_motion_state : &m2_motion_state;
    motor->v_target = DEFAULT_VMIN;
    motor->a = 0.0f;
    motor->j = 0.0f;
    motor->pos = 0.0f;
    motor->Distance = 0.0f;
    motor->dt = 0.001f;
    motor->v_actual = 0.0f;
}

// Gửi tín hiệu PWM dựa vào Actual_Speed
void Motor1_OutputPWM(MotorRegisterMap_t* motor, uint8_t duty_percent){
    // Chuyển % thành giá trị phù hợp với Timer (0 - TIM_ARR)
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t ccr = duty_percent * arr / 100; // ✅ FIX: Sử dụng duty_percent thay vì hardcode 50%
    
    // ✅ DEBUG: In thông tin PWM
    printf("Motor1 PWM: duty=%d%%, arr=%lu, ccr=%lu, dir=%d\n", 
           duty_percent, arr, ccr, motor->Direction);
    
    // ✅ FIX: Kiểm tra điều kiện phát xung
    if(duty_percent == 0) {
        // Dừng PWM khi duty = 0
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        printf("Motor1 PWM: STOPPED (duty=0)\n");
        return;
    }
    
    // ✅ FIX: Phát xung theo chiều quay
    if(motor->Direction == FORWARD){
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
        printf("Motor1 PWM: FORWARD CH1, ccr=%lu\n", ccr);
    }else if(motor->Direction == REVERSE){
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr);
        printf("Motor1 PWM: REVERSE CH2, ccr=%lu\n", ccr);
    }else{
        // IDLE - dừng tất cả PWM
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        printf("Motor1 PWM: IDLE - STOPPED\n");
    }
}

void Motor2_OutputPWM(MotorRegisterMap_t* motor, uint8_t duty_percent){
    // Chuyển % thành giá trị phù hợp với Timer (0 - TIM_ARR)
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t ccr = duty_percent * arr / 100;
    
    if(motor->Direction == FORWARD){
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
    }else if(motor->Direction == REVERSE){
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr);
    }

    
}

// Điều khiển chiều quay motor


// Khởi tạo giá trị PID cho từng motor
// void PID_Init(MotorRegisterMap_t* motor, float kp, float ki, float kd){

// Khởi tạo giá trị PID cho từng motor

// Tính toán PID mỗi chu kỳ - trả về duty % (0-100)

// Reset các lỗi nếu có
void Motor_ResetError(MotorRegisterMap_t* motor){

}

// Kiểm tra và xử lý các điều kiện lỗi (overcurrent, timeout,...)
void Motor_CheckError(MotorRegisterMap_t* motor){

}

// MODIFICATION LOG
// Date: 2025-01-26
// Changed by: AI Agent
// Description: Added debug functions to monitor PID values and motor status
// Reason: Need to troubleshoot PID performance and verify calculations
// Impact: Can now monitor PID terms and motor response in real-time
// Testing: Use debug output to verify PID is working correctly

// Debug/log
void Motor_DebugPrint(const MotorRegisterMap_t* motor, const char* name){

}
void System_DebugPrint(const SystemRegisterMap_t* sys){
    // This can be implemented to output system status via UART if needed
}
