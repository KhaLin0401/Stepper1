#include "Encoder.h"
#include "ModbusMap.h"
#include "UartModbus.h"
#include "string.h"
#include "math.h"

// -----------------------------------------------------------------------------
// Encoder mechanical parameters (mặc định, có thể bị override từ Modbus)
// -----------------------------------------------------------------------------
#define ENCODER_PPR_DISK             12U                  // số khe trên đĩa (default)
#define ENCODER_GEAR_RATIO           2U                   // 2 vòng encoder -> 1 vòng bánh xe

// -----------------------------------------------------------------------------
// TIM counter state (External Clock mode)
// -----------------------------------------------------------------------------
static uint16_t last_counter = 0;
static uint32_t last_update_tick = 0;
static uint32_t pulse_accumulator = 0;

// -----------------------------------------------------------------------------
// Encoder instance
// -----------------------------------------------------------------------------
Encoder_t encoder = {
    .velocity = 0,
    .pulse_count = 0,
    .diameter_mm = 0,
    .revolutions = 0,
    .reset_flag = false
};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// PUBLIC: Khởi tạo encoder + TIM (External Clock)
// -----------------------------------------------------------------------------
void Encoder_Init(void)
{
    pulse_accumulator = 0;
    last_update_tick = HAL_GetTick();
    encoder.reset_flag = DEFAULT_ENCODER_RESET_FLAG;
    encoder.velocity = DEFAULT_ENCODER_VELOCITY;
    encoder.pulse_count = DEFAULT_ENCODER_PULSE_COUNT;
    encoder.diameter_mm = DEFAULT_ENCODER_DIAMETER;
    encoder.revolutions = DEFAULT_ENCODER_REVOLUTIONS;

    // TIM2 ở External Clock Mode: đếm xung trực tiếp trên CNT
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start(&htim2);
    last_counter = 0;
}

// -----------------------------------------------------------------------------
// INTERNAL: Tính tốc độ tuyến tính mm/s
// -----------------------------------------------------------------------------
static float Encoder_Compute_Speed_mm_s(uint32_t pps, float mm_per_pulse)
{
    return (float)pps * mm_per_pulse;
}

// -----------------------------------------------------------------------------
// PUBLIC: Hàm update gọi mỗi 1ms hoặc 10ms tùy scheduler
// -----------------------------------------------------------------------------
void Encoder_Update(void)
{
    Encoder_Load_Reset_Flag();
    if(encoder.reset_flag == 1)
    {
        Encoder_Reset();
    }
    Encoder_Load_Diameter();
    if(encoder.diameter_mm == 0)
    {
        encoder.diameter_mm = DEFAULT_ENCODER_DIAMETER;
    }
    Encoder_Load_Revolutions();
    if(encoder.revolutions == 0)
    {
        encoder.revolutions = DEFAULT_ENCODER_REVOLUTIONS;
    }

    // Đọc bộ đếm TIM2 (External Clock mode)
    uint16_t counter_now = __HAL_TIM_GET_COUNTER(&htim2);
    uint16_t new_pulses =
        (counter_now >= last_counter)
            ? (counter_now - last_counter)
            : ((uint16_t)(htim2.Init.Period + 1U) - last_counter + counter_now);
    last_counter = counter_now;

    // Dùng accumulator để tính pulse/s
    pulse_accumulator += new_pulses;

    uint32_t now = HAL_GetTick();

    // Mỗi 100 ms cập nhật tốc độ (quy đổi về mỗi giây)
    if ((now - last_update_tick) >= 500U)
    {
        uint32_t pulses_in_window = pulse_accumulator;
        uint32_t pulses_per_second = pulses_in_window * 2U; // 100 ms cửa sổ => x10 để ra pps

        // Dùng thông số runtime từ Modbus: đường kính và PPR
        uint32_t pulses_per_rev_cfg = encoder.revolutions * ENCODER_GEAR_RATIO;
        if (pulses_per_rev_cfg == 0U)
        {
            pulses_per_rev_cfg = DEFAULT_ENCODER_REVOLUTIONS * ENCODER_GEAR_RATIO;
        }

        float mm_per_pulse = 0.0f;
        if (pulses_per_rev_cfg > 0U)
        {
            // chu vi = pi * đường kính
            float wheel_circumference_mm = 3.14159265358979323846f * (float)encoder.diameter_mm;
            mm_per_pulse = wheel_circumference_mm / (float)pulses_per_rev_cfg;
        }

        // Pulse count (pps) giới hạn 16 bit để ghi vào holding register
        // Lưu số xung thực thu trong cửa sổ 500 ms (không nhân 10) để hiển thị đúng theo chu kỳ
        encoder.pulse_count = (pulses_in_window > 0xFFFFU)
                                  ? 0xFFFFU
                                  : (uint16_t)pulses_in_window;
        g_holdingRegisters[REG_FEEDBACK_PULSE_COUNT] = encoder.pulse_count;

        // Tính tốc độ mm/s từ pps
        float speed_mm_s = Encoder_Compute_Speed_mm_s(pulses_per_second, mm_per_pulse);
        encoder.velocity = (uint16_t)speed_mm_s;
        g_holdingRegisters[REG_FEEDBACK_VELOCITY] = encoder.velocity;

        pulse_accumulator = 0;
        last_update_tick = now;
    }
    Encoder_Save_Velocity();
    Encoder_Save_Pulse_Count();
    Encoder_Save_Reset_Flag();
}

// -----------------------------------------------------------------------------
// PUBLIC: Reset encoder
// -----------------------------------------------------------------------------
void Encoder_Reset(void)
{
    encoder.pulse_count = 0;
    encoder.revolutions = 0;
    encoder.velocity = 0;
    pulse_accumulator = 0;
}

// -----------------------------------------------------------------------------
// Setter APIs
// -----------------------------------------------------------------------------
void Encoder_Save_Velocity(void) 
{ 
    g_holdingRegisters[REG_FEEDBACK_VELOCITY] = encoder.velocity;
}
void Encoder_Save_Pulse_Count(void) 
{ 
    g_holdingRegisters[REG_FEEDBACK_PULSE_COUNT] = encoder.pulse_count;
}
void Encoder_Save_Reset_Flag(void) 
{ 
    g_holdingRegisters[REG_RESET_FLAG] = encoder.reset_flag;
}
void Encoder_Load_Diameter(void) 
{ 
    encoder.diameter_mm = g_holdingRegisters[REG_DIAMETER]; 
}
void Encoder_Load_Reset_Flag(void) 
{ 
    encoder.reset_flag = g_holdingRegisters[REG_RESET_FLAG];
}
void Encoder_Load_Revolutions(void) 
{ 
    encoder.revolutions = g_holdingRegisters[REG_REVOLUTIONS];
}
// Getter
void Encoder_Get_Velocity(uint16_t *velocity) { *velocity = encoder.velocity; }

