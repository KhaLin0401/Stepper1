# Tài liệu Thuật toán S-Curve cho Điều khiển Động cơ Stepper

## 1. Tổng quan

Thuật toán S-Curve được sử dụng trong chế độ điều khiển **RAMP** (`CONTROL_MODE_RAMP = 2`) để tạo ra quỹ đạo chuyển động mượt mà cho động cơ stepper. Khác với gia tốc/giảm tốc tuyến tính (trapezoidal), S-Curve điều khiển cả **jerk** (tốc độ thay đổi gia tốc), giúp giảm rung động cơ học và tăng độ chính xác định vị.

### 1.1. Ưu điểm của S-Curve so với Trapezoidal

| Đặc điểm | Trapezoidal | S-Curve |
|----------|-------------|---------|
| Gia tốc | Thay đổi đột ngột | Thay đổi mượt mà |
| Rung động | Cao | Thấp |
| Độ chính xác | Trung bình | Cao |
| Độ phức tạp | Đơn giản | Phức tạp hơn |
| Mài mòn cơ khí | Cao hơn | Thấp hơn |

### 1.2. Biểu đồ Profile S-Curve

```
Velocity (v)
    ^
    |           ___________
    |          /           \
    |         /             \
    |        /               \
    |       /                 \
    |______/                   \______
    +---------------------------------> Time (t)
    
Acceleration (a)
    ^
    |      ____
    |     /    \
    |    /      \
    +---/--------\---------/----\---> Time (t)
    |             \       /      \
    |              \_____/        \____
    
Jerk (j)
    ^
    |  __        __
    | |  |      |  |
    +-|--|------|--|---------|--|---> Time (t)
    |      |__|        |__|
    |
```

## 2. Các thông số điều khiển

### 2.1. Thông số đầu vào (từ Modbus Registers)

| Thông số | Ký hiệu | Đơn vị | Phạm vi | Mô tả |
|----------|---------|--------|---------|-------|
| **Command_Speed** | `motor->Command_Speed` | % | 0-100 | Tốc độ mục tiêu từ người dùng (phần trăm) |
| **Vmax** | `motor->Vmax` | ×100 steps/s | 1-255 | Tốc độ tối đa cho phép |
| **Amax** | `motor->Amax` | ×100 steps/s² | 1-255 | Gia tốc tối đa cho phép |
| **Jmax** | `motor->Jmax` | ×100 steps/s³ | 1-255 | Jerk tối đa cho phép |

### 2.2. Giá trị mặc định (ModbusMap.h)

```c
#define DEFAULT_VMAX               20      // 20 × 100 = 2000 steps/s
#define DEFAULT_AMAX               5       // 5 × 100 = 500 steps/s²
#define DEFAULT_JMAX               2       // 2 × 100 = 200 steps/s³
#define DEFAULT_VMIN               400     // 400 steps/s (tốc độ tối thiểu)
```

### 2.3. Cấu trúc trạng thái chuyển động (MotionState_t)

```c
typedef struct {
    float v_target;    // Vận tốc mục tiêu (steps/s)
    float v_actual;    // Vận tốc thực tế hiện tại (steps/s)
    float a;           // Gia tốc hiện tại (steps/s²)
    float j;           // Jerk hiện tại (steps/s³)
    float pos;         // Vị trí hiện tại (steps)
    float Distance;    // Tổng quãng đường (steps)
    float dt;          // Chu kỳ lấy mẫu (mặc định 0.008s = 8ms)
} MotionState_t;
```

## 3. Thuật toán chi tiết

### 3.1. Sơ đồ khối thuật toán

```
┌─────────────────────────────────────────────────────────────────┐
│                    Motor_HandleRamp()                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ BƯỚC 1: Tính v_target từ Command_Speed (0-100%)          │   │
│  │ v_target = VMIN + (Vmax×100 - VMIN) × (Command_Speed/100)│   │
│  └──────────────────────────────────────────────────────────┘   │
│                           │                                     │
│                           ▼                                     │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ BƯỚC 2: Tính độ lệch vận tốc                             │   │
│  │ dv = v_target - v_actual                                 │   │
│  └──────────────────────────────────────────────────────────┘   │
│                           │                                     │
│                           ▼                                     │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ BƯỚC 3: Xác định jerk dựa trên dv                        │   │
│  │ if |dv| < ε  → j = 0                                     │   │
│  │ if dv > 0    → j = +Jmax×100 (tăng tốc)                  │   │
│  │ if dv < 0    → j = -Jmax×100 (giảm tốc)                  │   │
│  └──────────────────────────────────────────────────────────┘   │
│                           │                                     │
│                           ▼                                     │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ BƯỚC 4: Cập nhật gia tốc                                 │   │
│  │ a = a + j × dt                                           │   │
│  │ Giới hạn: -Amax×100 ≤ a ≤ +Amax×100                      │   │
│  └──────────────────────────────────────────────────────────┘   │
│                           │                                     │
│                           ▼                                     │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ BƯỚC 5: Cập nhật vận tốc                                 │   │
│  │ v_actual = v_actual + a × dt                             │   │
│  │ Giới hạn: VMIN ≤ v_actual ≤ Vmax×100                     │   │
│  └──────────────────────────────────────────────────────────┘   │
│                           │                                     │
│                           ▼                                     │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ BƯỚC 6: Soft clamp - Đạt mục tiêu                        │   │
│  │ if (a<0 && v_actual≤v_target) hoặc                       │   │
│  │    (a>0 && v_actual≥v_target):                           │   │
│  │    v_actual = v_target; a = 0; j = 0                     │   │
│  └──────────────────────────────────────────────────────────┘   │
│                           │                                     │
│                           ▼                                     │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ BƯỚC 7: Tính Actual_Speed (%) cho UI                     │   │
│  │ percent = (v_actual - VMIN) / (Vmax×100 - VMIN) × 100    │   │
│  └──────────────────────────────────────────────────────────┘   │
│                           │                                     │
│                           ▼                                     │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ BƯỚC 8: Xuất tần số PWM                                  │   │
│  │ Stepper_OutputFreq(htim, channel, v_actual)              │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2. Code Implementation

```c
uint8_t Motor_HandleRamp(MotorRegisterMap_t* motor) {
    uint8_t motor_id = (motor == &motor1) ? 1 : 2;
    MotionState_t *motion_state = (motor_id == 1) ? &m1_motion_state : &m2_motion_state;

    /* BƯỚC 1: Tính v_target từ Command_Speed (0..100%) */
    motion_state->v_target = DEFAULT_VMIN + 
        (motor->Vmax*100 - DEFAULT_VMIN) * (motor->Command_Speed / 100.0f);

    /* BƯỚC 2-3: Xác định jerk dựa trên độ lệch vận tốc */
    float dv = motion_state->v_target - motion_state->v_actual;
    if (fabsf(dv) < 1e-6f) {
        motion_state->j = 0.0f;
    } else if (dv > 0.0f) {
        motion_state->j = motor->Jmax * 100.0f;      // Tăng tốc
    } else {
        motion_state->j = -motor->Jmax * 100.0f;     // Giảm tốc
    }

    /* BƯỚC 4: Cập nhật gia tốc với giới hạn */
    motion_state->a += motion_state->j * motion_state->dt;
    if (motion_state->a > motor->Amax * 100.0f) 
        motion_state->a = motor->Amax * 100.0f;
    if (motion_state->a < -motor->Amax * 100.0f) 
        motion_state->a = -motor->Amax * 100.0f;

    /* BƯỚC 5: Cập nhật vận tốc với giới hạn */
    motion_state->v_actual += motion_state->a * motion_state->dt;
    if (motion_state->v_actual > motor->Vmax*100)
        motion_state->v_actual = motor->Vmax*100;
    if (motion_state->v_actual < DEFAULT_VMIN)
        motion_state->v_actual = DEFAULT_VMIN;

    /* BƯỚC 6: Soft clamp khi đạt mục tiêu */
    if ((motion_state->a < 0) && (motion_state->v_actual <= motion_state->v_target)) {
        motion_state->v_actual = motion_state->v_target;
        motion_state->a = 0.0f;
        motion_state->j = 0.0f;
    }
    if ((motion_state->a > 0) && (motion_state->v_actual >= motion_state->v_target)) {
        motion_state->v_actual = motion_state->v_target;
        motion_state->a = 0.0f;
        motion_state->j = 0.0f;
    }

    /* BƯỚC 7: Tính phần trăm tốc độ cho UI */
    float percent_speed = (motion_state->v_actual - DEFAULT_VMIN) / 
                          (motor->Vmax*100 - DEFAULT_VMIN) * 100.0f;
    motor->Actual_Speed = (uint8_t)(percent_speed);

    /* BƯỚC 8: Xuất tần số PWM */
    Stepper_OutputFreq(&htim3, TIM_CHANNEL_1, motion_state->v_actual);
    
    return motor->Actual_Speed;
}
```

## 4. Phân tích tác động của các thông số

### 4.1. Ảnh hưởng của Vmax (Tốc độ tối đa)

**Công thức liên quan:**
```
v_target = VMIN + (Vmax×100 - VMIN) × (Command_Speed / 100)
```

| Vmax | Vmax thực (steps/s) | Phạm vi v_target | Ảnh hưởng |
|------|---------------------|------------------|-----------|
| 10 | 1000 | 400 - 1000 | Tốc độ thấp, phù hợp tải nặng |
| 20 | 2000 | 400 - 2000 | Cân bằng (mặc định) |
| 50 | 5000 | 400 - 5000 | Tốc độ cao, cần motor tốt |

**Tác động:**
- **Vmax cao**: Cho phép đạt tốc độ lớn hơn, nhưng cần motor có moment xoắn đủ lớn
- **Vmax thấp**: Hạn chế tốc độ tối đa, an toàn hơn cho tải nặng
- **Độ phân giải tốc độ**: `(Vmax×100 - VMIN) / 100` steps/s cho mỗi 1% Command_Speed

**Ví dụ với Vmax = 20:**
```
Command_Speed = 0%   → v_target = 400 steps/s
Command_Speed = 50%  → v_target = 400 + (2000-400)×0.5 = 1200 steps/s
Command_Speed = 100% → v_target = 2000 steps/s
```

### 4.2. Ảnh hưởng của Amax (Gia tốc tối đa)

**Công thức liên quan:**
```
a = a + j × dt
Giới hạn: -Amax×100 ≤ a ≤ +Amax×100
```

| Amax | Amax thực (steps/s²) | Thời gian tăng tốc* | Ảnh hưởng |
|------|----------------------|---------------------|-----------|
| 2 | 200 | ~8s | Rất mượt, chậm |
| 5 | 500 | ~3.2s | Cân bằng (mặc định) |
| 10 | 1000 | ~1.6s | Nhanh, có thể rung |
| 20 | 2000 | ~0.8s | Rất nhanh, rung nhiều |

*Thời gian ước tính để tăng từ 400 → 2000 steps/s (với Jmax = 2)

**Tác động:**
- **Amax cao**: 
  - ✅ Đáp ứng nhanh hơn
  - ❌ Tăng rung động cơ học
  - ❌ Có thể gây mất bước (step loss)
  - ❌ Tăng dòng điện đỉnh

- **Amax thấp**:
  - ✅ Chuyển động mượt mà
  - ✅ Giảm mài mòn cơ khí
  - ❌ Đáp ứng chậm

**Biểu đồ ảnh hưởng Amax:**
```
Velocity
    ^
    |     Amax cao
    |    /
    |   / ← Amax trung bình
    |  /
    | / ← Amax thấp
    |/___________________________> Time
```

### 4.3. Ảnh hưởng của Jmax (Jerk tối đa)

**Công thức liên quan:**
```
if (dv > 0) j = +Jmax×100    // Tăng tốc
if (dv < 0) j = -Jmax×100    // Giảm tốc

a = a + j × dt
```

| Jmax | Jmax thực (steps/s³) | Thời gian đạt Amax* | Đặc điểm |
|------|----------------------|---------------------|----------|
| 1 | 100 | 40ms | Rất mượt, profile S rõ |
| 2 | 200 | 20ms | Cân bằng (mặc định) |
| 5 | 500 | 8ms | Nhanh, gần trapezoidal |
| 10 | 1000 | 4ms | Rất nhanh, như trapezoidal |

*Với Amax = 5 (500 steps/s²), dt = 8ms

**Tác động:**
- **Jmax cao**:
  - ✅ Đáp ứng nhanh
  - ❌ Profile gần giống trapezoidal
  - ❌ Tăng shock cơ học
  - ❌ Giảm lợi ích của S-curve

- **Jmax thấp**:
  - ✅ Profile S-curve rõ ràng
  - ✅ Giảm rung động tối đa
  - ✅ Tăng tuổi thọ cơ khí
  - ❌ Đáp ứng chậm hơn

**So sánh profile với các giá trị Jmax khác nhau:**
```
Acceleration
    ^
    |   Jmax cao (gần step function)
    |  ┌────────┐
    |  │        │
    |  │        │
    +──┴────────┴──────> Time

    ^
    |   Jmax thấp (S-curve mượt)
    |     ____
    |    /    \
    |   /      \
    +──/────────\──────> Time
```

### 4.4. Ảnh hưởng của dt (Chu kỳ lấy mẫu)

**Giá trị mặc định:** `dt = 0.008s (8ms)`

**Công thức liên quan:**
```
a = a + j × dt
v_actual = v_actual + a × dt
```

| dt (ms) | Tần số cập nhật | Độ mượt | CPU load |
|---------|-----------------|---------|----------|
| 1 | 1000 Hz | Rất mượt | Cao |
| 8 | 125 Hz | Tốt (mặc định) | Trung bình |
| 20 | 50 Hz | Có thể thấy bậc | Thấp |

**Tác động:**
- **dt nhỏ**: Cập nhật thường xuyên hơn, profile mượt hơn, nhưng tăng tải CPU
- **dt lớn**: Giảm tải CPU nhưng profile có thể bị "bậc thang"

### 4.5. Bảng tổng hợp tác động các thông số

| Thông số | Tăng giá trị | Giảm giá trị |
|----------|--------------|--------------|
| **Vmax** | ↑ Tốc độ tối đa, ↑ Rủi ro mất bước | ↓ Tốc độ, ↑ An toàn |
| **Amax** | ↑ Đáp ứng nhanh, ↑ Rung động | ↓ Rung động, ↓ Đáp ứng |
| **Jmax** | ↑ Đáp ứng, ↓ Hiệu quả S-curve | ↑ Mượt mà, ↓ Đáp ứng |
| **dt** | ↓ Độ mượt, ↓ CPU load | ↑ Độ mượt, ↑ CPU load |

## 5. Công thức tính toán quan trọng

### 5.1. Ánh xạ Command_Speed → v_target

```c
v_target = VMIN + (Vmax×100 - VMIN) × (Command_Speed / 100.0f)
```

Với giá trị mặc định:
```
v_target = 400 + (2000 - 400) × (Command_Speed / 100)
v_target = 400 + 16 × Command_Speed  [steps/s]
```

### 5.2. Thời gian đạt gia tốc tối đa

```
t_amax = Amax×100 / (Jmax×100) = Amax / Jmax  [giây]
```

Với giá trị mặc định (Amax=5, Jmax=2):
```
t_amax = 5 / 2 = 2.5s (lý thuyết)
t_amax_discrete = (Amax×100) / (Jmax×100 × dt) × dt = Amax / (Jmax) [giây]
```

### 5.3. Thời gian tăng tốc từ v1 → v2 (ước tính)

Với S-curve đầy đủ 7 pha, thời gian tăng tốc:
```
Δv = v2 - v1
t_total ≈ Δv/Amax + Amax/Jmax  [giây, đơn vị gốc]
```

### 5.4. Ánh xạ v_actual → Actual_Speed (%)

```c
percent_speed = (v_actual - VMIN) / (Vmax×100 - VMIN) × 100.0f
```

### 5.5. Tính tần số PWM từ v_actual

```c
// v_actual = tần số step (Hz)
f_step = v_actual;  // steps/s = Hz

// Timer calculation
desired_counts = timer_clk / f_step;
PSC = desired_counts / 65536;  // Prescaler
ARR = desired_counts / (PSC + 1) - 1;  // Auto-reload
CCR = (ARR + 1) / 2;  // 50% duty cycle
```

## 6. Ví dụ mô phỏng

### 6.1. Tăng tốc từ 0% → 100%

**Điều kiện:**
- Vmax = 20 (2000 steps/s)
- Amax = 5 (500 steps/s²)
- Jmax = 2 (200 steps/s³)
- dt = 8ms
- Command_Speed: 0% → 100%

**Quá trình (đơn giản hóa):**

| Bước | t (ms) | j (steps/s³) | a (steps/s²) | v (steps/s) | Ghi chú |
|------|--------|--------------|--------------|-------------|---------|
| 0 | 0 | 0 | 0 | 400 | Khởi đầu |
| 1 | 8 | +200 | +1.6 | 400.01 | Bắt đầu tăng |
| 2 | 16 | +200 | +3.2 | 400.04 | j tích lũy a |
| ... | ... | ... | ... | ... | ... |
| n | t_n | +200 | 500 | v_n | a đạt Amax |
| ... | ... | 0 | 500 | v_n | a giữ nguyên |
| m | t_m | -200 | 0 | 2000 | Đạt v_target |

### 6.2. Giảm tốc từ 100% → 50%

**Quá trình:**
1. j = -Jmax×100 = -200 steps/s³
2. a giảm dần từ 0 → -500 steps/s²
3. v giảm dần từ 2000 → 1200 steps/s
4. Khi v ≤ v_target: soft clamp, reset a và j về 0

## 7. Khuyến nghị cấu hình

### 7.1. Ứng dụng tốc độ cao, tải nhẹ
```
Vmax = 50    (5000 steps/s)
Amax = 10    (1000 steps/s²)
Jmax = 5     (500 steps/s³)
```

### 7.2. Ứng dụng chính xác, tải trung bình (mặc định)
```
Vmax = 20    (2000 steps/s)
Amax = 5     (500 steps/s²)
Jmax = 2     (200 steps/s³)
```

### 7.3. Ứng dụng tải nặng, cần mượt mà
```
Vmax = 10    (1000 steps/s)
Amax = 2     (200 steps/s²)
Jmax = 1     (100 steps/s³)
```

### 7.4. Quy tắc chung khi điều chỉnh

1. **Bắt đầu với giá trị thấp**, tăng dần đến khi đạt hiệu suất mong muốn
2. **Tỷ lệ Amax/Jmax** nên trong khoảng 2-5 để có profile S-curve tốt
3. **Kiểm tra mất bước** khi tăng Vmax hoặc Amax
4. **Nghe tiếng motor**: Tiếng ồn cao = cần giảm Jmax hoặc Amax

## 8. Giới hạn và lưu ý

### 8.1. Giới hạn phần cứng

- **Tần số PWM tối thiểu**: 400 Hz (DEFAULT_VMIN)
- **Tần số PWM tối đa**: 50000 Hz (giới hạn trong Stepper_OutputFreq)
- **Độ phân giải timer**: 16-bit (PSC và ARR max = 65535)

### 8.2. Lưu ý khi sử dụng

1. **Không thay đổi thông số khi motor đang chạy** - có thể gây mất bước
2. **Đảm bảo nguồn cấp đủ dòng** khi tăng Amax
3. **Kiểm tra nhiệt độ motor** khi chạy tốc độ cao liên tục
4. **Soft clamp** giúp tránh overshoot nhưng có thể gây "giật" nhẹ khi đạt mục tiêu

### 8.3. Hạn chế của implementation hiện tại

1. **Không có position control** - chỉ điều khiển velocity
2. **Không có look-ahead** - không tính trước quỹ đạo
3. **Jerk cố định** - không có phase 7 đầy đủ của S-curve chuẩn
4. **Không có feedforward** - chỉ dựa vào feedback loop đơn giản

## 9. Sơ đồ quan hệ các thông số

```
┌─────────────────────────────────────────────────────────────────────┐
│                         USER INPUT                                  │
│                                                                     │
│   Command_Speed (0-100%)                                            │
│         │                                                           │
│         ▼                                                           │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │              v_target = f(Command_Speed, Vmax)              │   │
│   └─────────────────────────────────────────────────────────────┘   │
│         │                                                           │
│         │  dv = v_target - v_actual                                 │
│         ▼                                                           │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │                    JERK CONTROLLER                          │   │
│   │         j = sign(dv) × Jmax × 100                           │   │
│   └─────────────────────────────────────────────────────────────┘   │
│         │                                                           │
│         │  a = a + j × dt                                           │
│         ▼                                                           │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │              ACCELERATION LIMITER                           │   │
│   │         -Amax×100 ≤ a ≤ +Amax×100                           │   │
│   └─────────────────────────────────────────────────────────────┘   │
│         │                                                           │
│         │  v_actual = v_actual + a × dt                             │
│         ▼                                                           │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │               VELOCITY LIMITER                              │   │
│   │         VMIN ≤ v_actual ≤ Vmax×100                          │   │
│   └─────────────────────────────────────────────────────────────┘   │
│         │                                                           │
│         ▼                                                           │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │              PWM FREQUENCY OUTPUT                           │   │
│   │         f_step = v_actual (Hz)                              │   │
│   └─────────────────────────────────────────────────────────────┘   │
│         │                                                           │
│         ▼                                                           │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │                  STEPPER MOTOR                              │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## 10. Tài liệu tham khảo

1. **Source code**: `Core/Src/MotorControl.c` - Hàm `Motor_HandleRamp()`
2. **Header file**: `Core/Inc/MotorControl.h` - Cấu trúc `MotionState_t`
3. **Register map**: `Core/Inc/ModbusMap.h` - Các giá trị mặc định

---

**Phiên bản tài liệu**: 1.0  
**Cập nhật lần cuối**: Tháng 12/2024  
**Tác giả**: RAYBOT Team
