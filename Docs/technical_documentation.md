# 📘 TÀI LIỆU KỸ THUẬT - MODULE ĐIỀU KHIỂN ĐỘNG CƠ STEP

## 📋 Mục Lục
1. [Tổng Quan Hệ Thống](#1-tổng-quan-hệ-thống)
2. [Phần Cứng Sử Dụng](#2-phần-cứng-sử-dụng)
3. [Sơ Đồ Giải Thuật](#3-sơ-đồ-giải-thuật)
4. [Chi Tiết Các Thanh Ghi](#4-chi-tiết-các-thanh-ghi)
5. [Hướng Dẫn Sử Dụng](#5-hướng-dẫn-sử-dụng)

---

## 1. Tổng Quan Hệ Thống

### 1.1. Giới Thiệu
Module điều khiển động cơ step là một hệ thống nhúng dựa trên vi điều khiển **STM32F103C8T6** có khả năng điều khiển đồng thời **2 động cơ bước** thông qua giao thức **Modbus RTU** qua UART.

### 1.2. Tính Năng Chính
- ✅ Điều khiển 2 động cơ bước độc lập
- ✅ 2 chế độ điều khiển:
  - **Mode 1 (ON/OFF)**: Chuyển đổi tốc độ ngay lập tức
  - **Mode 2 (S-CURVE/RAMP)**: Tăng/giảm tốc mượt mà với kiểm soát jerk
- ✅ Giao tiếp Modbus RTU (RS485/UART)
- ✅ 4 Digital Input có thể cấu hình chức năng
- ✅ 2 Digital Output có thể cấu hình chức năng
- ✅ Đọc phản hồi encoder (tốc độ, số xung, số vòng quay)
- ✅ FreeRTOS đa luồng (5 tasks)
- ✅ Cấu hình baudrate, parity, stop bit động

### 1.3. Thông Số Kỹ Thuật
| Thông Số | Giá Trị |
|----------|---------|
| Vi điều khiển | STM32F103C8T6 |
| Tần số xung nhịp | 72 MHz |
| Điện áp hoạt động | 3.3V logic, 12-24V motor |
| Giao tiếp | UART/RS485 (Modbus RTU) |
| Baudrate | 9600 - 115200 bps |
| Số motor hỗ trợ | 2 (độc lập) |
| Tần số PWM tối đa | 50 kHz |
| Digital Input | 4 (DI1-DI4) |
| Digital Output | 2 (DO1-DO2) |
| RTOS | FreeRTOS |

---

## 2. Phần Cứng Sử Dụng

### 2.1. Vi Điều Khiển: STM32F103C8T6
- **ARM Cortex-M3 Core**, 72 MHz
- **64 KB Flash, 20 KB SRAM**
- **3x Timer**: TIM1, TIM2, TIM3
- **UART**: USART2 cho Modbus
- **I2C**: I2C1 (dự phòng cho cảm biến)
- **GPIO**: Điều khiển EN, DIR, Digital I/O

### 2.2. Sơ Đồ Chân GPIO

#### Motor 1
| Chức Năng | Chân GPIO | Mô Tả |
|-----------|-----------|-------|
| STEP (PWM) | PA6 (TIM3_CH1) | Xung bước |
| DIR | PA4 | Chiều quay (0=Reverse, 1=Forward) |
| EN | PA5 | Enable động cơ (0=Enable, 1=Disable) |

#### Motor 2
| Chức Năng | Chân GPIO | Mô Tả |
|-----------|-----------|-------|
| STEP (PWM) | PA8 (TIM1_CH1) | Xung bước |
| DIR | PB6 | Chiều quay (0=Reverse, 1=Forward) |
| EN | PB7 | Enable động cơ (0=Enable, 1=Disable) |

#### Digital I/O
| Chức Năng | Chân GPIO | Mô Tả |
|-----------|-----------|-------|
| DI1 | PA7 | Digital Input 1 |
| DI2 | PB10 | Digital Input 2 |
| DI3 | PB11 | Digital Input 3 |
| DI4 | PB12 | Digital Input 4 |
| DO1 | PB8 | Digital Output 1 |
| DO2 | PB9 | Digital Output 2 |

#### Encoder (Motor 1)
| Chức Năng | Chân GPIO | Mô Tả |
|-----------|-----------|-------|
| ENC_IN | PA0 (TIM2_CH1) | Tín hiệu encoder (External Clock Mode) |

#### Communication
| Chức Năng | Chân GPIO | Mô Tả |
|-----------|-----------|-------|
| UART2_TX | PA2 | Modbus RTU TX |
| UART2_RX | PA3 | Modbus RTU RX |

#### Status LEDs
| Chức Năng | Chân GPIO | Mô Tả |
|-----------|-----------|-------|
| LED2 | PC13 | Heartbeat (nhấp nháy 120ms) |
| LED3 | PC14 | Modbus Activity |
| LED4 | PC15 | (Dự phòng) |

### 2.3. Timer Configuration

#### TIM1 (Motor 2 - PWM)
- **Clock**: APB2 (72 MHz)
- **Mode**: PWM Output
- **Channel**: CH1 (PA8)
- **AutoReloadPreload**: Enabled (giảm jitter)
- **Prescaler**: Động (tính theo tần số mong muốn)
- **ARR**: Động (0-65535)

#### TIM2 (Encoder)
- **Clock**: External Clock Mode
- **Mode**: Input Capture (TI1F_ED)
- **Channel**: CH1 (PA0)
- **Filter**: 8 (lọc nhiễu)
- **Counter**: 16-bit (0-65535)

#### TIM3 (Motor 1 - PWM)
- **Clock**: APB1 (36 MHz x 2 = 72 MHz)
- **Mode**: PWM Output
- **Channel**: CH1 (PA6)
- **AutoReloadPreload**: Enabled (giảm jitter)
- **Prescaler**: Động
- **ARR**: Động

### 2.4. UART/Modbus Configuration
- **Instance**: USART2
- **Default Baudrate**: 115200 bps
- **Word Length**: 8 bits
- **Stop Bits**: 1 bit
- **Parity**: None
- **Mode**: TX + RX
- **Protocol**: Modbus RTU
- **Frame Timeout**: 3.5 character time

---

## 3. Sơ Đồ Giải Thuật

### 3.1. Kiến Trúc Hệ Thống (FreeRTOS Tasks)

```
┌─────────────────────────────────────────────────────────────┐
│                    MAIN PROGRAM                              │
│  - Khởi tạo HAL, GPIO, UART, Timers                         │
│  - Khởi tạo Modbus Registers                                │
│  - Khởi tạo FreeRTOS Scheduler                              │
└──────────────────────┬──────────────────────────────────────┘
                       │
       ┌───────────────┼───────────────┐
       │               │               │
       ▼               ▼               ▼
┌──────────┐    ┌──────────┐   ┌──────────┐
│ IOTask   │    │UartTask  │   │MotorTask │
│Priority: │    │Priority: │   │Priority: │
│ Normal   │    │  High    │   │ Normal   │
│Period:   │    │Period:   │   │Period:   │
│ 500ms    │    │  20ms    │   │  20ms    │
└──────────┘    └──────────┘   └──────────┘
       │               │               │
       │               │               │
       ▼               ▼               ▼
┌──────────┐    ┌──────────┐   ┌──────────┐
│VisibleTask│    │EncoderTask│  │          │
│Priority: │    │Priority: │   │          │
│ Normal2  │    │ Normal5  │   │          │
│Period:   │    │Period:   │   │          │
│ 120ms    │    │ 100ms    │   │          │
└──────────┘    └──────────┘   └──────────┘
```

### 3.2. Chi Tiết Các Task

#### Task 1: MotorTask (20ms, Priority: Normal)
```
START MotorTask
│
├─> Load System Registers từ Modbus
│   ├─> Device_ID, Baudrate, Parity, Stop_Bit
│   ├─> Module_Type, Firmware_Version
│   └─> Reset_Error_Command
│
├─> Load Motor1 Registers (Base: 0x0000)
│   ├─> Control_Mode, Enable, Command_Speed
│   ├─> Direction, Vmax, Amax, Jmax
│   └─> Max_Acceleration, Max_Deceleration
│
├─> Load Motor2 Registers (Base: 0x0010)
│   └─> (Tương tự Motor1)
│
├─> Kiểm tra Reset_Error_Command
│   └─> Nếu = 1: Reset toàn bộ hệ thống
│
├─> Update Baudrate (nếu thay đổi)
│
├─> Motor_ProcessControl(&motor1)
│   ├─> Nếu Enable = 1:
│   │   ├─> Set Direction (DIR pin)
│   │   ├─> Enable motor (EN pin = LOW)
│   │   ├─> Start PWM (TIM3_CH1)
│   │   │
│   │   └─> Switch Control_Mode:
│   │       ├─> MODE 1 (ON/OFF):
│   │       │   ├─> Tính v_target = Vmin + (Vmax-Vmin)*(Command_Speed/100)
│   │       │   ├─> Set v_actual = v_target (ngay lập tức)
│   │       │   ├─> Tính Actual_Speed (%) = (v_actual - Vmin)/(Vmax-Vmin)*100
│   │       │   └─> Stepper_OutputFreq(TIM3, CH1, v_actual)
│   │       │
│   │       └─> MODE 2 (S-CURVE/RAMP):
│   │           ├─> Tính v_target từ Command_Speed
│   │           ├─> Tính jerk: j = ±Jmax (tùy v_target > v_actual)
│   │           ├─> Cập nhật acceleration: a += j * dt
│   │           ├─> Giới hạn: -Amax ≤ a ≤ Amax
│   │           ├─> Cập nhật velocity: v_actual += a * dt
│   │           ├─> Giới hạn: 0 ≤ v_actual ≤ Vmax
│   │           ├─> Soft clamp đến v_target
│   │           ├─> Tính Actual_Speed (%)
│   │           └─> Stepper_OutputFreq(TIM3, CH1, v_actual)
│   │
│   └─> Nếu Enable = 0:
│       ├─> Stop PWM
│       ├─> Disable motor (EN pin = HIGH)
│       ├─> Reset Actual_Speed = 0
│       └─> Reset motion_state.v_actual = 0
│
├─> Motor_ProcessControl(&motor2)
│   └─> (Tương tự Motor1, dùng TIM1_CH1)
│
├─> Save Motor1 Registers về Modbus
├─> Save Motor2 Registers về Modbus
├─> Save System Registers về Modbus
│
└─> Delay 20ms (osDelayUntil)
```

#### Task 2: UartTask (20ms, Priority: High)
```
START UartTask
│
├─> Tính frameTimeout = (11 bit/char * 4 char * 1000ms) / Baudrate
│   └─> Tối thiểu 5ms
│
├─> Loop:
│   ├─> Kiểm tra frameReceived flag
│   │   └─> Nếu = 1: processModbusFrame()
│   │       ├─> Verify CRC
│   │       ├─> Kiểm tra Slave Address
│   │       ├─> Parse Function Code (03, 06, 10)
│   │       ├─> Đọc/Ghi Holding Registers
│   │       ├─> Gửi phản hồi qua UART
│   │       └─> Reset buffer
│   │
│   ├─> Kiểm tra timeout frame chưa hoàn chỉnh
│   │   └─> Nếu (rxIndex > 0) và (HAL_GetTick() - lastActivity > frameTimeout):
│   │       ├─> Reset rxIndex = 0
│   │       └─> g_corruptionCount++
│   │
│   ├─> checkUARTHealth()
│   │   └─> Kiểm tra UART không bị treo
│   │
│   └─> Delay 20ms
```

#### Task 3: IOTask (500ms, Priority: Normal)
```
START IOTask
│
├─> Loop:
│   ├─> DOutput_Load(&doutput_state)
│   │   └─> Đọc DO_Control, DO_Assignment từ Modbus
│   │
│   ├─> DOutput_Process(&doutput_state)
│   │   ├─> Xử lý logic assignment:
│   │   │   ├─> 0 = None
│   │   │   ├─> 1 = Running M1
│   │   │   ├─> 2 = Fault M1
│   │   │   └─> ...
│   │   └─> Set GPIO DO1, DO2
│   │
│   ├─> DOutput_Save(&doutput_state)
│   │   └─> Cập nhật DO_Status_Word về Modbus
│   │
│   ├─> g_taskCounter++
│   │
│   └─> Delay 500ms
```

#### Task 4: EncoderTask (100ms, Priority: Normal5)
```
START EncoderTask
│
├─> Loop:
│   ├─> Encoder_Update()
│   │   ├─> Load Reset_Flag từ Modbus
│   │   │   └─> Nếu = 1: Encoder_Reset()
│   │   │
│   │   ├─> Load Diameter, Revolutions từ Modbus
│   │   │
│   │   ├─> Đọc TIM2 Counter (External Clock Mode)
│   │   │   └─> new_pulses = counter_now - last_counter
│   │   │
│   │   ├─> pulse_accumulator += new_pulses
│   │   │
│   │   ├─> Mỗi 500ms:
│   │   │   ├─> pulses_per_second = pulse_accumulator * 2
│   │   │   ├─> mm_per_pulse = (π * diameter) / (revolutions * gear_ratio)
│   │   │   ├─> speed_mm_s = pulses_per_second * mm_per_pulse
│   │   │   ├─> encoder.velocity = speed_mm_s
│   │   │   ├─> encoder.pulse_count = pulses_in_window
│   │   │   └─> pulse_accumulator = 0
│   │   │
│   │   ├─> Save Velocity, Pulse_Count về Modbus
│   │   │
│   │   └─> Delay 100ms
```

#### Task 5: VisibleTask (120ms, Priority: Normal2)
```
START VisibleTask
│
├─> Loop:
│   ├─> Toggle LED2 (Heartbeat)
│   ├─> Nếu g_ledIndicator = 1:
│   │   ├─> Toggle LED3 (Modbus Activity)
│   │   └─> g_ledIndicator = 0
│   └─> Delay 120ms
```

### 3.3. Thuật Toán Tạo PWM Tần Số Biến Đổi

#### Hàm: Stepper_OutputFreq(TIM, Channel, v_actual)
```
INPUT: 
  - TIM: Timer handle (TIM1 hoặc TIM3)
  - Channel: PWM channel (TIM_CHANNEL_1)
  - v_actual: Tần số mong muốn (Hz)

THUẬT TOÁN:
│
├─> Nếu v_actual ≤ 0.1 Hz:
│   ├─> Dừng PWM: __HAL_TIM_SET_COMPARE(TIM, Channel, 0)
│   └─> Return
│
├─> Kiểm tra threshold: |v_actual - v_current| < 0.1 Hz
│   └─> Nếu đúng: Return (không cần cập nhật)
│
├─> Xác định Timer Clock:
│   ├─> Nếu TIM1: timer_clk = PCLK2 (72 MHz)
│   └─> Nếu TIM3: timer_clk = PCLK1 * 2 (72 MHz)
│
├─> Tính desired_counts = timer_clk / v_actual
│
├─> Tính Prescaler (PSC):
│   ├─> Nếu desired_counts > 65536:
│   │   └─> psc = desired_counts / 65536
│   └─> Giới hạn: psc ≤ 0xFFFF
│
├─> Tính Auto-Reload (ARR):
│   └─> arr = (desired_counts / (psc + 1)) - 1
│   └─> Giới hạn: arr ≤ 0xFFFF
│
├─> Tính Compare (CCR) - Duty 50%:
│   └─> ccr = (arr + 1) / 2
│
├─> Cập nhật Shadow Register:
│   ├─> shadow->psc = psc
│   ├─> shadow->arr = arr
│   ├─> shadow->ccr = ccr
│   ├─> shadow->v_current = v_actual
│   └─> shadow->update_pending = 1
│
└─> Enable Update Interrupt:
    └─> __HAL_TIM_ENABLE_IT(TIM, TIM_IT_UPDATE)

NOTE: 
- ARR/CCR chỉ cập nhật tại Update Event (TIM IRQ)
- AutoReloadPreload = ENABLE để tránh glitch
```

### 3.4. Thuật Toán S-Curve (Mode 2)

#### Khái niệm:
S-Curve motion profile kiểm soát **jerk** (đạo hàm bậc 3 của vị trí) để tránh rung động cơ học.

```
Vị trí (s) ───> Vận tốc (v) ───> Gia tốc (a) ───> Jerk (j)
   position         velocity        acceleration      jerk
```

#### Công thức:
```
j(t) = ±Jmax                    (tăng tốc: +, giảm tốc: -)
a(t) = a(t-1) + j * dt          (dt = 0.02s = 20ms)
v(t) = v(t-1) + a(t) * dt
s(t) = s(t-1) + v(t) * dt
```

#### Flowchart:
```
┌─────────────────────────────────────────┐
│  Đầu vào: v_target (từ Command_Speed)   │
│           v_actual (tốc độ hiện tại)     │
│           Jmax, Amax, dt                 │
└─────────────────┬───────────────────────┘
                  │
                  ▼
         ┌────────────────┐
         │ dv = v_target  │
         │    - v_actual  │
         └────────┬───────┘
                  │
        ┌─────────▼──────────┐
        │ dv > 0 (tăng tốc)? │
        └─┬─────────────┬────┘
     YES  │             │  NO
          ▼             ▼
    ┌──────────┐  ┌──────────┐
    │j = +Jmax │  │j = -Jmax │
    └─────┬────┘  └────┬─────┘
          │            │
          └─────┬──────┘
                ▼
       ┌─────────────────┐
       │ a = a + j * dt  │
       │ Giới hạn:       │
       │ -Amax ≤ a ≤ Amax│
       └────────┬────────┘
                ▼
       ┌─────────────────┐
       │v = v + a * dt   │
       │Giới hạn:        │
       │0 ≤ v ≤ Vmax     │
       └────────┬────────┘
                ▼
       ┌─────────────────┐
       │Soft Clamp:      │
       │Nếu v gần v_target│
       │thì v = v_target │
       │     a = 0       │
       │     j = 0       │
       └────────┬────────┘
                ▼
       ┌─────────────────┐
       │Tính Actual_Speed│
       │     (%)         │
       └────────┬────────┘
                ▼
       ┌─────────────────┐
       │Stepper_OutputFreq│
       │   (TIM, v_actual)│
       └─────────────────┘
```

### 3.5. Sơ Đồ Luồng Modbus RTU

```
┌───────────────────────────────────────┐
│    Master gửi Request qua UART        │
└──────────────┬────────────────────────┘
               │
               ▼
┌──────────────────────────────────────┐
│  UART RX Interrupt (HAL_UART_RxCpltCallback) │
│  - Nhận từng byte vào rxBuffer       │
│  - rxIndex++                          │
│  - lastUARTActivity = HAL_GetTick()   │
└──────────────┬────────────────────────┘
               │
               ▼
┌──────────────────────────────────────┐
│  UartTask kiểm tra timeout           │
│  - Nếu (HAL_GetTick() - lastActivity)│
│    > frameTimeout (3.5 char time)    │
│  - frameReceived = 1                 │
└──────────────┬────────────────────────┘
               │
               ▼
┌──────────────────────────────────────┐
│  processModbusFrame()                │
│  ├─> Kiểm tra CRC                    │
│  ├─> Kiểm tra Slave Address          │
│  ├─> Parse Function Code:            │
│  │   ├─> 0x03: Read Holding Registers│
│  │   ├─> 0x06: Write Single Register │
│  │   └─> 0x10: Write Multiple Regs   │
│  ├─> Thực hiện đọc/ghi g_holdingRegisters[]│
│  ├─> Tạo response frame              │
│  ├─> Tính CRC cho response           │
│  └─> HAL_UART_Transmit(response)     │
└──────────────┬────────────────────────┘
               │
               ▼
┌──────────────────────────────────────┐
│  Reset buffer: rxIndex = 0           │
│  frameReceived = 0                   │
│  g_ledIndicator = 1 (bật LED3)       │
└──────────────────────────────────────┘
```

---

## 4. Chi Tiết Các Thanh Ghi

### 4.1. System Registers (Base: 0x0100)

#### REG_DEVICE_ID (0x0100)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Địa chỉ Modbus Slave (1-247)
- **Giá trị mặc định**: 5
- **Ví dụ**: Ghi 0x0A để đổi địa chỉ slave thành 10

#### REG_CONFIG_BAUDRATE (0x0101)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Tốc độ truyền UART
- **Giá trị**:
  - 1 = 9600 bps
  - 2 = 19200 bps
  - 3 = 38400 bps
  - 4 = 57600 bps
  - 5 = 115200 bps (mặc định)
- **Lưu ý**: Thay đổi sẽ áp dụng sau khi reset hoặc gọi updateBaudrate()

#### REG_CONFIG_PARITY (0x0102)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Cấu hình parity
- **Giá trị**:
  - 0 = None (mặc định)
  - 1 = Even
  - 2 = Odd

#### REG_CONFIG_STOP_BIT (0x0103)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Số stop bit
- **Giá trị**: 1 hoặc 2 (mặc định: 1)

#### REG_MODULE_TYPE (0x0104)
- **Loại**: uint8
- **R/W**: Read Only
- **Mô tả**: Loại module
- **Giá trị**: 5 (Stepper Driver)

#### REG_FIRMWARE_VERSION (0x0105)
- **Loại**: uint16
- **R/W**: Read Only
- **Mô tả**: Phiên bản firmware
- **Format**: 0xMMNN (MM=major, NN=minor)
- **Ví dụ**: 0x0001 = v0.01

#### REG_HARDWARE_VERSION (0x0106)
- **Loại**: uint16
- **R/W**: Read Only
- **Mô tả**: Phiên bản phần cứng
- **Format**: 0xMMNN
- **Ví dụ**: 0x0001 = v0.01

#### REG_SYSTEM_STATUS (0x0107)
- **Loại**: uint16
- **R/W**: Read Only
- **Mô tả**: Trạng thái hệ thống (bitfield)
- **Bit mapping**:
  - Bit 0: Motor1 Running
  - Bit 1: Motor2 Running
  - Bit 2: Motor1 Fault
  - Bit 3: Motor2 Fault
  - Bit 4-15: Reserved

#### REG_SYSTEM_ERROR (0x0108)
- **Loại**: uint16
- **R/W**: Read Only
- **Mô tả**: Mã lỗi hệ thống
- **Giá trị**:
  - 0 = No Error
  - 1 = UART Error
  - 2 = Motor Overcurrent
  - 3 = Encoder Error
  - 4-255 = Reserved

#### REG_RESET_ERROR_COMMAND (0x0109)
- **Loại**: uint16
- **R/W**: Write Only
- **Mô tả**: Lệnh reset lỗi
- **Cách dùng**: Ghi 1 để reset tất cả cờ lỗi và khởi tạo lại hệ thống

---

### 4.2. Motor 1 Registers (Base: 0x0000)

#### REG_M1_CONTROL_MODE (0x0000)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Chế độ điều khiển
- **Giá trị**:
  - 1 = ON/OFF Mode (chuyển đổi tốc độ tức thì)
  - 2 = S-CURVE Mode (tăng/giảm tốc mượt)
- **Mặc định**: 2

#### REG_M1_ENABLE (0x0001)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Kích hoạt động cơ
- **Giá trị**:
  - 0 = DISABLE (EN pin = HIGH, PWM stop)
  - 1 = ENABLE (EN pin = LOW, PWM run)
- **Mặc định**: 0

#### REG_M1_COMMAND_SPEED (0x0002)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Tốc độ đặt (%)
- **Phạm vi**: 0 - 100
- **Mặc định**: 0
- **Công thức**: v_target = Vmin + (Vmax - Vmin) * (Command_Speed / 100)

#### REG_M1_ACTUAL_SPEED (0x0003)
- **Loại**: uint8
- **R/W**: Read Only
- **Mô tả**: Tốc độ thực tế (%)
- **Phạm vi**: 0 - 100
- **Lưu ý**: 
  - Mode ON/OFF: Actual_Speed = Command_Speed ngay lập tức
  - Mode S-CURVE: Actual_Speed thay đổi dần theo jerk/acceleration

#### REG_M1_DIRECTION (0x0004)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Chiều quay
- **Giá trị**:
  - 0 = IDLE (dừng, không phát xung)
  - 1 = FORWARD (DIR pin = HIGH)
  - 2 = REVERSE (DIR pin = LOW)
- **Mặc định**: 0

#### REG_M1_MAX_SPEED (0x0005)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Giới hạn tốc độ tối đa (%)
- **Phạm vi**: 0 - 100
- **Mặc định**: 100
- **Lưu ý**: Command_Speed sẽ bị clamp vào Max_Speed

#### REG_M1_MIN_SPEED (0x0006)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Giới hạn tốc độ tối thiểu (%)
- **Phạm vi**: 0 - 100
- **Mặc định**: 0

#### REG_M1_VMAX (0x0007)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Tốc độ tối đa (×100 Hz)
- **Phạm vi**: 1 - 255
- **Mặc định**: 8 (= 800 Hz)
- **Ví dụ**: Ghi 50 → Vmax = 5000 Hz = 5 kHz

#### REG_M1_AMAX (0x0008)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Gia tốc tối đa (×100 Hz/s)
- **Phạm vi**: 1 - 255
- **Mặc định**: 5 (= 500 Hz/s)
- **Chỉ dùng cho**: Mode S-CURVE

#### REG_M1_JMAX (0x0009)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Jerk tối đa (×100 Hz/s²)
- **Phạm vi**: 1 - 255
- **Mặc định**: 2 (= 200 Hz/s²)
- **Chỉ dùng cho**: Mode S-CURVE
- **Lưu ý**: Jmax càng nhỏ, chuyển động càng mượt

#### REG_M1_MAX_ACCELERATION (0x000A)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Tốc độ tăng tốc (dự phòng)
- **Mặc định**: 5

#### REG_M1_MAX_DECELERATION (0x000B)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Tốc độ giảm tốc (dự phòng)
- **Mặc định**: 4

#### REG_M1_STATUS_WORD (0x000C)
- **Loại**: uint8
- **R/W**: Read Only
- **Mô tả**: Trạng thái động cơ (bitfield)
- **Bit mapping**:
  - Bit 0: Enabled
  - Bit 1: Running
  - Bit 2: At Target Speed
  - Bit 3: Fault
  - Bit 4-7: Reserved

#### REG_M1_ERROR_CODE (0x000D)
- **Loại**: uint8
- **R/W**: Read Only
- **Mô tả**: Mã lỗi động cơ
- **Giá trị**:
  - 0 = No Error
  - 1 = Overcurrent
  - 2 = Overvoltage
  - 3 = Encoder Error
  - 4-255 = Reserved

---

### 4.3. Motor 2 Registers (Base: 0x0010)

Tương tự Motor 1, địa chỉ base khác:

| Register | Address | Type | R/W |
|----------|---------|------|-----|
| M2_Control_Mode | 0x0010 | uint8 | R/W |
| M2_Enable | 0x0011 | uint8 | R/W |
| M2_Command_Speed | 0x0012 | uint8 | R/W |
| M2_Actual_Speed | 0x0013 | uint8 | R |
| M2_Direction | 0x0014 | uint8 | R/W |
| M2_Max_Speed | 0x0015 | uint8 | R/W |
| M2_Min_Speed | 0x0016 | uint8 | R/W |
| M2_Vmax | 0x0017 | uint8 | R/W |
| M2_Amax | 0x0018 | uint8 | R/W |
| M2_Jmax | 0x0019 | uint8 | R/W |
| M2_Max_Acceleration | 0x001A | uint8 | R/W |
| M2_Max_Deceleration | 0x001B | uint8 | R/W |
| M2_Status_Word | 0x001C | uint8 | R |
| M2_Error_Code | 0x001D | uint8 | R |

---

### 4.4. Digital Input Registers (Base: 0x0020)

#### REG_DI_STATUS_WORD (0x0020)
- **Loại**: uint16
- **R/W**: Read Only
- **Mô tả**: Trạng thái 4 digital input (bitfield)
- **Bit mapping**:
  - Bit 0: DI1 (1=active, 0=inactive)
  - Bit 1: DI2
  - Bit 2: DI3
  - Bit 3: DI4
  - Bit 4-15: Reserved

#### REG_DI1_ASSIGNMENT (0x0021)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Gán chức năng cho DI1
- **Giá trị**:
  - 0 = None (không dùng)
  - 1 = Start M1
  - 2 = Stop M1
  - 3 = Reverse M1
  - 4 = Fault Reset
  - 5 = Mode Switch
  - 6 = Start M2
  - 7 = Stop M2
  - 8 = Reverse M2
  - 9 = Emergency Stop
  - 10 = Jog Mode

#### REG_DI2_ASSIGNMENT (0x0022)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Gán chức năng cho DI2 (giống DI1)

#### REG_DI3_ASSIGNMENT (0x0023)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Gán chức năng cho DI3 (giống DI1)

#### REG_DI4_ASSIGNMENT (0x0024)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Gán chức năng cho DI4 (giống DI1)

#### REG_CURRENT (0x0025)
- **Loại**: uint8
- **R/W**: Read Only
- **Mô tả**: Dòng điện tiêu thụ (×100 mA)
- **Ví dụ**: Giá trị 15 = 1.5A

---

### 4.5. Digital Output Registers (Base: 0x0030)

#### REG_DO_STATUS_WORD (0x0030)
- **Loại**: uint16
- **R/W**: Read Only
- **Mô tả**: Trạng thái 2 digital output (bitfield)
- **Bit mapping**:
  - Bit 0: DO1 (1=on, 0=off)
  - Bit 1: DO2
  - Bit 2-15: Reserved

#### REG_DO1_CONTROL (0x0031)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Điều khiển thủ công DO1
- **Giá trị**: 0 = Off, 1 = On
- **Lưu ý**: Chỉ hoạt động khi DO1_Assignment = 0 (None)

#### REG_DO1_ASSIGNMENT (0x0032)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Gán chức năng cho DO1
- **Giá trị**:
  - 0 = None (điều khiển thủ công)
  - 1 = Running M1 (tự động bật khi M1 chạy)
  - 2 = Fault M1 (bật khi M1 lỗi)
  - 3 = Speed Reached M1 (bật khi đạt tốc độ)
  - 4 = Ready (hệ thống sẵn sàng)

#### REG_DO2_CONTROL (0x0033)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Điều khiển thủ công DO2 (giống DO1)

#### REG_DO2_ASSIGNMENT (0x0034)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Gán chức năng cho DO2 (giống DO1)

---

### 4.6. Encoder Registers (Base: 0x0035)

#### REG_FEEDBACK_VELOCITY (0x0035)
- **Loại**: uint16
- **R/W**: Read Only
- **Mô tả**: Vận tốc phản hồi từ encoder (mm/s)
- **Công thức**: velocity = pulses_per_second × mm_per_pulse
- **Cập nhật**: Mỗi 500ms trong EncoderTask

#### REG_FEEDBACK_PULSE_COUNT (0x0036)
- **Loại**: uint16
- **R/W**: Read Only
- **Mô tả**: Số xung đếm được trong 500ms
- **Lưu ý**: Không phải tổng tích lũy, chỉ là giá trị trong cửa sổ 500ms

#### REG_DIAMETER (0x0037)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Đường kính bánh xe/trục lăn (mm)
- **Mặc định**: 50mm
- **Công thức**: Chu vi = π × diameter

#### REG_REVOLUTIONS (0x0038)
- **Loại**: uint16
- **R/W**: Read/Write
- **Mô tả**: Số khe/xung trên đĩa encoder
- **Mặc định**: 12 (PPR - Pulses Per Revolution)
- **Lưu ý**: Tổng xung/vòng = revolutions × gear_ratio

#### REG_RESET_FLAG (0x0039)
- **Loại**: uint8
- **R/W**: Read/Write
- **Mô tả**: Cờ reset bộ đếm encoder
- **Cách dùng**: Ghi 1 để reset pulse_count, velocity về 0

---

## 5. Hướng Dẫn Sử Dụng

### 5.1. Khởi Động Hệ Thống

1. **Kết nối phần cứng**:
   - Nguồn 12-24V cho động cơ
   - RS485 (A, B) hoặc UART (TX, RX) cho Modbus
   - Encoder (nếu có) kết nối vào PA0

2. **Cấu hình Modbus Master**:
   - Baudrate: 115200 bps (mặc định)
   - Slave Address: 5 (mặc định)
   - Parity: None
   - Stop Bits: 1

3. **Kiểm tra kết nối**:
   - Đọc REG_MODULE_TYPE (0x0104) → Phải trả về 5
   - Đọc REG_FIRMWARE_VERSION (0x0105) → Trả về 0x0001

### 5.2. Điều Khiển Motor 1 - Mode ON/OFF

#### Bước 1: Cấu hình chế độ
```
Ghi REG_M1_CONTROL_MODE (0x0000) = 1
```

#### Bước 2: Cấu hình tốc độ tối đa
```
Ghi REG_M1_VMAX (0x0007) = 10  (= 1000 Hz = 1 kHz)
```

#### Bước 3: Đặt chiều quay
```
Ghi REG_M1_DIRECTION (0x0004) = 1  (FORWARD)
```

#### Bước 4: Enable motor
```
Ghi REG_M1_ENABLE (0x0001) = 1
```

#### Bước 5: Đặt tốc độ
```
Ghi REG_M1_COMMAND_SPEED (0x0002) = 50  (50%)
→ Motor sẽ chạy tức thì ở tốc độ = Vmin + (Vmax - Vmin) × 0.5
→ Với Vmin=8Hz, Vmax=1000Hz → v_actual = 8 + (1000-8)×0.5 = 504 Hz
```

#### Bước 6: Đọc tốc độ thực tế
```
Đọc REG_M1_ACTUAL_SPEED (0x0003) → Trả về 50
```

#### Bước 7: Dừng motor
```
Ghi REG_M1_COMMAND_SPEED (0x0002) = 0
hoặc
Ghi REG_M1_ENABLE (0x0001) = 0
```

### 5.3. Điều Khiển Motor 1 - Mode S-CURVE

#### Bước 1: Cấu hình chế độ
```
Ghi REG_M1_CONTROL_MODE (0x0000) = 2
```

#### Bước 2: Cấu hình tham số S-Curve
```
Ghi REG_M1_VMAX (0x0007) = 10   (1000 Hz)
Ghi REG_M1_AMAX (0x0008) = 5    (500 Hz/s)
Ghi REG_M1_JMAX (0x0009) = 2    (200 Hz/s²)
```

#### Bước 3: Enable và đặt tốc độ
```
Ghi REG_M1_ENABLE (0x0001) = 1
Ghi REG_M1_DIRECTION (0x0004) = 1
Ghi REG_M1_COMMAND_SPEED (0x0002) = 80  (80%)
```

#### Kết quả:
- Motor tăng tốc dần từ 0 → 80% theo profile S-Curve
- Jerk (j) điều khiển gia tốc (a)
- Gia tốc (a) điều khiển vận tốc (v)
- Đọc REG_M1_ACTUAL_SPEED (0x0003) theo thời gian thực:
  - t=0s: 0%
  - t=1s: 20%
  - t=2s: 50%
  - t=3s: 70%
  - t=4s: 80% (đạt setpoint)

### 5.4. Đọc Encoder

#### Bước 1: Cấu hình encoder
```
Ghi REG_DIAMETER (0x0037) = 50      (đường kính 50mm)
Ghi REG_REVOLUTIONS (0x0038) = 12   (12 PPR)
```

#### Bước 2: Đọc tốc độ phản hồi
```
Đọc REG_FEEDBACK_VELOCITY (0x0035) → Trả về tốc độ (mm/s)
Ví dụ: 150 = 150 mm/s = 0.15 m/s
```

#### Bước 3: Đọc số xung
```
Đọc REG_FEEDBACK_PULSE_COUNT (0x0036) → Số xung trong 500ms
Ví dụ: 24 xung → 2 vòng trong 500ms → 4 vòng/giây
```

#### Bước 4: Reset encoder
```
Ghi REG_RESET_FLAG (0x0039) = 1
→ Encoder_Reset() sẽ reset pulse_count, velocity về 0
```

### 5.5. Cấu hình Digital I/O

#### Ví dụ 1: DI1 điều khiển Start Motor 1
```
Ghi REG_DI1_ASSIGNMENT (0x0021) = 1  (Start M1)
→ Khi DI1 = HIGH, Motor 1 tự động Enable
```

#### Ví dụ 2: DO1 hiển thị Motor 1 đang chạy
```
Ghi REG_DO1_ASSIGNMENT (0x0032) = 1  (Running M1)
→ DO1 tự động bật khi Motor 1 đang chạy
```

#### Ví dụ 3: Điều khiển thủ công DO2
```
Ghi REG_DO2_ASSIGNMENT (0x0034) = 0  (None)
Ghi REG_DO2_CONTROL (0x0033) = 1     (On)
→ DO2 sẽ bật
```

### 5.6. Thay Đổi Baudrate

```
Ghi REG_CONFIG_BAUDRATE (0x0101) = 4  (57600 bps)
→ Sau chu kỳ MotorTask (20ms), UART sẽ reinit với baudrate mới
→ Master phải đổi baudrate tương ứng
```

### 5.7. Reset Hệ Thống

```
Ghi REG_RESET_ERROR_COMMAND (0x0109) = 1
→ Hệ thống sẽ gọi initializeModbusRegisters()
→ Tất cả thanh ghi về giá trị mặc định
→ Motor dừng, encoder reset
```

### 5.8. Xử Lý Lỗi

#### Đọc lỗi hệ thống:
```
Đọc REG_SYSTEM_ERROR (0x0108)
→ 0 = OK
→ 1 = UART Error
→ 2 = Motor Overcurrent
```

#### Đọc lỗi motor:
```
Đọc REG_M1_ERROR_CODE (0x000D)
Đọc REG_M2_ERROR_CODE (0x001D)
```

#### Reset lỗi:
```
Ghi REG_RESET_ERROR_COMMAND (0x0109) = 1
```

---

## 6. Ví Dụ Modbus RTU Frame

### 6.1. Đọc Tốc độ Motor 1 (Function Code 0x03)

**Request** (Master → Slave):
```
05 03 00 03 00 01 [CRC_L] [CRC_H]

- 05: Slave Address
- 03: Function Code (Read Holding Registers)
- 00 03: Starting Address (REG_M1_ACTUAL_SPEED = 0x0003)
- 00 01: Number of Registers (1)
- CRC: 2 bytes
```

**Response** (Slave → Master):
```
05 03 02 32 [CRC_L] [CRC_H]

- 05: Slave Address
- 03: Function Code
- 02: Byte Count (2 bytes = 1 register)
- 32: Data (50 = 50%)
- CRC: 2 bytes
```

### 6.2. Ghi Tốc độ Motor 1 (Function Code 0x06)

**Request**:
```
05 06 00 02 00 50 [CRC_L] [CRC_H]

- 05: Slave Address
- 06: Function Code (Write Single Register)
- 00 02: Register Address (REG_M1_COMMAND_SPEED = 0x0002)
- 00 50: Data (80 = 80%)
- CRC: 2 bytes
```

**Response**:
```
05 06 00 02 00 50 [CRC_L] [CRC_H]
(Echo lại request nếu thành công)
```

### 6.3. Ghi Nhiều Thanh Ghi (Function Code 0x10)

**Request**: Cấu hình Motor 1 (Mode, Enable, Speed, Direction)
```
05 10 00 00 00 04 08 02 01 50 01 [CRC_L] [CRC_H]

- 05: Slave Address
- 10: Function Code (Write Multiple Registers)
- 00 00: Starting Address (REG_M1_CONTROL_MODE = 0x0000)
- 00 04: Number of Registers (4)
- 08: Byte Count (8 bytes = 4 registers × 2)
- 02: M1_Control_Mode = 2 (S-CURVE)
- 01: M1_Enable = 1
- 50: M1_Command_Speed = 80
- 01: M1_Direction = 1 (FORWARD)
- CRC: 2 bytes
```

**Response**:
```
05 10 00 00 00 04 [CRC_L] [CRC_H]

- 05: Slave Address
- 10: Function Code
- 00 00: Starting Address
- 00 04: Number of Registers
- CRC: 2 bytes
```

---

## 7. Troubleshooting

### 7.1. Motor không chạy

**Kiểm tra**:
1. ✅ REG_M1_ENABLE (0x0001) = 1?
2. ✅ REG_M1_COMMAND_SPEED (0x0002) > 0?
3. ✅ REG_M1_DIRECTION (0x0004) = 1 hoặc 2? (không phải 0 - IDLE)
4. ✅ EN pin (PA5/PB7) = LOW?
5. ✅ PWM đang phát xung? (dùng oscilloscope kiểm tra PA6/PA8)

### 7.2. Motor rung/jitter

**Nguyên nhân**: Timer ARR/CCR cập nhật không đồng bộ

**Giải pháp**:
- ✅ AutoReloadPreload đã ENABLE (đã fix trong code)
- ✅ Shadow register chỉ cập nhật tại Update Event
- ✅ Threshold 0.1 Hz để tránh cập nhật liên tục

### 7.3. UART Timeout

**Kiểm tra**:
- Baudrate Master = Slave?
- Parity, Stop Bit khớp?
- Frame timeout = 3.5 × (11 bit / baudrate)?
- Cable RS485 đấu đúng (A-A, B-B)?

### 7.4. Encoder không đọc được

**Kiểm tra**:
1. ✅ TIM2 đã start? (HAL_TIM_Base_Start(&htim2))
2. ✅ PA0 có tín hiệu? (dùng oscilloscope)
3. ✅ REG_DIAMETER, REG_REVOLUTIONS đã cấu hình?
4. ✅ EncoderTask đang chạy? (LED2 nhấp nháy?)

### 7.5. Tốc độ không chính xác

**Công thức kiểm tra**:
```
v_actual (Hz) = Vmin + (Vmax - Vmin) × (Command_Speed / 100)

Ví dụ:
- Vmin = 8 Hz
- Vmax = 1000 Hz (Vmax register = 10)
- Command_Speed = 50%

→ v_actual = 8 + (1000 - 8) × 0.5 = 504 Hz

Kiểm tra bằng oscilloscope:
- Chu kỳ PWM = 1 / 504 = 1.984 ms
- Tần số = 504 Hz ✅
```

---

## 8. Công Thức Tham Khảo

### 8.1. Timer PWM
```
Timer Clock (TIM1/TIM3) = 72 MHz
Desired Frequency = v_actual (Hz)

desired_counts = Timer Clock / Desired Frequency

If desired_counts > 65536:
    PSC = desired_counts / 65536
Else:
    PSC = 0

ARR = (desired_counts / (PSC + 1)) - 1
CCR = (ARR + 1) / 2  (Duty 50%)

Tần số thực tế = Timer Clock / ((PSC + 1) × (ARR + 1))
```

### 8.2. S-Curve Motion
```
Jerk (j):
    j = +Jmax × 100  (nếu v_target > v_actual)
    j = -Jmax × 100  (nếu v_target < v_actual)

Acceleration (a):
    a(n) = a(n-1) + j × dt
    Giới hạn: -Amax×100 ≤ a ≤ Amax×100

Velocity (v):
    v(n) = v(n-1) + a × dt
    Giới hạn: 0 ≤ v ≤ Vmax×100

Position (s):
    s(n) = s(n-1) + v × dt
```

### 8.3. Encoder Speed
```
Pulses per second (pps) = pulse_accumulator × (1000 / window_ms)
                          window_ms = 500ms → pps = pulse_accumulator × 2

Pulses per revolution = revolutions × gear_ratio
                        revolutions = 12 (PPR)
                        gear_ratio = 2

Wheel circumference (mm) = π × diameter

mm per pulse = circumference / (pulses per revolution)

Velocity (mm/s) = pps × mm_per_pulse
```

---

## 9. Thông Tin Bổ Sung

### 9.1. Phiên Bản Firmware
- **Version**: v0.01 (0x0001)
- **Ngày phát hành**: 2025

### 9.2. Liên Hệ Hỗ Trợ
- **Email**: support@raybot.com
- **Website**: https://raybot.com

### 9.3. Changelog
- **v0.01** (2025-01-15):
  - Phát hành ban đầu
  - Hỗ trợ 2 motor, 2 mode (ON/OFF, S-CURVE)
  - Modbus RTU, 4 DI, 2 DO
  - Encoder feedback

---

## Phụ Lục A: Bảng Tóm Tắt Thanh Ghi

| Address | Name | Type | R/W | Default |
|---------|------|------|-----|---------|
| **SYSTEM REGISTERS** |
| 0x0100 | Device_ID | uint8 | R/W | 5 |
| 0x0101 | Config_Baudrate | uint8 | R/W | 5 (115200) |
| 0x0102 | Config_Parity | uint8 | R/W | 0 (None) |
| 0x0103 | Config_Stop_Bit | uint8 | R/W | 1 |
| 0x0104 | Module_Type | uint8 | R | 5 |
| 0x0105 | Firmware_Version | uint16 | R | 0x0001 |
| 0x0106 | Hardware_Version | uint16 | R | 0x0001 |
| 0x0107 | System_Status | uint16 | R | 0x0000 |
| 0x0108 | System_Error | uint16 | R | 0 |
| 0x0109 | Reset_Error_Command | uint16 | W | 0 |
| **MOTOR 1 REGISTERS** |
| 0x0000 | M1_Control_Mode | uint8 | R/W | 2 |
| 0x0001 | M1_Enable | uint8 | R/W | 0 |
| 0x0002 | M1_Command_Speed | uint8 | R/W | 0 |
| 0x0003 | M1_Actual_Speed | uint8 | R | 0 |
| 0x0004 | M1_Direction | uint8 | R/W | 0 |
| 0x0005 | M1_Max_Speed | uint8 | R/W | 100 |
| 0x0006 | M1_Min_Speed | uint8 | R/W | 0 |
| 0x0007 | M1_Vmax | uint8 | R/W | 8 |
| 0x0008 | M1_Amax | uint8 | R/W | 5 |
| 0x0009 | M1_Jmax | uint8 | R/W | 2 |
| 0x000A | M1_Max_Acceleration | uint8 | R/W | 5 |
| 0x000B | M1_Max_Deceleration | uint8 | R/W | 4 |
| 0x000C | M1_Status_Word | uint8 | R | 0x0000 |
| 0x000D | M1_Error_Code | uint8 | R | 0 |
| **MOTOR 2 REGISTERS** |
| 0x0010-0x001D | (Tương tự Motor 1) | - | - | - |
| **DIGITAL I/O** |
| 0x0020 | DI_Status_Word | uint16 | R | 0x0000 |
| 0x0021 | DI1_Assignment | uint8 | R/W | 0 |
| 0x0022 | DI2_Assignment | uint8 | R/W | 0 |
| 0x0023 | DI3_Assignment | uint8 | R/W | 0 |
| 0x0024 | DI4_Assignment | uint8 | R/W | 0 |
| 0x0025 | Current | uint8 | R | 0 |
| 0x0030 | DO_Status_Word | uint16 | R | 0x0000 |
| 0x0031 | DO1_Control | uint8 | R/W | 0 |
| 0x0032 | DO1_Assignment | uint8 | R/W | 0 |
| 0x0033 | DO2_Control | uint8 | R/W | 0 |
| 0x0034 | DO2_Assignment | uint8 | R/W | 0 |
| **ENCODER** |
| 0x0035 | Feedback_Velocity | uint16 | R | 0 |
| 0x0036 | Feedback_Pulse_Count | uint16 | R | 0 |
| 0x0037 | Diameter | uint8 | R/W | 50 |
| 0x0038 | Revolutions | uint16 | R/W | 12 |
| 0x0039 | Reset_Flag | uint8 | R/W | 0 |

---

**HẾT TÀI LIỆU**
