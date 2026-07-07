#include "stm32f10x.h"
#include "OLED.h"
#include <stdio.h>
#include <stdlib.h>

// --- 硬件与参数配置 ---
#define SERVO_MIN 500    // 270度舵机最小脉宽
#define SERVO_MID 1500   // 中位
#define SERVO_MAX 2500   // 最大脉宽
#define DEADZONE 40      // 扩大死区：中心点正负40像素内不动，防止微颤
#define MAX_STEP 35      // 限制单次最大脉宽增量，解决“猛甩头”问题

// --- PID 调优参数 (重点) ---
float Kp = 0.18f;        // 调小比例项：让移动更温柔 (原 0.45)
float Kd = 0.35f;        // 加大微分项：增加阻尼，防止过冲摇晃 (原 0.12)

volatile int current_pwm = SERVO_MID;
volatile int latest_error = 0;
volatile uint32_t rx_count = 0;
volatile uint32_t raw_byte_count = 0;
volatile uint8_t last_rx_byte = 0;
volatile uint8_t servo_moved_flag = 0;
volatile uint8_t rx_frame_flag = 0;
int last_error = 0;

// --- 函数声明 ---
void RCC_Config(void);
void GPIO_Config(void);
void TIM2_PWM_Init(void);
void USART1_Init(void);
void Servo_SetPWM(int pwm);
void OLED_Status_Init(void);
void OLED_Status_Update(void);

// --- 核心平滑控制逻辑 ---
void Control_Loop(int error) {
    // 1. 死区判定
    if (abs(error) < DEADZONE) {
        last_error = error;
        servo_moved_flag = 0;
        return;
    }

    // 2. 计算 PD 输出
    // error - last_error 是变化率，Kd 越大，阻力越大
    float delta = (Kp * (float)error) + (Kd * (float)(error - last_error));
    last_error = error;

    // 3. 限速逻辑：单次移动不能超过 MAX_STEP
    if (delta > MAX_STEP) delta = MAX_STEP;
    if (delta < -MAX_STEP) delta = -MAX_STEP;

    // 4. 更新 PWM 指令 (注意方向：如果反了，请把 -= 改为 +=)
    current_pwm -= (int)delta;

    // 5. 执行
    Servo_SetPWM(current_pwm);
    servo_moved_flag = 1;
}

// 串口中断服务：解析格式 "E120\n"
void USART1_IRQHandler(void) {
    static char buf[10];
    static int idx = 0;
    static uint8_t receiving = 0;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint16_t status = USART1->SR;
        char c = USART_ReceiveData(USART1);
        if (status & (USART_FLAG_PE | USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_ORE)) {
            receiving = 0;
            idx = 0;
            return;
        }
        raw_byte_count++;
        last_rx_byte = (uint8_t)c;
        if (c == 'E') {
            idx = 0;
            receiving = 1;
        }
        else if (c == '\n') {
            if (receiving) {
                buf[idx] = '\0';
                int err = atoi(buf);
                latest_error = err;
                rx_count++;
                rx_frame_flag = 1;
                Control_Loop(err);
            }
            receiving = 0;
        } else if (receiving && idx < 9) {
            buf[idx++] = c;
        }
    }
}

int main(void) {
    RCC_Config();
    GPIO_Config();
    TIM2_PWM_Init();
    USART1_Init();
    Servo_SetPWM(SERVO_MID);
    OLED_Status_Init();
    while (1) {
        OLED_Status_Update();
    }
}

void OLED_Status_Init(void) {
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "RX:E+0000      ");
    OLED_ShowString(2, 1, "F:00000 B:00000");
    OLED_ShowString(3, 1, "PWM:1500 C:00  ");
    OLED_ShowString(4, 1, "HB:0000 WAIT   ");
}

void OLED_Status_Update(void) {
    static uint32_t last_count = 0xFFFFFFFF;
    static uint32_t last_raw_count = 0xFFFFFFFF;
    static uint32_t heartbeat_div = 0;
    static uint32_t heartbeat = 0;
    uint32_t count_snapshot;
    uint32_t raw_count_snapshot;
    int error_snapshot;
    int pwm_snapshot;
    uint8_t byte_snapshot;
    uint8_t move_snapshot;

    heartbeat_div++;
    if (heartbeat_div >= 50000) {
        heartbeat_div = 0;
        heartbeat++;
        OLED_ShowString(4, 1, "HB:");
        OLED_ShowNum(4, 4, heartbeat % 10000, 4);
        OLED_ShowString(4, 9, rx_count > 0 ? "RX OK  " : "WAIT   ");
    }

    if (last_count != rx_count || last_raw_count != raw_byte_count) {
        count_snapshot = rx_count;
        raw_count_snapshot = raw_byte_count;
        error_snapshot = latest_error;
        pwm_snapshot = current_pwm;
        byte_snapshot = last_rx_byte;
        move_snapshot = servo_moved_flag;

        OLED_ShowString(1, 1, "RX:E");
        OLED_ShowSignedNum(1, 5, error_snapshot, 4);
        OLED_ShowString(1, 10, "      ");

        OLED_ShowString(2, 1, "F:");
        OLED_ShowNum(2, 3, count_snapshot % 100000, 5);
        OLED_ShowString(2, 9, "B:");
        OLED_ShowNum(2, 11, raw_count_snapshot % 100000, 5);

        OLED_ShowString(3, 1, "PWM:");
        OLED_ShowNum(3, 5, pwm_snapshot, 4);
        OLED_ShowString(3, 10, "C:");
        OLED_ShowHexNum(3, 12, byte_snapshot, 2);
        OLED_ShowString(3, 14, "  ");

        OLED_ShowString(4, 9, move_snapshot ? "MOVE   " : "HOLD   ");

        rx_frame_flag = 0;
        last_count = count_snapshot;
        last_raw_count = raw_count_snapshot;
    }
}

// (以下硬件初始化代码与之前版本一致，略...)
void Servo_SetPWM(int pwm) {
    if (pwm < SERVO_MIN) pwm = SERVO_MIN;
    if (pwm > SERVO_MAX) pwm = SERVO_MAX;
    TIM_SetCompare1(TIM2, pwm);
}

// --- 硬件初始化 ---

void RCC_Config(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // PA0 - TIM2_CH1 PWM 输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA9 - USART1_TX (调试用)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA10 - USART1_RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void TIM2_PWM_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 舵机需要 50Hz (20ms) 周期
    // 72MHz / 72 = 1MHz, 1MHz / 20000 = 50Hz
    TIM_TimeBaseStructure.TIM_Period = 19999; 
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = SERVO_MID; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_Cmd(TIM2, ENABLE);
}

void USART1_Init(void) {
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
