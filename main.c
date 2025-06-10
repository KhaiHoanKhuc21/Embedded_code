#include "stm32f4xx.h"
#include <string.h>

// Bộ đệm nhận UART
char rx_buffer[20];
int rx_index = 0;
volatile uint8_t button_state = 0; // 0: ban đầu, 1: nháy, 2: sáng liên tục
volatile uint8_t led_red = 0, led_yellow = 0, led_green = 0; // Trạng thái LED

// Cấu hình UART2 (PA2: TX, PA3: RX)
void USART2_Init(void) {
    RCC->AHB1ENR |= (1 << 0);  // Bật clock GPIOA
    RCC->APB1ENR |= (1 << 17); // Bật clock USART2

    // PA2 (TX), PA3 (RX) dùng AF7
    GPIOA->MODER &= ~((3U << 4) | (3U << 6));
    GPIOA->MODER |= (2U << 4) | (2U << 6);
    GPIOA->AFR[0] |= (7U << 8) | (7U << 12);

    // Cấu hình UART: 9600 baud, 8N1
    USART2->CR1 = 0;
    USART2->BRR = 16000000 / 9600; // Giả định PCLK1 = 16 MHz
    USART2->CR1 |= (1 << 2) | (1 << 3) | (1 << 5) | (1 << 13); // RE, TE, RXNEIE, UE

    NVIC_EnableIRQ(USART2_IRQn);
}

// Cấu hình LED (PA5, PA6, PA7)
void LED_Init(void) {
    RCC->AHB1ENR |= (1 << 0); // Bật clock GPIOA
    GPIOA->MODER &= ~((3U << 10) | (3U << 12) | (3U << 14));
    GPIOA->MODER |= (1U << 10) | (1U << 12) | (1 << 14); // Output mode
}

// Cấu hình nút bấm (PA0)
void Button_Init(void) {
    RCC->AHB1ENR |= (1 << 0); // Bật clock GPIOA
    GPIOA->MODER &= ~(3U << 0); // PA0 input
    GPIOA->PUPDR |= (1U << 0);  // Pull-up PA0
}

// Cấu hình ngắt ngoài cho PA0
void EXTI_Init(void) {
    RCC->APB2ENR |= (1 << 14); // Bật clock SYSCFG
    SYSCFG->EXTICR[0] &= ~(0xF << 0); // PA0 cho EXTI0
    EXTI->IMR |= (1 << 0); // Bật ngắt EXTI0
    EXTI->FTSR |= (1 << 0); // Cạnh xuống (falling edge)
    NVIC_EnableIRQ(EXTI0_IRQn);
}

// Cấu hình Timer 2 cho chu kỳ 200ms
void TIM2_Init(void) {
    RCC->APB1ENR |= (1 << 0); // Bật clock TIM2
    TIM2->PSC = 15999; // Tick 1ms (clock 16MHz)
    TIM2->ARR = 199;   // Chu kỳ 200ms
    TIM2->DIER |= (1 << 0); // Bật ngắt update
    NVIC_EnableIRQ(TIM2_IRQn);
}

// Gửi chuỗi qua UART
void UART_SendString(const char *str) {
    while (*str) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *str++;
    }
}

// Xử lý mã nhận được từ UART
void handle_command(void) {
    rx_buffer[rx_index] = '\0';
    UART_SendString("Received: ");
    UART_SendString(rx_buffer);
    UART_SendString("\r\n");

    if (button_state == 1) { // Trạng thái nháy
        if (strcmp(rx_buffer, "123R") == 0) {
            if (led_red) {
                led_red = 0; // Tắt LED đỏ
                GPIOA->ODR &= ~(1 << 5); // Đảm bảo LED tắt hoàn toàn
                if (!led_yellow && !led_green) {
                    TIM2->CR1 &= ~(1 << 0); // Tắt timer nếu không còn LED nào nháy
                }
                UART_SendString("RED LED turned OFF\r\n");
            } else {
                led_red = 1; // Bật nháy LED đỏ
                TIM2->CR1 |= (1 << 0); // Bật timer (nếu chưa bật)
                UART_SendString("RED LED blinking started\r\n");
            }
        } else if (strcmp(rx_buffer, "123Y") == 0) {
            if (led_yellow) {
                led_yellow = 0; // Tắt LED vàng
                GPIOA->ODR &= ~(1 << 6); // Đảm bảo LED tắt hoàn toàn
                if (!led_red && !led_green) {
                    TIM2->CR1 &= ~(1 << 0); // Tắt timer nếu không còn LED nào nháy
                }
                UART_SendString("YELLOW LED turned OFF\r\n");
            } else {
                led_yellow = 1; // Bật nháy LED vàng
                TIM2->CR1 |= (1 << 0); // Bật timer (nếu chưa bật)
                UART_SendString("YELLOW LED blinking started\r\n");
            }
        } else if (strcmp(rx_buffer, "123G") == 0) {
            if (led_green) {
                led_green = 0; // Tắt LED xanh lá
                GPIOA->ODR &= ~(1 << 7); // Đảm bảo LED tắt hoàn toàn
                if (!led_red && !led_yellow) {
                    TIM2->CR1 &= ~(1 << 0); // Tắt timer nếu không còn LED nào nháy
                }
                UART_SendString("GREEN LED turned OFF\r\n");
            } else {
                led_green = 1; // Bật nháy LED xanh lá
                TIM2->CR1 |= (1 << 0); // Bật timer (nếu chưa bật)
                UART_SendString("GREEN LED blinking started\r\n");
            }
        } else {
            UART_SendString("Unknown command\r\n");
        }
    } else if (button_state == 2) { // Trạng thái sáng liên tục
        if (strcmp(rx_buffer, "123R") == 0) {
            led_red ^= 1; // Đảo trạng thái LED đỏ
            if (led_red) {
                GPIOA->ODR |= (1 << 5); // Bật LED đỏ
                UART_SendString("RED LED ON\r\n");
            } else {
                GPIOA->ODR &= ~(1 << 5); // Tắt LED đỏ
                UART_SendString("RED LED OFF\r\n");
            }
        } else if (strcmp(rx_buffer, "123Y") == 0) {
            led_yellow ^= 1; // Đảo trạng thái LED vàng
            if (led_yellow) {
                GPIOA->ODR |= (1 << 6); // Bật LED vàng
                UART_SendString("YELLOW LED ON\r\n");
            } else {
                GPIOA->ODR &= ~(1 << 6); // Tắt LED vàng
                UART_SendString("YELLOW LED OFF\r\n");
            }
        } else if (strcmp(rx_buffer, "123G") == 0) {
            led_green ^= 1; // Đảo trạng thái LED xanh lá
            if (led_green) {
                GPIOA->ODR |= (1 << 7); // Bật LED xanh lá
                UART_SendString("GREEN LED ON\r\n");
            } else {
                GPIOA->ODR &= ~(1 << 7); // Tắt LED xanh lá
                UART_SendString("GREEN LED OFF\r\n");
            }
        } else {
            UART_SendString("Unknown command\r\n");
        }
    }
    rx_index = 0;
}

// Xử lý ngắt UART
void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        if (USART2->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
            USART2->DR; // Xóa cờ lỗi
            UART_SendString("UART error\r\n");
            rx_index = 0;
        } else {
            char c = USART2->DR;
            if (c == '\r' || c == '\n') {
                handle_command(); // Xử lý lệnh khi nhận xuống dòng
            } else if (rx_index < sizeof(rx_buffer) - 1) {
                rx_buffer[rx_index++] = c; // Lưu ký tự vào bộ đệm
            } else {
                UART_SendString("Buffer overflow\r\n");
                rx_index = 0;
            }
        }
    }
}

// Xử lý ngắt ngoài từ nút bấm
void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1 << 0)) {
        EXTI->PR = (1 << 0); // Xóa cờ ngắt
        if (button_state == 0) {
            button_state = 1; // Chuyển sang trạng thái nháy
            TIM2->CR1 &= ~(1 << 0); // Đảm bảo timer tắt trước khi bật lại
            UART_SendString("Switched to blinking mode\r\n");
        } else if (button_state == 1) {
            button_state = 2; // Chuyển sang trạng thái sáng liên tục
            TIM2->CR1 &= ~(1 << 0); // Tắt timer
            GPIOA->ODR &= ~((1 << 5) | (1 << 6) | (1 << 7)); // Tắt tất cả LED
            UART_SendString("Switched to continuous mode\r\n");
        } else if (button_state == 2) {
            button_state = 0; // Reset về trạng thái ban đầu
            GPIOA->ODR &= ~((1 << 5) | (1 << 6) | (1 << 7)); // Tắt tất cả LED
            UART_SendString("Switched to initial mode\r\n");
        }
    }
}

// Xử lý ngắt Timer 2 (nháy LED)
void TIM2_IRQHandler(void) {
    if (TIM2->SR & (1 << 0)) {
        TIM2->SR &= ~(1 << 0); // Xóa cờ ngắt
        if (led_red) {
            GPIOA->ODR ^= (1 << 5); // Nháy LED đỏ
        }
        if (led_yellow) {
            GPIOA->ODR ^= (1 << 6); // Nháy LED vàng
        }
        if (led_green) {
            GPIOA->ODR ^= (1 << 7); // Nháy LED xanh lá
        }
    }
}

// Hàm chính
int main(void) {
    SystemCoreClockUpdate(); // Cập nhật clock
    LED_Init();
    Button_Init();
    EXTI_Init();
    TIM2_Init();
    USART2_Init();

    // Gửi thông báo kết nối với PuTTY
    UART_SendString("STM32 connected to PuTTY\r\n");

    while (1) {
        __WFI(); // Chờ ngắt
    }
}
