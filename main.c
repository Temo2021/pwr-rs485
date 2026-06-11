#include "main.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include "cmd.h"

void SystemClock_Config(void);

// ==================== RS485 硬件定义 ====================
#define RS485_DIR_PIN      UART2_DR_Pin
#define RS485_DIR_PORT     GPIOA

// ==================== Modbus 配置 ====================
#define RX_BUF_SIZE        64
#define REG_NUM            10
uint16_t Regs[REG_NUM] = {
    0x1234, 0x5678, 0x9ABC, 0xDEF0,
    0x1111, 0x2222, 0x3333, 0x4444,
    0x5555, 0x6666
};

uint8_t RS485_RX_BUF[RX_BUF_SIZE];
uint8_t RS485_RX_CNT = 0;

// ==================== UART1 调试指令接收缓冲 ====================
#define RX_BUF_LEN 128
uint8_t uart_rx_buf[RX_BUF_LEN];
uint8_t rx_data = 0;
uint16_t rx_cnt = 0;

/* 解决 __aeabi_assert 错误 */
void __aeabi_assert(const char *expr, const char *file, int line)
{

}

// ==================== CRC16 ====================
uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for(uint16_t i=0; i<len; i++)
    {
        crc ^= buf[i];
        for(uint8_t j=0; j<8; j++)
        {
            if(crc & 1)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
                crc >>= 1;
        }
    }
    return crc;
}

// ==================== USART2 中断处理 ====================
void USART2_IRQHandler(void)
{
    uint8_t ch;
    if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
    {
        ch = (uint8_t)(huart2.Instance->RDR & 0xFF);
        if(RS485_RX_CNT < RX_BUF_SIZE)
        {
            RS485_RX_BUF[RS485_RX_CNT++] = ch;
        }
    }
    HAL_UART_IRQHandler(&huart2);
}

// ==================== Modbus 处理 ====================
void Modbus_Process(void)
{
    uint8_t resp[32];
    uint8_t resp_len = 0;

    if(RS485_RX_CNT < 8) goto exit;

    uint8_t addr = RS485_RX_BUF[0];
    uint8_t func = RS485_RX_BUF[1];

    if(addr != 1) goto exit;

    if(func == 0x03)
    {
        uint16_t reg_addr = (RS485_RX_BUF[2] << 8) | RS485_RX_BUF[3];
        uint16_t reg_cnt  = (RS485_RX_BUF[4] << 8) | RS485_RX_BUF[5];
        if(reg_addr + reg_cnt > REG_NUM) goto exit;

        resp[0] = addr;
        resp[1] = 0x03;
        resp[2] = reg_cnt * 2;
        resp_len = 3;

        for(int i = 0; i < reg_cnt; i++)
        {
            resp[resp_len++] = Regs[reg_addr + i] >> 8;
            resp[resp_len++] = Regs[reg_addr + i] & 0xFF;
        }
        uint16_t crc = Modbus_CRC16(resp, resp_len);
        resp[resp_len++] = crc & 0xFF;
        resp[resp_len++] = crc >> 8;
    }
    else if(func == 0x06)
    {
        uint16_t reg_addr = (RS485_RX_BUF[2] << 8) | RS485_RX_BUF[3];
        uint16_t val = (RS485_RX_BUF[4] << 8) | RS485_RX_BUF[5];
        if(reg_addr >= REG_NUM) goto exit;
        Regs[reg_addr] = val;

        for(int i = 0; i < 8; i++)
            resp[i] = RS485_RX_BUF[i];
        resp_len = 8;
    }
    else
        goto exit;

    HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart2, resp, resp_len, 200);
    while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);
    HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_RESET);

exit:
    RS485_RX_CNT = 0;
}

/* printf 重定向到 UART1 */
/* printf 重定向到 UART1（直接操作寄存器，绕过 HAL 阻塞问题） */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// ==================== 主函数 ====================
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_USART1_UART_Init();

    // UART1 调试指令初始化
    printf("\r\n==================================\r\n");
    printf("  UART1 Command System Ready\r\n");
    printf("  TX:PB6  RX:PB7\r\n");
    printf("==================================\r\n");
    printf(">> ");
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);

    // RS485 初始化
    HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_RESET);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

    uint32_t led_tick = HAL_GetTick();

    while (1)
    {
        // LED 闪烁
        if(HAL_GetTick() - led_tick >= 300)
        {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            led_tick = HAL_GetTick();
        }
	//	uint8_t data;
    //if (HAL_UART_Receive(&huart1, &data, 1, 100) == HAL_OK) {
        // 轮询接收成功，发送回去
     //   HAL_UART_Transmit(&huart1, &data, 1, 100);
   // }

        // Modbus 处理
       if(RS485_RX_CNT > 0)
       {
           HAL_Delay(10);
           Modbus_Process();
       }
    }
}

// ==================== 时钟配置 ====================
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    // 只用 HSI 16MHz，关闭PLL，最简时钟
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    // 16MHz 下 FLASH 延时设为 0    23232323
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

void Error_Handler(void)
{
    while(1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(100);
    }
}

// ==================== UART1 接收中断回调（指令系统） ====================
// 接收中断回调（main.c 最后面）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        uint8_t ch = rx_data;

        if(ch == '\n' || ch == '\r')
        {
            uart_rx_buf[rx_cnt] = '\0';
            if(rx_cnt > 0)
            {
                // 发送换行和解析结果（printf 内部也是用寄存器，没问题）
                printf("\r\n");
                CMD_Parse((char*)uart_rx_buf);
                printf("\r\n>> ");
            }
            else
            {
                printf("\r\n>> ");
            }
            rx_cnt = 0;
        }
        else if(rx_cnt < RX_BUF_LEN - 1)
        {
            if(ch >= 0x20 && ch <= 0x7E)
            {
                uart_rx_buf[rx_cnt++] = ch;
                // ***** 关键：非阻塞回显 *****
                while ((USART1->ISR & USART_ISR_TXE_TXFNF) == 0);
                USART1->TDR = ch;
            }
        }
        else
        {
            rx_cnt = 0;
            printf("\r\nBuffer overflow\r\n>> ");
        }

        // 重新开启接收中断
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}

// 必须保留，删除会导致中断异常、主循环卡死.必须文件
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
