/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : NTC 10K temperature measurement via ADS1118 ADC (SPI)
  *
  * Hardware:
  *   ADS1118 AIN0 <- NTC voltage divider (R34 10k + NTC 10k @ 3.3V)
  *   CS_ADC       -> PB0
  *   SPI1 SCK/MISO/MOSI -> PA5/PA6/PA7
  *   USART1       -> temperature output
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver_ads1118.h"
#include "driver_ads1118_interface.h"
#include "ntc_temperature.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef  hspi1;

static ads1118_handle_t gs_handle;

static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

/* Bit-bang SPI diagnostic: bypasses the SPI peripheral entirely.
 * Reconfigures PA5/PA6/PA7 as plain GPIO, manually clocks 16 bits,
 * reads MISO bit-by-bit, then restores SPI.
 * Result interpretation:
 *   rx=0xFFFF -> MISO physically stuck high = hardware problem (no power / bad solder)
 *   rx=anything else -> SPI peripheral was the issue, bit-bang works
 */
static void bitbang_spi_diag(void)
{
    GPIO_InitTypeDef g = {0};
    uint32_t rx16 = 0;
    int i;
    char buf[64];

    HAL_SPI_DeInit(&hspi1);

    /* SCK(PA5), MOSI(PA7) -> output */
    g.Pin = GPIO_PIN_5 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    /* MISO(PA6) -> input, no pull */
    g.Pin = GPIO_PIN_6;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);

    /* idle: SCK=0, MOSI=1, CS=1 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    GPIO_PinState miso_idle = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);

    /* assert CS */
    HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);

    GPIO_PinState miso_after_cs = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);

    /* clock 16 bits, Mode 0: sample MISO on rising edge */
    for (i = 15; i >= 0; i--) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); /* MOSI=1 */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); /* rising edge */
        HAL_Delay(1);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)
            rx16 |= (1u << i);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); /* falling edge */
        HAL_Delay(1);
    }

    HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_SET); /* deassert CS */

    snprintf(buf, sizeof(buf), "BBSPI: idle=%d cs=%d rx=0x%04lX\r\n",
             (int)miso_idle, (int)miso_after_cs, rx16);
    uart_print(buf);

    MX_SPI1_Init(); /* restore hardware SPI */
}

static int ADS1118_Setup(void)
{
    DRIVER_ADS1118_LINK_INIT(&gs_handle, ads1118_handle_t);
    DRIVER_ADS1118_LINK_SPI_INIT(&gs_handle, ads1118_interface_spi_init);
    DRIVER_ADS1118_LINK_SPI_DEINIT(&gs_handle, ads1118_interface_spi_deinit);
    DRIVER_ADS1118_LINK_SPI_TRANSMIT(&gs_handle, ads1118_interface_spi_transmit);
    DRIVER_ADS1118_LINK_DELAY_MS(&gs_handle, ads1118_interface_delay_ms);
    DRIVER_ADS1118_LINK_DEBUG_PRINT(&gs_handle, ads1118_interface_debug_print);

    if (ads1118_init(&gs_handle) != 0) { return 1; }
    if (ads1118_set_channel(&gs_handle, ADS1118_CHANNEL_AIN0_GND) != 0) { return 1; }
    if (ads1118_set_range(&gs_handle, ADS1118_RANGE_4P096V) != 0) { return 1; }
    if (ads1118_set_rate(&gs_handle, ADS1118_RATE_128SPS) != 0) { return 1; }
    if (ads1118_set_mode(&gs_handle, ADS1118_MODE_ADC) != 0) { return 1; }
    if (ads1118_set_dout_pull_up(&gs_handle, ADS1118_BOOL_FALSE) != 0) { return 1; }
    if (ads1118_start_continuous_read(&gs_handle) != 0) { return 1; }
    return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);

  /* --- Bit-bang diagnostic: runs before ADS1118 init --- */
  bitbang_spi_diag();

  if (ADS1118_Setup() != 0) {
      uart_print("ADS1118 init FAILED\r\n");
  } else {
      uart_print("ADS1118 init OK\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    int16_t raw = 0;
    float voltage = 0.0f;
    float temp_c = 0.0f;
    float r_ntc = 0.0f;
    char buf[80];
    static uint8_t err_count = 0;

    if (ads1118_continuous_read(&gs_handle, &raw, &voltage) == 0) {
        err_count = 0;
        r_ntc = NTC_R_SERIES_OHM * voltage / (NTC_VCC_V - voltage);
        if (NTC_VoltageToTemperature(voltage, &temp_c) == 0) {
            snprintf(buf, sizeof(buf), "Temp: %.2f C  V=%.4f  R=%.0f  raw=%d\r\n",
                     (double)temp_c, (double)voltage, (double)r_ntc, (int)raw);
        } else {
            snprintf(buf, sizeof(buf), "Temp: OUT_OF_RANGE  V=%.4f  R=%.0f  raw=%d\r\n",
                     (double)voltage, (double)r_ntc, (int)raw);
        }
        uart_print(buf);
    } else {
        err_count++;
        uart_print("ADS1118 read error\r\n");
        if (err_count >= 3) {
            err_count = 0;
            uart_print("Re-initialising ADS1118...\r\n");
            ADS1118_Setup();
        }
    }

    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
