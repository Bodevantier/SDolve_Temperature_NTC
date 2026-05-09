/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : NTC Temperature Sensor to NMEA 2000 Gateway
  *
  * Hardware:
  *   ADS1118 AIN0 <- NTC voltage divider (R34 10k + NTC 10k @ 3.3V)
  *   CS_ADC       -> PB0
  *   SPI1 SCK/MISO/MOSI -> PA5/PA6/PA7
  *   CAN          -> PA11 (RX), PA12 (TX)
  *   USART1       -> debug output
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "can.h"

/* USER CODE BEGIN Includes */
// C modules wrapped for C++ linkage
extern "C" {
  #include "driver_ads1118.h"
  #include "driver_ads1118_interface.h"
  #include "ntc_temperature.h"
  #include "n2k_storage.h"
}

// C++ NMEA2000 libraries
#include "NMEA2000.h"
#include "NMEA2000_STM32.hpp"
#include "N2kMessages.h"
#include "N2kTimer.h"

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMP_UPDATE_PERIOD   2500   // ms – PGN 130316 temperature update period
#define TEMP_OFFSET           500   // ms – offset avoids send failures during address claim
#define TEMP_POLL_PERIOD      500   // ms – how often to read the ADC
#define LED2_FLASH_MS          50   // ms – brief TX indicator flash

// ---------------------------------------------------------------------------
// Power saving: disable the two debug LEDs (status + TX-flash). When set,
// LED helpers become no-ops and both pins are forced LOW once at startup,
// removing the steady ~5 mA drawn by LED1 (always on once N2K opens) and
// the periodic LED2 spike on every PGN 130316 transmission. Set to 0 to
// re-enable the LEDs for diagnostics.
#define POWER_SAVE_DISABLE_LEDS 1

// Power saving: silence the per-poll "Temp: x.x C" debug prints over USART1.
// Boot banner, ADS1118/N2K init status, watchdog/CAN re-init notices are
// kept (only printed once) so a USB-TTL probe still tells you the node
// came up. Set to 0 to get the chatty per-poll telemetry back.
#define POWER_SAVE_QUIET_DEBUG  1

// Power saving: sleep the core in WFI between iterations. SysTick (1 kHz)
// and CAN/SPI/UART IRQs all wake the MCU promptly, so loop responsiveness
// is unchanged. Disable only when chasing real-time bugs that need the
// loop to spin freely.
#define POWER_SAVE_USE_WFI      1

// Power saving: run the ADS1118 in single-shot mode. After each conversion
// the chip enters power-down (~quiescent < 1 µA) until the next request,
// instead of continuously sampling (~150 µA analog front-end). At 8 SPS
// each conversion is ~125 ms, well under the 500 ms poll period. Set to 0
// to fall back to free-running continuous mode for diagnostics.
#define POWER_SAVE_ADS1118_SINGLE 1

// Power saving: gate the SPI digital isolator (EN_SPI / PA3) so it is only
// powered while we're actually talking to the ADS1118. Typical SPI isolators
// (ADuM-class) draw 2-10 mA per channel continuously when enabled; this
// node only needs the bus active briefly per 500 ms poll.
// Brief settle delay (~1 ms) is added after EN before the first SPI byte.
// Set to 0 if your isolator needs longer warm-up or to debug SPI on a scope.
#define POWER_SAVE_GATE_SPI_ISOLATOR 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static ads1118_handle_t gs_handle;

// Temperature data
static float temp_celsius       = 0.0f;
static bool  temp_data_valid    = false;
static uint32_t last_temp_poll  = 0;
static uint32_t temp_error_count = 0;

// LED state
static bool     n2k_is_open   = false;
static uint32_t led2_off_tick = 0;

// Rolling SID (0-252, wraps; ties PGN samples from the same cycle together)
static unsigned char tempSID = 0;

// Independent watchdog – ~2 s timeout (LSI 40 kHz / 64 / 1250 ≈ 2.0 s)
static IWDG_HandleTypeDef hiwdg;

// Pointer captured in NMEA2000_Init for use inside the OnN2kOpen callback
static tNMEA2000 *s_N2k = nullptr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static int  ADS1118_Setup(void);
static void NMEA2000_Init(tNMEA2000_STM32 *N2k);
void        OnN2kOpen(void);
static void PollTemperatureSensor(void);
static void NMEA2000_SendTemperature(tNMEA2000_STM32 *N2k);
static void LED_Update(void);
static void LED2_Flash(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern "C" CAN_HandleTypeDef  hcan;
extern "C" UART_HandleTypeDef huart1;

// ====== Message Scheduler ======
// Disabled at start; OnN2kOpen synchronises it to the library open time.
tN2kSyncScheduler TempScheduler(false, TEMP_UPDATE_PERIOD, TEMP_OFFSET);

// ====== LED helpers ======
static void LED_Update(void)
{
#if POWER_SAVE_DISABLE_LEDS
  // LEDs disabled for power saving – nothing to update.
  (void)led2_off_tick;
  return;
#else
  // LED1: slow blink (1 Hz) while claiming address, solid ON once open
  if (n2k_is_open) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  } else {
    GPIO_PinState state = (HAL_GetTick() % 1000u < 500u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state);
  }
  // LED2: turn off after flash duration expires
  if (led2_off_tick != 0u && HAL_GetTick() >= led2_off_tick) {
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    led2_off_tick = 0u;
  }
#endif
}

static void LED2_Flash(void)
{
#if POWER_SAVE_DISABLE_LEDS
  return;
#else
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  led2_off_tick = HAL_GetTick() + LED2_FLASH_MS;
#endif
}

// ====== ADS1118 Setup ======
static int ADS1118_Setup(void)
{
  DRIVER_ADS1118_LINK_INIT(&gs_handle, ads1118_handle_t);
  DRIVER_ADS1118_LINK_SPI_INIT(&gs_handle, ads1118_interface_spi_init);
  DRIVER_ADS1118_LINK_SPI_DEINIT(&gs_handle, ads1118_interface_spi_deinit);
  DRIVER_ADS1118_LINK_SPI_TRANSMIT(&gs_handle, ads1118_interface_spi_transmit);
  DRIVER_ADS1118_LINK_DELAY_MS(&gs_handle, ads1118_interface_delay_ms);
  DRIVER_ADS1118_LINK_DEBUG_PRINT(&gs_handle, ads1118_interface_debug_print);

  if (ads1118_init(&gs_handle) != 0)                                             { return 1; }
  if (ads1118_set_channel(&gs_handle, ADS1118_CHANNEL_AIN0_GND) != 0)           { return 1; }
  if (ads1118_set_range(&gs_handle, ADS1118_RANGE_4P096V) != 0)                 { return 1; }
  if (ads1118_set_rate(&gs_handle, ADS1118_RATE_8SPS) != 0)                     { return 1; }
  if (ads1118_set_mode(&gs_handle, ADS1118_MODE_ADC) != 0)                      { return 1; }
  if (ads1118_set_dout_pull_up(&gs_handle, ADS1118_BOOL_FALSE) != 0)            { return 1; }
#if POWER_SAVE_ADS1118_SINGLE
  // Don't kick off continuous conversions; ads1118_single_read() will
  // trigger one conversion at a time and the chip auto-powers-down between.
#else
  if (ads1118_start_continuous_read(&gs_handle) != 0)                           { return 1; }
#endif
  return 0;
}

// ====== Poll NTC temperature sensor (non-blocking) ======
static void PollTemperatureSensor(void)
{
  uint32_t now = HAL_GetTick();
  if ((now - last_temp_poll) < TEMP_POLL_PERIOD) { return; }
  last_temp_poll = now;

#if POWER_SAVE_GATE_SPI_ISOLATOR
  // Power up the SPI isolator and let it settle (datasheet typical < 100 µs
  // for ADuM-class parts; HAL_Delay(1) is the minimum granularity here).
  HAL_GPIO_WritePin(EN_SPI_GPIO_Port, EN_SPI_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
#endif

  int16_t raw     = 0;
  float   voltage = 0.0f;
  float   t_c     = 0.0f;

#if POWER_SAVE_ADS1118_SINGLE
  uint8_t adc_ok = (ads1118_single_read(&gs_handle, &raw, &voltage) == 0);
#else
  uint8_t adc_ok = (ads1118_continuous_read(&gs_handle, &raw, &voltage) == 0);
#endif

  if (adc_ok) {
    if (NTC_VoltageToTemperature(voltage, &t_c) == 0) {
      temp_celsius    = t_c;
      temp_data_valid = true;
      temp_error_count = 0;

#if !POWER_SAVE_QUIET_DEBUG
      static char dbg[48];
      int ti = (int)(t_c * 10);
      snprintf(dbg, sizeof(dbg), "Temp: %d.%d C\r\n", ti / 10, (ti < 0 ? -ti : ti) % 10);
      HAL_UART_Transmit(&huart1, (uint8_t *)dbg, (uint16_t)strlen(dbg), 100);
#endif
    }
  } else {
    temp_error_count++;
    if (temp_error_count >= 5) {
#if !POWER_SAVE_QUIET_DEBUG
      static const char err[] = "Temp: ADC ERROR\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t *)err, sizeof(err) - 1u, 100);
#endif
      temp_data_valid  = false;
      temp_error_count = 0;
    }
  }

#if POWER_SAVE_GATE_SPI_ISOLATOR
  // Done talking to the ADS1118 — power down the isolator until next poll.
  HAL_GPIO_WritePin(EN_SPI_GPIO_Port, EN_SPI_Pin, GPIO_PIN_RESET);
#endif
}

// ====== Send temperature PGN 130316 ======
static void NMEA2000_SendTemperature(tNMEA2000_STM32 *N2k)
{
  if (!TempScheduler.IsTime()) { return; }
  TempScheduler.UpdateNextTime();

  if (!temp_data_valid) { return; }

  tN2kMsg N2kMsg;
  // PGN 130316 – Temperature Extended Range
  // ActualTemperature must be in Kelvin; SetTemperature = N2kDoubleNA (not set)
  SetN2kTemperatureExt(N2kMsg,
                       tempSID,
                       0,                                  // TempInstance = 0 (single sensor)
                       N2kts_RefridgerationTemperature,    // Source = Refrigeration
                       CToKelvin((double)temp_celsius),    // Actual temperature in K
                       N2kDoubleNA);                       // Set temperature – not used

  if (N2k->SendMsg(N2kMsg)) {
    LED2_Flash();
  }

  // Advance SID (0-252 per NMEA 2000 spec; 253-255 are reserved)
  if (++tempSID > 252u) { tempSID = 0u; }
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
  MX_CAN_Init();

  /* USER CODE BEGIN 2 */
#if POWER_SAVE_DISABLE_LEDS
  // Force both debug LEDs OFF immediately after GPIO init so they never
  // light up, even briefly, when power-saving build flag is set.
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
#endif
  HAL_Delay(200);

  HAL_UART_Transmit(&huart1, (uint8_t *)"=== SDolve Temperature to N2K ===\r\n", 36, 100);

  if (ADS1118_Setup() != 0) {
    HAL_UART_Transmit(&huart1, (uint8_t *)"ADS1118 init FAILED\r\n", 21, 100);
  } else {
    HAL_UART_Transmit(&huart1, (uint8_t *)"ADS1118 OK\r\n", 12, 100);
  }

#if POWER_SAVE_GATE_SPI_ISOLATOR
  // Setup is complete; cut isolator power until the first poll re-enables it.
  HAL_GPIO_WritePin(EN_SPI_GPIO_Port, EN_SPI_Pin, GPIO_PIN_RESET);
#endif

  tNMEA2000_STM32 N2k(&hcan);
  NMEA2000_Init(&N2k);
  HAL_UART_Transmit(&huart1, (uint8_t *)"N2K init OK\r\n", 13, 100);

  {
    static char src_msg[40];
    int n = snprintf(src_msg, sizeof(src_msg),
                     "N2K src: %u\r\n", (unsigned)N2k.GetN2kSource(0));
    if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t *)src_msg, (uint16_t)n, 100);
  }

  HAL_UART_Transmit(&huart1, (uint8_t *)"System ready!\r\n\r\n", 17, 100);

  // Independent watchdog: prescaler /64, reload 1250 @ 40 kHz LSI ≈ 2.0 s.
  // Started AFTER all init so a slow first-time peripheral bring-up cannot
  // trip a reset.
  hiwdg.Instance       = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload    = 1250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) { Error_Handler(); }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_addr_check = 0u;
  uint32_t last_can_check  = 0u;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Service N2K stack first (ISO requests, group functions, address claim)
    N2k.ParseMessages();

    LED_Update();
    PollTemperatureSensor();
    NMEA2000_SendTemperature(&N2k);

    uint32_t now_tick = HAL_GetTick();

    // Persist negotiated source address if it changed (checked every 1 s)
    if ((now_tick - last_addr_check) >= 1000u) {
      last_addr_check = now_tick;
      if (N2k.ReadResetAddressChanged()) {
        uint8_t new_src = N2k.GetN2kSource(0);
        if (N2kStorage_SaveSource(new_src)) {
          char m[48];
          int n = snprintf(m, sizeof(m), "N2K: src addr -> %u (saved)\r\n", new_src);
          if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t*)m, (uint16_t)n, 100);
        }
      }
    }

    // CAN bus-off recovery watchdog (>5 s persistent bus-off → force re-init)
    if ((now_tick - last_can_check) >= 5000u) {
      last_can_check = now_tick;
      if (HAL_CAN_GetError(&hcan) & HAL_CAN_ERROR_BOF) {
        const char m[] = "CAN: bus-off persistent, re-init\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)m, sizeof(m)-1, 100);
        HAL_CAN_Stop(&hcan);
        HAL_CAN_DeInit(&hcan);
        MX_CAN_Init();
        N2k.Open();
      }
    }

    // Kick the independent watchdog
    HAL_IWDG_Refresh(&hiwdg);

#if POWER_SAVE_USE_WFI
    // Sleep the core until the next interrupt. SysTick fires every 1 ms
    // (HAL tick), and CAN RX / SPI / UART IRQs also wake us, so loop
    // latency stays in the millisecond range while average current drops
    // significantly: the F103 spends most of its time in low-power sleep
    // instead of spinning the busy-loop at full clock.
    __WFI();
#endif
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

#if POWER_SAVE_LOW_CLOCK
  // ---------------------------------------------------------------------
  // Low-power clock tree: HSE bypass direct, PLL OFF.
  //   SYSCLK = HCLK = PCLK1 = PCLK2 = 8 MHz, FLASH latency = 0 ws.
  // HAL_RCC_ClockConfig() updates SystemCoreClock and re-arms SysTick from
  // the new HCLK, so HAL_GetTick / HAL_Delay stay accurate. UART & SPI baud
  // registers are recomputed at HAL_*_Init() from HAL_RCC_GetPCLKxFreq().
  // The NMEA2000_STM32 library does not touch the clock tree; CAN bit
  // timing is handled in can.c via N2K_CAN_PRESCALER (see main.h).
  // ---------------------------------------------------------------------
  RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState        = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState        = RCC_HSI_OFF;          // not needed
  RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_OFF;          // bypass PLL
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSE;  // direct HSE
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;       // HCLK = 8 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;         // PCLK1 = 8 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         // PCLK2 = 8 MHz
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
#else
  // Default: 72 MHz (HSE 8 MHz x PLL x9), PCLK1 = 36 MHz, PCLK2 = 72 MHz.
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
#endif
}

/* USER CODE BEGIN 4 */

// =====================
// NMEA2000 Init
// =====================
static void NMEA2000_Init(tNMEA2000_STM32 *N2k)
{
  // Use STM32 96-bit UID so each physical unit has a unique N2K identity
  const uint32_t uid0 = *(const volatile uint32_t *)0x1FFFF7E8UL;
  const uint32_t uid1 = *(const volatile uint32_t *)0x1FFFF7ECUL;
  const uint32_t uid2 = *(const volatile uint32_t *)0x1FFFF7F0UL;
  static char serial_code[33];
  snprintf(serial_code, sizeof(serial_code), "%08lX%08lX%08lX",
           (unsigned long)uid0, (unsigned long)uid1, (unsigned long)uid2);
  const uint32_t unique_number = (uid0 ^ uid1 ^ uid2) & 0x1FFFFFUL;

  N2k->SetProductInformation(serial_code,
                             101,                          // Product code
                             "SDolve Temperature",         // Model ID
                             "1.0.0.0 (2026-04-23)",      // Software version
                             "1.0.0.0 (2026-04-23)",      // Model version
                             1);                           // Load equivalency: 1 × 50 mA

  N2k->SetDeviceInformation(unique_number,
                            160,   // Device function = Temperature
                            85,    // Device class    = External Environment
                            2046); // Manufacturer code

  // PGN 126998 Configuration Information
  N2k->SetConfigurationInformation("SDolve Marine",
                                   "SDolve Temperature – NTC refrigeration sensor",
                                   "SPI ADS1118 to NMEA 2000 bridge");

  // Restore negotiated source address from flash; derive a stable preferred
  // address from the MCU UID on first boot so two nodes don't collide.
  uint8_t preferred_src;
  if (!N2kStorage_LoadSource(&preferred_src) ||
      preferred_src < 128u || preferred_src > 251u) {
    preferred_src = (uint8_t)(128u + (unique_number % 124u));
  }
  N2k->SetMode(tNMEA2000::N2km_NodeOnly, preferred_src);
  N2k->EnableForward(false);

  // Declare transmitted PGNs
  static const unsigned long TransmitMessages[] PROGMEM = {130316L, 0};
  N2k->ExtendTransmitMessages(TransmitMessages);

  // Capture pointer for OnN2kOpen identity burst
  s_N2k = N2k;
  N2k->SetOnOpen(OnN2kOpen);
  N2k->Open();
}

// =====================
// OnN2kOpen Callback
// =====================
void OnN2kOpen(void)
{
  TempScheduler.UpdateNextTime();
  n2k_is_open = true;

  // Identity burst – announce ourselves immediately so plotters populate
  // device metadata without waiting for an ISO request.
  if (s_N2k != nullptr) {
    s_N2k->SendIsoAddressClaim();
    s_N2k->SendProductInformation();
    s_N2k->SendConfigurationInformation();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
extern "C" void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
extern "C" void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
