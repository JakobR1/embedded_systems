/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TMP117_STM32_HAL_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int16_t temp_integer;
    int16_t temp_fraction;
} SensorData_t;

typedef struct {
    int16_t temp_integer;
    int16_t temp_fraction;
    uint8_t valid;  // 1 = gültig, 0 = ungültig
} ValidatedSensorData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/** @brief Default I²C address of the TMP117 sensor */
#define TMP117_DEFAULT_ADD 0x48

/** @brief Register address for the temperature register */
#define TMP117_REG_ADD 0x00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
static SensorData_t last_valid_temp = {0, 0};

/* Definitions for uartInputTask */
osThreadId_t uartInputTaskHandle;
const osThreadAttr_t uartInputTask_attributes = {
  .name = "uartInputTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Definitions for controllerTask */
osThreadId_t controllerTaskHandle;
const osThreadAttr_t controllerTask_attributes = {
  .name = "controllerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for measurementTask */
osThreadId_t measurementTaskHandle;
const osThreadAttr_t measurementTask_attributes = {
  .name = "measurementTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Definitions for uartOutputTask */
osThreadId_t uartOutputTaskHandle;
const osThreadAttr_t uartOutputTask_attributes = {
  .name = "uartOutputTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for watchdog_timer */
osTimerId_t watchdog_timerHandle;
const osTimerAttr_t watchdog_timer_attributes = {
		.name = "watchdog_timer"
};

/* Definitions for measurement_timer */
osTimerId_t measurement_timerHandle;
const osTimerAttr_t measurement_timer_attributes = {
		.name = "measurement_timer"
};

/* Definitions for meas_sem */
osSemaphoreId_t meas_semHandle;
const osSemaphoreAttr_t meas_sem_attributes = {
		.name = "meas_sem"
};

/* Definitions for dataQueue */
osMessageQueueId_t dataQueueHandle;
const osMessageQueueAttr_t dataQueue_attributes = {
		.name = "dataQueue"
};

/* Definitions for dataQueueOutput */
osMessageQueueId_t dataQueueOutputHandle;
const osMessageQueueAttr_t dataQueueOutput_attributes = {
  .name = "dataQueueOutput"
};

/* Definitions for intervalQueue */
osMessageQueueId_t intervalQueueHandle;
const osMessageQueueAttr_t intervalQueue_attributes = {
  .name = "intervalQueue"
};

osMessageQueueId_t sensorOutputQueueHandle;
const osMessageQueueAttr_t sensorOutputQueue_attributes = {
    .name = "sensorOutputQueue"
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Print given character on UART 2. Translate '\n' to "\r\n" on the fly. */
/*int __io_putchar(int ch) {
	int ret;
	while((ret=HAL_UART_GetState(&huart2)) != HAL_UART_STATE_READY)
		;
	if (ch == '\n') {
		static uint8_t buf[2] = {'\r', '\n'};
		HAL_UART_Transmit_IT(&huart2, buf, sizeof(buf));
	} else {
		static char buf;
		buf = ch;
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)&buf, 1);
	}
	return ch;
}*/
int __io_putchar(int ch) {
    if (ch == '\n') {
        uint8_t buf[2] = {'\r', '\n'};
        HAL_UART_Transmit(&huart2, buf, 2, HAL_MAX_DELAY);
    } else {
        uint8_t buf = ch;
        HAL_UART_Transmit(&huart2, &buf, 1, HAL_MAX_DELAY);
    }
    return ch;
}

int _write(int file, char *ptr, int len)
{
	for (int DataIdx = 0; DataIdx < len; DataIdx++) {
		__io_putchar(*ptr++);
	}
	return len;
}

void Controller_Task(void *argument)
{
    SensorData_t current_temp;
    ValidatedSensorData_t validated;
    int32_t delta;
    uint32_t flags;
    uint8_t new_interval;

    for (;;)
    {
        // Prüfe, ob ein neues Intervall vom Input Task kam
        if (osMessageQueueGet(intervalQueueHandle, &new_interval, NULL, 0) == osOK)
        {
            osTimerStop(measurement_timerHandle);
            osTimerStart(measurement_timerHandle, new_interval * 1000);

            osTimerStop(watchdog_timerHandle);
            osTimerStart(watchdog_timerHandle, (new_interval + 2) * 1000);

            printf("Controller: Neues Intervall übernommen: %d Sekunden\r\n", new_interval);
        }

        printf("Start wait\r\n");
        flags = osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        printf("Stop wait\r\n");

        if (flags & 0x01)
        {
            osSemaphoreRelease(meas_semHandle);

            if (osMessageQueueGet(dataQueueHandle, &current_temp, NULL, 1000) == osOK)
            {
                // Temperaturdifferenz prüfen
                delta = ((current_temp.temp_integer * 100 + current_temp.temp_fraction) -
                         (last_valid_temp.temp_integer * 100 + last_valid_temp.temp_fraction));

                if (delta < 0) delta = -delta;

                validated.temp_integer = current_temp.temp_integer;
                validated.temp_fraction = current_temp.temp_fraction;
                validated.valid = (delta <= 1000) ? 1 : 0;

                if (validated.valid)
                {
                    last_valid_temp = current_temp;
                    osTimerStart(watchdog_timerHandle, (new_interval + 2) * 1000);
                    printf("Controller: Gültige Messung – sende an Output\r\n");
                }
                else
                {
                    printf("Controller: Ungültige Messung (>10°C) – sende trotzdem\r\n");
                }

                // Validiertes Ergebnis an Output Queue senden
                osMessageQueuePut(sensorOutputQueueHandle, &validated, 0, 0);
            }
            else
            {
                printf("Controller: Fehler – keine Messdaten erhalten\r\n");
            }
        }
    }
}
/*void Controller_Task(void *argument)
{
	SensorData_t current_temp;
	    int32_t delta; // Temperaturdifferenz *100 skaliert
	    uint32_t flags;
	    uint8_t new_interval;

	    for (;;)
	    {
	        // Prüfen, ob ein neuer Intervall vom UART Input Task vorliegt
	        if (osMessageQueueGet(intervalQueueHandle, &new_interval, NULL, 0) == osOK)
	        {
	            // Timer mit neuem Intervall setzen
	            osTimerStop(measurement_timerHandle);
	            osTimerStart(measurement_timerHandle, new_interval * 1000);

	            osTimerStop(watchdog_timerHandle);
	            osTimerStart(watchdog_timerHandle, (new_interval + 2) * 1000);

	            printf("Controller: Neues Intervall übernommen: %d Sekunden\r\n", new_interval);
	        }
	        printf("Start wait\r\n");
	        // Auf Timer-Flag vom Measurement Timer warten
	        flags = osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
	        printf("Stop wait\r\n");
	        if (flags & 0x01)
	        {
	            // Messe starten (Mess-Task wird durch Semaphore getriggert)
	            osSemaphoreRelease(meas_semHandle);

	            // Auf neue Messdaten von Measurement_Task warten
	            if (osMessageQueueGet(dataQueueHandle, &current_temp, NULL, 1000) == osOK)
	            {
	                // Temperaturdifferenz berechnen
	                delta = ((current_temp.temp_integer * 100 + current_temp.temp_fraction) -
	                         (last_valid_temp.temp_integer * 100 + last_valid_temp.temp_fraction));

	                if (delta < 0) delta = -delta;
	                delta = 500;
	                if (delta <= 1000) // Differenz <= 10.00 °C
	                {
	                	printf("Output freigeben\r\n");
	                    // Gültige Messung: letzte Temperatur aktualisieren
	                    last_valid_temp = current_temp;

	                    // Watchdog zurücksetzen
	                    osTimerStart(watchdog_timerHandle, (new_interval + 2) * 1000);
	                    printf("Controller: Gültige Messung – Output freigeben\r\n");
	                    // Output freigeben – z. B. mit Flag 0x02
	                    osThreadFlagsSet(uartOutputTaskHandle, 0x02);
	                }
	                else
	                {
	                    printf("Ungültige Messung: Temperaturänderung zu groß (>10°C)\r\n");
	                }
	            }
	            else
	            {
	                printf("Fehler: Keine Messdaten empfangen\r\n");
	            }
	        }
	    }
}*/
void UART_Input_Task(void *argument)
{
	char input_buffer[10];
	    uint8_t ch;
	    uint8_t idx = 0;
	    int user_interval = 0;

	    printf("Usage: Geben Sie das Messintervall in Sekunden ein (1–10)\r\n");

	    for (;;)
	    {
	        idx = 0;
	        memset(input_buffer, 0, sizeof(input_buffer));
	        printf("Intervall (1-10): ");

	        // Zeichenweise lesen, bis \r (Enter)
	        while (1)
	        {
	            HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY); // blockiert Task
	            HAL_UART_Transmit(&huart2, &ch, 1, HAL_MAX_DELAY); // Echo

	            if (ch == '\r' || ch == '\n') break;

	            if (idx < sizeof(input_buffer) - 1)
	                input_buffer[idx++] = ch;
	        }

	        user_interval = atoi((char *)input_buffer);

	        if (user_interval >= 1 && user_interval <= 10)
	        {
	        	uint8_t interval = (uint8_t)user_interval;
	        	if (osMessageQueuePut(intervalQueueHandle, &interval, 0, 0) == osOK) {
	        	    printf("\r\nIntervall erfolgreich gesendet: %d Sekunden\r\n", interval);
	        	} else {
	        	    printf("\r\nFehler: Intervall konnte nicht gesendet werden.\r\n");
	        	}
	        }
	        else
	        {
	            printf("\r\nFehler: Ungültiger Wert. Bitte geben Sie eine Zahl von 1 bis 10 ein.\r\n");
	        }

	        osDelay(100); // kleine Entlastung
	    }
}
void UART_Output_Task(void *argument)
{
	SensorData_t output_temp;
	    uint32_t flags;
	    ValidatedSensorData_t msg;

	    for (;;)
	    {
	        if (osMessageQueueGet(sensorOutputQueueHandle, &msg, NULL, osWaitForever) == osOK)
	        {
	            if (msg.valid)
	            {
	                printf("Temperatur: %d.%02d °C\r\n", msg.temp_integer, msg.temp_fraction);
	            }
	            else
	            {
	                printf("Info: Ungültige Messung (Differenz > 10°C)\r\n");
	            }
	        }
	    }

	    /*for (;;)
	    {
	    	printf("UART Output Task: Warte auf Queue-Daten\r\n");
	        // Warten auf Daten in der Queue (z. B. max 1 Sekunde warten)
	        if (osMessageQueueGet(dataQueueOutputHandle, &output_temp, NULL, osWaitForever) == osOK)
	        {
	            // Jetzt auf Bestätigungs-Flag vom Controller Task warten
	            flags = osThreadFlagsWait(0x02, osFlagsWaitAny, 1000); // Timeout optional

	            if (flags & 0x02)
	            {
	                // Alles bereit: Temperatur ausgeben
	                printf("Temperatur: %d.%02d °C\r\n",
	                       output_temp.temp_integer,
	                       output_temp.temp_fraction);
	            }
	            else
	            {
	                // Kein gültiges Flag empfangen – ignoriere Messung
	                printf("Info: Messwert ohne Freigabe erhalten – verworfen.\r\n");
	            }
	        }
	    }*/
}
void Measurement_Task(void *argument)
{
    uint8_t raw_temp[2];
    int16_t temp_int = 0, temp_frac = 0;
    SensorData_t measurement;

    for (;;)
    {
    	printf("Measurement Task: Warte auf Semaphore...\r\n");
        // Warte auf das Startsignal vom Controller Task
        osSemaphoreAcquire(meas_semHandle, osWaitForever);
        printf("Measurement Task: Semaphore erhalten – lese Sensor\r\n");

        // Temperatur vom Sensor auslesen
        if (TMP117_read_raw_temp_value(hi2c1, TMP117_DEFAULT_ADD, raw_temp) == HAL_OK)
        {
            TMP117_conv_raw_temp_celsius(raw_temp, &temp_int, &temp_frac);

            measurement.temp_integer = temp_int;
            measurement.temp_fraction = temp_frac;

            // Sende Messwert an die Queue
            osStatus_t status1 = osMessageQueuePut(dataQueueHandle, &measurement, 0, 0);
            if (status1 != osOK) {
                printf("Fehler: dataQueueHandle voll (%d)\r\n", status1);
            }

            osStatus_t status2 = osMessageQueuePut(dataQueueOutputHandle, &measurement, 0, 0);
            if (status2 != osOK) {
                printf("Fehler: dataQueueOutputHandle voll (%d)\r\n", status2);
            }
            printf("Measurement Task: Messwert an Queue geschickt\r\n");
        }
        else
        {
            printf("Sensorlesefehler: TMP117 konnte nicht gelesen werden.\r\n");
        }

        // Optional: kleine Pause (nicht nötig, aber nett zur Lastreduktion)
        osDelay(10);
    }
}
void MeasurementTimerCallback(void *argument)
{
	printf("Timer fired\r\n");
    // Signal an Controller Task senden: neue Messung erforderlich
    osThreadFlagsSet(controllerTaskHandle, 0x01); // z. B. Flag 0x01 für Measurement-Event
}
void WatchdogTimerCallback(void *argument)
{
    printf("Fehler: Watchdog ausgelöst – keine gültige Messung empfangen!\r\n");

    // Rote LED einschalten (Beispiel)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // PB14 = rote LED?

    // Fehlerbehandlungsroutine aufrufen
    Error_Handler();
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* creation of led_bin_sem */
  	meas_semHandle = osSemaphoreNew(1, 0, &meas_sem_attributes);
  	if (meas_semHandle == NULL) {
  		Error_Handler();
  		}
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  	/* creation of led_timer */
  	/* Create the watchdog timer */
  	/* Create the measurement timer */
  	measurement_timerHandle = osTimerNew(
  	    MeasurementTimerCallback,      // Callback-Funktion
  	    osTimerPeriodic,               // Typ: Periodisch
  	    NULL,                          // Argument
  	    &measurement_timer_attributes  // Attribute
  	);
  	if (measurement_timerHandle == NULL) {
  		Error_Handler();
  		}
  	watchdog_timerHandle = osTimerNew(
  	  	    WatchdogTimerCallback,         // Callback-Funktion
  	  	    osTimerOnce,                   // Typ: Einmalig
  	  	    NULL,                          // Argument (kann z. B. ein TaskHandle sein)
  	  	    &watchdog_timer_attributes     // Attribute
  	  	);
  	if (watchdog_timerHandle == NULL) {
  		Error_Handler();
  		}
  	// Starte Timer initial mit 3 Sekunden
  	osTimerStart(measurement_timerHandle, 3000);
  	osTimerStart(watchdog_timerHandle, 5000); // Intervall + 2 Sek
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  		/* creation of ledcolorQueue */
  	dataQueueHandle = osMessageQueueNew(
  	    10,                             // Anzahl an Elementen
  	    sizeof(SensorData_t),         // Größe eines Elements (dein Sensor-Daten-Struct)
  	    &dataQueue_attributes          // Attribute
  	);
  	if (dataQueueHandle == NULL) {
  		Error_Handler();
  		}

  	dataQueueOutputHandle = osMessageQueueNew(10, sizeof(SensorData_t), &dataQueueOutput_attributes);
  	if (dataQueueOutputHandle == NULL) {
  	    Error_Handler();
  	}

  	intervalQueueHandle = osMessageQueueNew(3, sizeof(uint8_t), &intervalQueue_attributes);
  	if (intervalQueueHandle == NULL) {
  	    Error_Handler();
  	}

  	sensorOutputQueueHandle = osMessageQueueNew(5, sizeof(ValidatedSensorData_t), &sensorOutputQueue_attributes);
  	if (sensorOutputQueueHandle == NULL) {
  	    Error_Handler();
  	}

  	uint8_t init_interval = 3;
  	osMessageQueuePut(intervalQueueHandle, &init_interval, 0, 0);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  uartInputTaskHandle = osThreadNew(UART_Input_Task, NULL, &uartInputTask_attributes);
  controllerTaskHandle = osThreadNew(Controller_Task, NULL, &controllerTask_attributes);
  measurementTaskHandle = osThreadNew(Measurement_Task, NULL, &measurementTask_attributes);
  uartOutputTaskHandle = osThreadNew(UART_Output_Task, NULL, &uartOutputTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
