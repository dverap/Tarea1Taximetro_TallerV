/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Cuerpo principal del programa.
 * @author         : Daverson Vera Patiño
 * @date           : 22-10-2025
 ******************************************************************************
 * @attention
 *
 * Taller V (2.0) - Tarea #1
 * PROFESOR: Nerio Andres Montoya Giraldo, PhD.
 *
 * ## Descripción de las herramientas del hardware utilizadas
 *
 * El sistema se implementa sobre una placa de desarrollo ST Nucleo-F411RE y
 * utiliza los siguientes componentes externos:
 *
 * 1.  **Encoder Rotatorio (Incremental):**
 * -   **Función:** Permite al usuario interactuar con el sistema, incrementando
 * o decrementando un valor numérico. Detecta tanto el giro (pasos) como
 * la dirección (sentido horario CW y antihorario CCW).
 * -   **Conexión:** Las salidas A (DT) y B (CLK) del encoder se conectan a los
 * pines PA0 y PA1 del microcontrolador, configurados para la interfaz
 * de encoder del Timer 2 (TIM2).
 *
 * 2.  **Display de 7 Segmentos de 4 Dígitos (Ánodo Común):**
 * -   **Función:** Visualiza un número de hasta 4 cifras (0-4095) que
 * corresponde al valor controlado por el encoder.
 * -   **Conexión:** Los 7 pines de los segmentos (A-G) se conectan al puerto
 * GPIOB. Los 4 pines de control de los dígitos (transistores) se
 * conectan al puerto GPIOA. Se utiliza una configuración de ánodo común,
 * donde para encender un dígito se aplica un nivel BAJO a su pin de control.
 *
 * 3.  **Pulsador (Botón externo -No se uso el sw):**
 * -   **Función:** Permite al usuario cambiar la frecuencia de refresco (FPS)
 * del display de 7 segmentos, ciclando entre varios niveles predefinidos.
 * -   **Conexión:** El pulsador está conectado al pin PA5, configurado como
 * una entrada con interrupción externa (EXTI).
 *
 * 4.  **Inversor con Disparador Schmitt (SN74HC14N):**
 * -   **Función:** Es un componente clave para el acondicionamiento de las
 * señales. Se utiliza para el sistema de anti-rebote (debounce) por
 * hardware tanto para las señales del encoder como para el pulsador.
 * Convierte las señales ruidosas, provenientes de los contactos mecánicos,
 * en señales digitales limpias y definidas (ondas cuadradas), asegurando
 * que el microcontrolador reciba entradas estables y precisas.
 *
 * 5.  **LED de Estado (Blinky)-Se uso el de la plca táctica:**
 * -   **Función:** Indica que el microcontrolador está energizado y ejecutando
 * el programa correctamente. Su parpadeo a una frecuencia constante sirve
 * como una señal visual de "heartbeat" del sistema.
 *
 * ## Explicación de cómo funciona el sistema
 *
 * El firmware está desarrollado utilizando el lenguaje C y las bibliotecas HAL
 * de STM32, orquestando el comportamiento de los periféricos del MCU para
 * cumplir con los requisitos.
 *
 * 1.  **Control del Encoder con Timer 2 (TIM2):**
 * -   El TIM2 está configurado en **Modo Encoder**. Esta potente característica
 * de hardware permite que el timer incremente o decremente su registro
 * contador (`CNT`) automáticamente en respuesta a los pulsos de cuadratura
 * del encoder.
 * -   La dirección del giro es detectada por el hardware del timer basándose
 * en cuál de las señales (A o B) lidera a la otra.
 * -   El `Period` del timer (ARR) se establece en **4095**, lo que define el
 * rango del contador como un valor de 12 bits sin signo. El hardware
 * gestiona automáticamente el desbordamiento (overflow/underflow),
 * pasando de 4095 a 0 al incrementar, y de 0 a 4095 al decrementar.
 * -   El bucle principal (`while(1)`) simplemente lee el valor actual del
 * contador del TIM2 (`__HAL_TIM_GET_COUNTER`) y lo almacena en la variable
 * `counter_value`, liberando a la CPU de la tarea de decodificar las
 * señales del encoder.
 *
 * 2.  **Refresco del Display con Timer 3 (TIM3) y Multiplexación (POV):**
 * -   Para evitar el efecto "fantasma" (ghosting) y controlar los 4 dígitos
 * con un número limitado de pines, se utiliza la técnica de **multiplexación**
 * o Persistencia de la Visión (POV).
 * -   El TIM3 se configura como un timer básico que genera una interrupción
 * periódica a una frecuencia elevada.
 * -   Dentro de la rutina de servicio de interrupción (`HAL_TIM_PeriodElapsedCallback`),
 * el código realiza lo siguiente en cada llamada:
 * a.  Apaga todos los dígitos y segmentos (`clear_display`.
 * b.  Selecciona un dígito para activar (de 0 a 3, secuencialmente).
 * c.  Extrae el número correspondiente de la variable `counter_value`
 * (unidades, decenas, centenas, millares).
 * d.  Activa los segmentos necesarios para formar ese número (`set_segments`).
 * e.  Activa el transistor del dígito seleccionado (`set_active_digit`),
 * iluminándolo.
 * -   Este proceso se repite tan rápido que el ojo humano percibe los cuatro
 * dígitos como si estuvieran encendidos simultáneamente.
 *
 * 3.  **Control de FPS con Interrupción Externa (EXTI):**
 * -   El pin PA5 (conectado al pulsador) está configurado para generar una
 * interrupción (`EXTI`) cuando detecta un flanco de subida.
 * -   La interrupción es manejada por `HAL_GPIO_EXTI_Callback`. Dentro de esta
 * función, se implementa un **mecanismo anti-rebote por software**
 * basado en el tiempo (`HAL_GetTick()`) para ignorar los rebotes mecánicos
 * que no fueron completamente filtrados por el hardware.
 * -   Al detectar una pulsación válida, el código actualiza un índice para
 * seleccionar un nuevo valor de FPS de un arreglo predefinido y llama a
 * `update_timer_for_fps()`.
 * -   La función `update_timer_for_fps()` recalcula y actualiza el valor del
 * registro de auto-recarga (ARR) del TIM3. Al cambiar este valor, se
 * modifica la frecuencia de la interrupción, y por ende, la velocidad de
 * refresco del display.
 *
 ******************************************************************************
 */
/* USER CODE END Header */


/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// --- Variables Globales ---

// Patrones de bits para los números del 0 al 9 en un display de 7 segmentos (cátodo común).
// El orden de los bits es G-F-E-D-C-B-A.
const uint8_t seven_seg_patterns[10] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};

// Variable volátil para almacenar el valor actual del encoder.
volatile uint16_t counter_value = 0; // Inicializado en 0.

// Variable para controlar qué dígito del display se está actualizando (0 a 3).
volatile uint8_t active_digit = 0;

// Niveles de FPS (Frames Per Second) para el refresco del display.
const uint8_t fps_levels[] = {5, 10, 20, 30, 60};
uint8_t fps_index = 4; // Inicia en el nivel más alto (60 FPS).
const uint8_t num_fps_levels = sizeof(fps_levels) / sizeof(fps_levels[0]);

// Variables para el debouncing (antirrebote) del botón.
uint32_t last_button_press_time = 0;
const uint32_t debounce_delay = 200; // 200 milisegundos.
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void clear_display(void);
void set_segments(uint8_t number);
void set_active_digit(uint8_t digit_num);
void update_timer_for_fps(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @retval int
  */
int main(void)
{
  /* Reset de todos los periféricos, inicialización de la Flash y el Systick. */
  HAL_Init();

  /* Configuración del reloj del sistema. */
  SystemClock_Config();

  /* Inicialización de todos los periféricos configurados. */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // Inicia el Timer 2 en modo Encoder.
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  // Establece el valor inicial del contador del encoder en 0.
  __HAL_TIM_SET_COUNTER(&htim2, 0);

  // Inicia el Timer 3 para generar interrupciones para el refresco del display.
  HAL_TIM_Base_Start_IT(&htim3);

  // Configura la frecuencia de refresco inicial del display.
  update_timer_for_fps();
  /* USER CODE END 2 */

  /* Bucle infinito */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // La CPU lee directamente el registro del contador del timer.
    // El hardware del TIM2 se encarga del trabajo de incrementar/decrementar
    // y de gestionar el desbordamiento (rollover) de 4095 a 0 y viceversa.
    counter_value = __HAL_TIM_GET_COUNTER(&htim2);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief Configuración del Reloj del Sistema.
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configura el voltaje de salida del regulador interno principal.
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Inicializa los osciladores RCC según los parámetros especificados.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Inicializa los relojes de los buses de CPU, AHB y APB.
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Función de Inicialización de TIM2 (Encoder).
  * @param  None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4095; // El contador irá de 0 a 4095.
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Función de Inicialización de TIM3 (Refresco del Display).
  * @param  None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9; // El valor se actualizará dinámicamente.
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Función de Inicialización de GPIO.
  * @param  None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Habilitación del reloj para los puertos GPIO */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configura el nivel de salida de los pines GPIO */
  HAL_GPIO_WritePin(GPIOA, DIGIT_3_Pin|DIGIT_2_Pin|DIGIT_1_Pin|DIGIT_4_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, SEG_A_Pin|SEG_E_Pin|SEG_F_Pin|SEG_B_Pin|SEG_G_Pin|SEG_C_Pin|SEG_D_Pin, GPIO_PIN_SET);

  /* Configura el pin GPIO : PA5 (Botón para cambiar FPS) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configura los pines GPIO : DIGIT_3_Pin DIGIT_2_Pin DIGIT_1_Pin DIGIT_4_Pin */
  GPIO_InitStruct.Pin = DIGIT_3_Pin|DIGIT_2_Pin|DIGIT_1_Pin|DIGIT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configura los pines GPIO : SEG_A_Pin a SEG_G_Pin */
  GPIO_InitStruct.Pin = SEG_A_Pin|SEG_E_Pin|SEG_F_Pin|SEG_B_Pin|SEG_G_Pin|SEG_C_Pin|SEG_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Inicialización de la interrupción EXTI */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Callback de interrupción externa (se ejecuta al presionar un botón).
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Asegurarse de que la interrupción fue causada por el pin correcto (PA5).
    if (GPIO_Pin == GPIO_PIN_5) {
        // Lógica de antirrebote (debounce).
        if (HAL_GetTick() - last_button_press_time > debounce_delay) {
            fps_index = (fps_index + 1) % num_fps_levels; // Cicla entre los niveles de FPS.
            update_timer_for_fps(); // Actualiza la configuración del timer.
            last_button_press_time = HAL_GetTick(); // Registra el tiempo de la pulsación.
        }
    }
}

/**
  * @brief  Callback de interrupción del timer (se ejecuta cuando el contador del TIM3 llega al valor de Period).
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Asegurarse de que la interrupción es del TIM3.
    if (htim->Instance == TIM3) {
        clear_display(); // Apaga todos los segmentos y dígitos para evitar "ghosting".

        uint16_t temp_counter = counter_value;
        uint8_t digit_to_show;

        // Lógica de multiplexación: calcula qué número mostrar en el dígito activo.
        if (active_digit == 0)      digit_to_show = (temp_counter / 1000) % 10;
        else if (active_digit == 1) digit_to_show = (temp_counter / 100) % 10;
        else if (active_digit == 2) digit_to_show = (temp_counter / 10) % 10;
        else                        digit_to_show = temp_counter % 10;

        set_segments(digit_to_show);    // Enciende los segmentos correspondientes al número.
        set_active_digit(active_digit); // Activa el transistor del dígito actual.

        active_digit = (active_digit + 1) % 4; // Pasa al siguiente dígito para la próxima interrupción.
    }
}

/**
  * @brief  Actualiza el valor de recarga automática (ARR) del TIM3 para cambiar los FPS.
  */
void update_timer_for_fps(void) {
    // La frecuencia de refresco de CADA dígito es FPS * 4 (porque son 4 dígitos).
    uint32_t refresh_freq = fps_levels[fps_index] * 4;
    if (refresh_freq == 0) return; // Evita división por cero.

    // El reloj del TIM3 después del prescaler es 100MHz / (9999+1) = 10,000 Hz.
    // Se debe consultar la configuración del reloj para el valor exacto.
    // Nuevo Periodo (ARR) = (Frecuencia_Timer / Frecuencia_Deseada) - 1.
    uint32_t new_arr_value = (10000 / refresh_freq) - 1;

    __HAL_TIM_SET_AUTORELOAD(&htim3, new_arr_value);
    htim3.Instance->EGR = TIM_EGR_UG; // Genera un evento de actualización para aplicar los cambios inmediatamente.
}

/**
  * @brief Apaga todos los segmentos y transistores de los dígitos.
  */
void clear_display(void) {
    // Pone en ALTO los pines de los segmentos (ánodo común, se apagan con '1').
    HAL_GPIO_WritePin(GPIOB, SEG_A_Pin|SEG_E_Pin|SEG_F_Pin|SEG_B_Pin|SEG_G_Pin|SEG_C_Pin|SEG_D_Pin, GPIO_PIN_SET);
    // Pone en ALTO los pines de los dígitos (transistores PNP, se apagan con '1').
    HAL_GPIO_WritePin(GPIOA, DIGIT_1_Pin|DIGIT_2_Pin|DIGIT_3_Pin|DIGIT_4_Pin, GPIO_PIN_SET);
}

/**
  * @brief Muestra un número (0-9) en los segmentos.
  */
void set_segments(uint8_t number) {
    if (number > 9) return;
    uint8_t pattern = seven_seg_patterns[number];

    HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, (pattern & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, (pattern & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, (pattern & 0x04) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, (pattern & 0x08) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, (pattern & 0x10) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, (pattern & 0x20) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, (pattern & 0x40) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
  * @brief Activa un dígito específico (0 a 3).
  */
void set_active_digit(uint8_t digit_num) {
    // Pone en BAJO el pin del transistor PNP correspondiente para permitir que la corriente fluya.
    switch (digit_num) {
        case 0: HAL_GPIO_WritePin(DIGIT_1_GPIO_Port, DIGIT_1_Pin, GPIO_PIN_RESET); break;
        case 1: HAL_GPIO_WritePin(DIGIT_2_GPIO_Port, DIGIT_2_Pin, GPIO_PIN_RESET); break;
        case 2: HAL_GPIO_WritePin(DIGIT_3_GPIO_Port, DIGIT_3_Pin, GPIO_PIN_RESET); break;
        case 3: HAL_GPIO_WritePin(DIGIT_4_GPIO_Port, DIGIT_4_Pin, GPIO_PIN_RESET); break;
    }
}
/* USER CODE END 4 */

/**
  * @brief  Esta función se ejecuta en caso de que ocurra un error.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reporta el nombre del archivo fuente y el número de línea
  * donde ha ocurrido el error assert_param.
  * @param  file: puntero al nombre del archivo fuente.
  * @param  line: número de línea del error.
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
