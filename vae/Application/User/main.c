/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/main.c
  * @author  MCD Application Team
  * @brief   Nodo HIL: la scheda riceve lo stato dal PC e risponde con le azioni.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ciclo_seriale.h"
#include "npu_cache.h"
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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
#if defined(__ICCARM__)
__ATTRIBUTES size_t __write(int, const unsigned char *, size_t);
#endif /* __ICCARM__ */

#if defined(__ICCARM__)
/* New definition from EWARM V9, compatible with EWARM8 */
int iar_fputc(int ch);
#define PUTCHAR_PROTOTYPE int iar_fputc(int ch)
#elif defined ( __CC_ARM ) || defined(__ARMCC_VERSION)
/* ARM Compiler 5/6*/
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif defined(__GNUC__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#endif /* __ICCARM__ */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* I DUE GANCI __weak DI ST, E PERCHE' VANNO QUI E NON DENTRO main().
 *
 * npu_cache.c li dichiara __weak con corpo vuoto: e' l'utente che deve
 * implementarli, perche' solo lui sa come si danno clock e reset sulla
 * sua scheda. npu_cache_enable() e npu_cache_disable() li chiamano.
 *
 * Fino al 2/10 la definizione stava DENTRO main(): una funzione annidata,
 * estensione GCC, visibile solo in quel blocco e senza linkage esterno. Il
 * linker non la vedeva, il simbolo debole restava quello vuoto di ST, e il
 * CACHEAXI non ha mai preso il clock. A livello di file, invece,
 * sovrascrive il debole come si deve. */
void npu_cache_enable_clocks_and_reset(void)
{
    __HAL_RCC_CACHEAXI_CLK_ENABLE();
    __HAL_RCC_CACHEAXI_FORCE_RESET();
    __HAL_RCC_CACHEAXI_RELEASE_RESET();
}

void npu_cache_disable_clocks_and_reset(void)
{
    __HAL_RCC_CACHEAXI_FORCE_RESET();
    __HAL_RCC_CACHEAXI_RELEASE_RESET();
    __HAL_RCC_CACHEAXI_CLK_DISABLE();
}

/* Apre il firewall RISAF su una regione. E' set_risaf_default() di ST,
 * da STEdgeAI/4.0/Projects/STM32N6570-DK/Applications/hello_world. Due
 * regioni interamente sovrapposte, una per gli accessi secure e una per i
 * non secure, con tutti i domini ammessi in lettura e scrittura. */
static void risaf_apri(RISAF_TypeDef *risaf, uint32_t fine)
{
    RISAF_BaseRegionConfig_t c;

    c.StartAddress   = 0x0u;
    c.EndAddress     = fine;
    c.Filtering      = RISAF_FILTER_ENABLE;
    c.PrivWhitelist  = RIF_CID_NONE;
    c.ReadWhitelist  = RIF_CID_MASK;
    c.WriteWhitelist = RIF_CID_MASK;

    c.Secure = RIF_ATTRIBUTE_SEC;
    HAL_RIF_RISAF_ConfigBaseRegion(risaf, 0, &c);
    c.Secure = RIF_ATTRIBUTE_NSEC;
    HAL_RIF_RISAF_ConfigBaseRegion(risaf, 1, &c);
}

/* Accende l'acceleratore.
 *
 * Ricostruita sulla sequenza di ST in hello_world, che e' il loro esempio
 * minimo con una rete sulla NPU: system_init_post() + NPU_Config() +
 * RISAF_Config(), tolte camera, display e memorie esterne. Nel loro main()
 * l'ordine e' NPU_Config() e poi RISAF_Config(), e conta: il commento di
 * ST dice che un IP va messo sotto clock PRIMA di configurargli il RISAF.
 *
 * QUATTRO cose servono, e averne due faceva sembrare che funzionasse.
 *
 * Per ogni banco di memoria:
 *   il CLOCK        senza, il bus si pianta e il debugger perde il bersaglio
 *   l'ALIMENTAZIONE senza, il banco risponde ma e' morto
 *
 * Per l'acceleratore:
 *   il CLOCK e poi il RESET DA RILASCIARE
 *   gli ATTRIBUTI RIF/RISAF. Questa e' la quarta, ed e' quella che ci ha
 *   fermato fino al 2/10.
 *
 * La NPU e' un MASTER del bus, e i master passano per il firewall. Senza
 * HAL_RIF_RIMC_ConfigMasterAttributes le sue transazioni non sono secure
 * privileged; senza i RISAF su AXISRAM 3-6 e sulle due porte master, le
 * sue scritture vengono scartate. Senza fault, senza un errore: le epoch
 * hardware "finiscono" subito e stai_network_run torna STAI_SUCCESS.
 *
 * Il 2/10 questo dava: le 36 epoch hardware non scrivevano UN byte, le 10
 * software sul M55 giravano regolarmente, e le quattro azioni che
 * leggevamo erano una costante — la coda software applicata a un buffer
 * di attivazioni che il nostro memset aveva azzerato. Misurato: a
 * 0x3435c080 (uscita dell'epoch 3, hardware) sessantaquattro byte a zero,
 * con zero-point -82; a 0x3435c480 (gli scalari, scritti dalla CPU) i
 * nostri dati, giusti.
 *
 * Per uni_arco_v6 non serviva niente di tutto questo: girava sul M55, e
 * per questo il difetto e' rimasto invisibile fino al primo vero uso
 * della NPU. */
static void npu_accendi(void)
{
    /* 1. I BANCHI. Clock e uscita dallo spegnimento. 3, 4, 5 tengono i
     *    pesi; 6 le attivazioni; CACHEAXIRAM la cache della NPU.
     *    Scritture dirette su registro, come fa system_init_post(): non
     *    serve il modulo HAL RAMCFG. */
    RCC->MEMENR |= RCC_MEMENR_AXISRAM3EN | RCC_MEMENR_AXISRAM4EN
                 | RCC_MEMENR_AXISRAM5EN | RCC_MEMENR_AXISRAM6EN
                 | RCC_MEMENR_CACHEAXIRAMEN;

    RAMCFG_SRAM3_AXI->CR &= ~RAMCFG_CR_SRAMSD;
    RAMCFG_SRAM4_AXI->CR &= ~RAMCFG_CR_SRAMSD;
    RAMCFG_SRAM5_AXI->CR &= ~RAMCFG_CR_SRAMSD;
    RAMCFG_SRAM6_AXI->CR &= ~RAMCFG_CR_SRAMSD;

    /* 2. L'ACCELERATORE. Clock, e poi fuori dal reset: l'ordine conta,
     *    il reset si rilascia a clock gia' presente. */
    __HAL_RCC_NPU_CLK_ENABLE();
    __HAL_RCC_NPU_FORCE_RESET();
    __HAL_RCC_NPU_RELEASE_RESET();

    /* La cache della NPU, accesa come fa ST con USE_NPU_CACHE definito.
     * npu_cache_enable() chiama da se' il gancio qui sopra. */
    npu_cache_enable();

    /* 3. LA NPU COME MASTER: secure, privilegiata, dominio CID 1. */
    {
        RIMC_MasterConfig_t m;

        m.MasterCID = RIF_CID_1;
        m.SecPriv   = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;
        HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_NPU, &m);
        HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_NPU,
                                              RIF_ATTRIBUTE_PRIV | RIF_ATTRIBUTE_SEC);
    }

    /* 4. I FIREWALL sulla memoria che la NPU deve attraversare.
     *
     * NON tocchiamo RISAF2_S e RISAF3_S (AXISRAM1 e AXISRAM2): la' dentro
     * a 0x34180400 vive il nostro programma, e la NPU non ci entra mai.
     * ST li configura perche' la loro applicazione e' piu' grande; a noi
     * servirebbe solo a riconfigurare il firewall della memoria da cui
     * stiamo eseguendo. */
    risaf_apri(RISAF4_S,  RISAF4_LIMIT_ADDRESS_SPACE_SIZE);   /* NPU MST0        */
    risaf_apri(RISAF5_S,  RISAF5_LIMIT_ADDRESS_SPACE_SIZE);   /* NPU MST1        */
    risaf_apri(RISAF6_S,  RISAF6_LIMIT_ADDRESS_SPACE_SIZE);   /* AXISRAM 3,4,5,6 */
    risaf_apri(RISAF8_S,  RISAF8_LIMIT_ADDRESS_SPACE_SIZE);   /* NPU_CACHE       */
    risaf_apri(RISAF15_S, RISAF15_LIMIT_ADDRESS_SPACE_SIZE);  /* NPU_CACHE conf  */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* STM32N6xx HAL library initialization:
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Initialize BSP Led for LED2 */
  BSP_LED_Init(LED2);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* L'ordine conta: prima si accende l'acceleratore, poi il ciclo seriale
   * apre la rete. ciclo_seriale_init() chiama stai_network_init(), che
   * tocca subito la memoria dei pesi. */
  npu_accendi();

  if (ciclo_seriale_init(&huart1) != 0)
  {
    Error_Handler();
  }
  ciclo_seriale_gira();          /* non torna mai */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}
/* USER CODE BEGIN CLK 1 */
/* USER CODE END CLK 1 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the System Power Supply
  */
  if (HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable HSI */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Get current CPU/System buses clocks configuration and if necessary switch
 to intermediate HSI clock to ensure target clock can be set
  */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
  if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1) ||
     (RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11))
  {
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK);
    RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL1.PLLM = 4;
  RCC_OscInitStruct.PLL1.PLLN = 75;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;
  RCC_OscInitStruct.PLL1.PLLP1 = 1;
  RCC_OscInitStruct.PLL1.PLLP2 = 1;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_CPUCLK|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2|RCC_CLOCKTYPE_PCLK5
                              |RCC_CLOCKTYPE_PCLK4;
  RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;
  RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC1Selection.ClockDivider = 2;
  RCC_ClkInitStruct.IC2Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC2Selection.ClockDivider = 3;
  RCC_ClkInitStruct.IC6Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC6Selection.ClockDivider = 4;
  RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC11Selection.ClockDivider = 3;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

#if defined(__ICCARM__)
size_t __write(int file, unsigned char const *ptr, size_t len)
{
  size_t idx;
  unsigned char const *pdata = ptr;

  for (idx = 0; idx < len; idx++)
  {
    iar_fputc((int)*pdata);
    pdata++;
  }
  return len;
}
#endif /* __ICCARM__ */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while (1);
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

  /* Infinite loop */
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
