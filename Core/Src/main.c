/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions

#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "BME280_STM32.h"
#include "icm20948.h"
#include "math.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define SPI_CS_GPIO_Port GPIOB
#define SPI_CS_Pin GPIO_PIN_0

//Variables for FatFs deplaced here to be global and accessible in the whole file, 
//including in the main loop for file operations
//Evite aussi le Stack overflow qui peut arriver si on déclare ces grosses structures dans la fonction main
FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations

volatile uint8_t stop_logging = 0; // pour le BP d'arrêt d'urgence
volatile uint32_t CTOP = 0; //Compteur de mesures pour limiter le nombre de fichiers créés sur la carte SD

/*------------Pour le gyroscope ICM20948------------*/
axises gyrodata;
axises acceldata;
axises magdata;

#define CENTER_X 30
#define CENTER_Y 40
#define RADIUS   18

#define HORIZON_CENTER_X 100
#define HORIZON_CENTER_Y 40
#define HORIZON_RADIUS   18

#define MAG_OFFSET_X 212.50f
#define MAG_OFFSET_Y 261.00f
#define MAG_OFFSET_Z 283.50f
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
void draw_compass(float angle_degrees);
void draw_artificial_horizon(float pitch, float roll);
uint32_t calcul_rapport_cyclique(float yaw);
//--------------- Fonctions pour l'ARINC 429 ---------------
uint8_t inversion_byte(uint8_t byte);
int count_set_bits(uint32_t n);
uint32_t generate_arinc_word(uint32_t pressure, uint32_t label);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}
//Pur travail de l'IA; pour avoir une visualisation de l'orientation du gyroscope ICM20948
void draw_compass(float angle_degrees) {
    // 1. Convertir l'angle en radians
    float angle_rad = angle_degrees * (M_PI / 180.0f);

    // 2. Calculer les coordonnées du bout de l'aiguille
    int x_target = CENTER_X + (int)(RADIUS * sin(angle_rad));
    int y_target = CENTER_Y - (int)(RADIUS * cos(angle_rad)); // Moins car le Y de l'écran descend

    // 3. Dessiner le cadran du compas (Cercle extérieur)
    ssd1306_DrawCircle(CENTER_X, CENTER_Y, RADIUS, White);
    
    // 4. Dessiner un petit repère pour le "Nord" (Le haut de l'écran)
    ssd1306_Line(CENTER_X, CENTER_Y - RADIUS, CENTER_X, CENTER_Y - RADIUS + 5, White);

    // 5. Dessiner l'aiguille (Une simple ligne du centre vers le bord)
    ssd1306_Line(CENTER_X, CENTER_Y, x_target, y_target, White);
}
void draw_artificial_horizon(float pitch, float roll) {
    // 1. Convertir le roulis en radians
    float roll_rad = roll * (M_PI / 180.0f);

    // 2. Calculer le décalage vertical dû au tangage
    // Le facteur 0.5 est ajustable selon la sensibilité visuelle voulue
    int y_offset = (int)(pitch * 0.5f); 
    
    // On limite le décalage pour que la ligne ne sorte pas trop du cercle
    if (y_offset > HORIZON_RADIUS) y_offset = HORIZON_RADIUS;
    if (y_offset < -HORIZON_RADIUS) y_offset = -HORIZON_RADIUS;

    int center_y_line = HORIZON_CENTER_Y + y_offset;

    // 3. Calculer les décalages X et Y pour les extrémités de la ligne
    int dx = (int)(HORIZON_RADIUS * cos(roll_rad));
    int dy = (int)(HORIZON_RADIUS * sin(roll_rad));

    // 4. Dessiner le cadran extérieur
    ssd1306_DrawCircle(HORIZON_CENTER_X, HORIZON_CENTER_Y, HORIZON_RADIUS, White);
    
    // 5. Dessiner un point central statique (représente ton système/avion)
    ssd1306_Line(HORIZON_CENTER_X - 5, HORIZON_CENTER_Y, HORIZON_CENTER_X + 5, HORIZON_CENTER_Y, White);

    // 6. Dessiner la ligne d'horizon (qui pivote et monte/descend)
    ssd1306_Line(HORIZON_CENTER_X - dx, center_y_line - dy, HORIZON_CENTER_X + dx, center_y_line + dy, White);
}

uint32_t calcul_rapport_cyclique(float yaw){
    // On sait que le yaw varie entre -180° et +180°
    // On veut un rapport cyclique entre 5% de 20 ms = 1 ms (pour -180°) -> seroM à -90°
    // et 10% de 20 ms = 2 ms (pour +180°) -> servoM à +90°
    float nb_ticks = 1500 + (yaw / 360) * 1000;
    if (nb_ticks < 1000) nb_ticks = 1000;   // Limite inférieure
    if (nb_ticks > 2000) nb_ticks = 2000; // Limite supérieure

    return (uint32_t)nb_ticks;
}

/*-------------------------------
Fonctions pour l'ARINC 429
-------------------------------*/

uint8_t inversion_byte(uint8_t byte){
  //L'ARINC 429 inverse le balel, il donne d'abord le MSB et à la fin le LSB
  //Si notre label est 429 = 110101101, on doit envoyer 101101011 (0x5B) et pas 110101101 (0x6D)
  byte = (byte & 0xF0) >> 4 | (byte & 0x0F) << 4;
  byte = (byte & 0xCC) >> 2 | (byte & 0x33) << 2;
  byte = (byte & 0xAA) >> 1 | (byte & 0x55) << 1;
  //Exemple avec : 7654 3210
  //byte1 : 3210 7654
  //byte2 : 1032 5476
  //byte3 : 0123 4567
  return byte;
}

// Fonction pour compter le nombre de bits à 1 dans un entier (utile pour le bit de parité)
int count_set_bits(uint32_t n) {
    int compte = 0;
    while (n) {
        compte += n & 1;
        n >>= 1;
    }
    return compte;
}

uint32_t generate_arinc_word(uint32_t pressure, uint32_t label) {
    
    uint32_t arinc_word = 0; // On part d'un mot vide : 0000 0000 0000 0000 0000 0000 0000 0000

    // Le label (Bits 1 à 8)
    uint8_t label_reversed = inversion_byte(label);
    // On implémente ce label dans les 8 premiers bits du mot ARINC  
    arinc_word = arinc_word | label_reversed; 

    // SDI (Bits 9 et 10)
    // L'énoncé demande 0, donc on ne fait rien

    // La donnée pure (Bits 11 à 28)
    uint32_t data_masked = pressure & 0x3FFFF; // Sécurité pour ne garder que 18 bits de données, sans débordement
    arinc_word = arinc_word | (data_masked << 10);

    // SSM (Bits 30 et 31)
    // L'énoncé demande 0. On ne fait rienn.

    // Bit de parité (Bit 32)
    int ones_count = count_set_bits(arinc_word);
    
    // Si le nombre de '1' est pair, on doit mettre le 32ème bit à '1' pour que le total devienne impair.
    if (ones_count % 2 == 0) {
        arinc_word = arinc_word | ((uint32_t)1 << 31); 
        //uint32_t sur le '1' nécessaire apparemment pour ne pas avoir un bit signé qui causerait des problèmes lors du décalage
    }

    return arinc_word;
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
  MX_SPI3_Init();
  MX_FATFS_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  //Initialisation de l'écran OLED
  ssd1306_Init();

  /*------------------------------------------
  Capteur de temperature et de pression BMP280
  --------------------------------------------*/
 int status = BME280_Config (OSRS_2, OSRS_16, MODE_NORMAL, T_SB_0p5, IIR_16);
  if (status!= 0)
    {
  	  Error_Handler();
    }

  BME280_WakeUP();
  float temp=0;
  float press=0;
 
  /*------------------------------------------
  Carte SD + FatFS demo
  --------------------------------------------*/
  myprintf("TEST SPI EN COURS ...\r\n");

  //Force CS high
  //SD cards require CS to be high when power is applied
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  // Envoie 10 octets 0xFF et affiche ce qu'on reçoit
  /*La norme exige de fournir un minimum de 74 impulsions d'horloge 
  avec la ligne de transmission (MOSI) maintenue à l'état haut (1) 
  avant de pouvoir envoyer la toute première commande.*/
  uint8_t tx = 0xFF, rx = 0x00;
  myprintf("Dummy bytes received: ");
  for(int i = 0; i < 10; i++) {
	  HAL_SPI_TransmitReceive(&hspi3, &tx, &rx, 1, 100);
	  myprintf("%02X ", rx);
  }
  myprintf("\r\n");

  // Force CS low et envoie CMD0 pour réveiller la carte SD
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  uint8_t cmd0[] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95}; //Norme de réveil pour les SD, 0x95 est le CRC correct pour CMD0
  uint8_t resp[7] = {0};
  HAL_SPI_Transmit(&hspi3, cmd0, 6, 100);

  // Lire 7 octets de réponse
  for(int i = 0; i < 7; i++) {
	  HAL_SPI_TransmitReceive(&hspi3, &tx, &resp[i], 1, 100);
	  myprintf("resp[%d] = %02X\r\n", i, resp[i]);
  }
  //Doit renvoyer "01" pour indiquer que le SD est en mode idle, prêt à recevoir des commandes

  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  myprintf("\r\n~ Fin de l'initialisation ~\r\n\r\n");

  HAL_Delay(1000); //a short delay is important to let the SD card settle

  //declaration of variables for FatFs deplaced to PV section
  
  //Open the file system
  fres = f_mount(&FatFs, "", 1); //1=mount now
  if (fres != FR_OK) {
  myprintf("f_mount error (%i)\r\n", fres);
  Error_Handler();
  }

  //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS* getFreeFs;
  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) {
  myprintf("f_getfree error (%i)\r\n", fres);
  Error_Handler();
  }

  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

  /* Example on how to open a file and read it :
  //Now let's try to open file "test.txt"
  fres = f_open(&fil, "TEST.TXT", FA_READ);
  if (fres != FR_OK) {
  myprintf("f_open error (%i)\r\n", fres);
  Error_Handler();
  }
  else {
  myprintf("I was able to open 'test.txt' for reading!\r\n");
  }

  //Read 30 bytes from a file on the SD card
  BYTE readBuf[30];
  
  //We can either use f_read OR f_gets to get data out of files
  //f_gets is a wrapper on f_read that does some string formatting for us
  TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
  if(rres != 0) {
  myprintf("Read string from 'test.txt' contents: %s\r\n", readBuf);
  } else {
  myprintf("f_gets error (%i)\r\n", fres);
  }

  //Be a tidy kiwi - don't forget to close your file!
  f_close(&fil); */
  /*---------------------------------*/

  /*Créer un nom de fichier unique, à partir du numéro de session précédente */
  int i = 0;
  char nom_fichier[30];
  FILINFO fno;

  // Boucle pour trouver le premier numéro disponible
  do {
      snprintf(nom_fichier, sizeof(nom_fichier), "0:/VOL_%d.CSV", i); //info*
      i++;
  } while (f_stat(nom_fichier, &fno) == FR_OK); // FR_OK = le fichier existe, on continue
  //info* : snprintf(nom_fichier, sizeof(nom_fichier), équivalent de sprintf mais avec une sécurité de la taille du buffer pour éviter les débordements de mémoire
  
  i--; // On a trouvé un nom libre
  // Donc nom de fichier unique pour cette session qui est prêt à être utilisé pour l'écriture
  myprintf("Nom de fichier unique genere : %s\r\n", nom_fichier);

  /* ----------------------------------
  Gyroscope ICM20948
  --------------------------------- */
  //Initialisation des capteurs de mouvement
  icm20948_init();
  ak09916_init();
  // Angles
  float pitch = 0;
  float roll = 0;
  float yaw = 0;
  //Variables de corrections pour l'OFFSET du magnétomètre et l'inclinaison du capteur
  float pitch_rad = 0;
  float roll_rad = 0;
  float mx = 0;
  float my = 0;
  float mz = 0;
  float mag_x_comp = 0;
  float mag_y_comp = 0;


  /*------------------------------------------
  Variable de fréquence d'acquisition
  --------------------------------------------*/
  uint32_t last_tick = HAL_GetTick();

  /*------------------------------------------
  Pilotage PWM du servomoteur 
  -------------------------------------------*/
  float rapport_cyclique = 75;
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);  // Start PWM on TIM1_CH1
  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, rapport_cyclique); // 7,5% duty cycle (1,5 ms / 20 ms) pour position = 0°

  /*------------------------------------------
    Message ARINC 429 
  --------------------------------------------*/
  uint32_t mesg_arinc = 0 ;
  uint32_t label_arinc = 0x100; //256 en héxadécimal

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  if (HAL_GetTick() - last_tick >= 200) 
  { //Acquisition toutes les 200ms
    last_tick = HAL_GetTick(); //Reset du timer pour la prochaine acquisition

    /*------------------------------------------
    Partie gyroscope ICM20948
    --------------------------------------------*/
    //Le capteur de temperature et de pression BMP208
    BME280_Measure(&temp,&press);
    
    //Le capteur de mouvement ICM20948 (gyroscope, accéléromètre et magnétomètre)
    icm20948_gyro_read(& gyrodata);
    icm20948_accel_read(& acceldata);
    ak09916_mag_read(& magdata); 
    mx = magdata.x - MAG_OFFSET_X;
    my = magdata.y - MAG_OFFSET_Y;
    mz = magdata.z - MAG_OFFSET_Z;
    // Calcul des angles
    pitch_rad = atan2(-acceldata.x, sqrt(acceldata.y * acceldata.y + acceldata.z * acceldata.z));
    roll_rad = atan2(acceldata.y, acceldata.z);
    pitch = pitch_rad * 180.0 / M_PI;
    roll = roll_rad * 180.0 / M_PI;

    mag_x_comp = mx * cos(pitch_rad) + mz * sin(pitch_rad);
    mag_y_comp = mx * sin(roll_rad) * sin(pitch_rad) + my * cos(roll_rad) - mz * sin(roll_rad) * cos(pitch_rad);
    yaw = atan2(mag_y_comp, mag_x_comp) * 180.0 / M_PI;

    /*------------------------------------------
    Partie message ARINC 429 pour la pression, à envoyer par UART et dans la microSD
    --------------------------------------------*/
    uint32_t press_dec = press * 100; //Pression*100 pour avoir les deux décimales
    mesg_arinc = generate_arinc_word(press_dec, label_arinc); 

    /*------------------------------------------
    Partie affichage Terminal Serie
    --------------------------------------------*/
    myprintf("Serial Terminal | T: %.2f C, P: %.2f hPa \r\n", temp, press);
    myprintf("Message ARINC | 0x%08lX\r\n", mesg_arinc); //%08lX repasse en hexadécimal long pour l'affichage, plus lisible pour un message binaire
    myprintf("Gyro (dps) | X: %.2f, Y: %.2f, Z: %.2f\r\n", gyrodata.x, gyrodata.y, gyrodata.z);
    myprintf("Accel (g) | X: %.2f, Y: %.2f, Z: %.2f\r\n", acceldata.x, acceldata.y, acceldata.z);
    myprintf("Mag (uT) | X: %.2f, Y: %.2f, Z: %.2f\r\n", magdata.x, magdata.y, magdata.z); 

    /*------------------------------------------
    Partie affichage Écran OLED
    --------------------------------------------*/
    ssd1306_Fill(Black);

    ssd1306_SetCursor(2, 0);
    char strPress[20];
    sprintf(strPress, "P: %.2f hPa", press);
    ssd1306_WriteString(strPress, Font_7x10, White);
    
    ssd1306_SetCursor(2, 10);
    char strTemp[20];
    sprintf(strTemp, "T: %.2f C", temp);
    ssd1306_WriteString(strTemp, Font_7x10, White);

    draw_compass(yaw);
    draw_artificial_horizon(pitch, roll);

    ssd1306_UpdateScreen();

    /*------------------------------------------
    Partie écriture sur carte SD
    --------------------------------------------*/

    //Now let's try and write a file "write.txt"
    fres = f_open(&fil, nom_fichier, FA_WRITE | FA_OPEN_ALWAYS | FA_OPEN_APPEND);
    if(fres == FR_OK) {
    myprintf("SD | Ouverture de %s pour écriture\r\n", nom_fichier);
    } else {
    myprintf("SD | Erreur f_open (%i)\r\n", fres);
    }

    char line[250];
    //Copy in a string
    //Format CSV : "temp;press;etc" car Excel sépare grace au "";"
    snprintf(line, sizeof(line), "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;0x%08lX\r\n", temp, press, gyrodata.x, gyrodata.y, gyrodata.z, acceldata.x, acceldata.y, acceldata.z, magdata.x, magdata.y, magdata.z, mesg_arinc);
    UINT bytesWrote;
    fres = f_write(&fil, line, strlen(line), &bytesWrote);
    if(fres == FR_OK) {
    myprintf("SD | Wrote %i bytes to %s!\r\n", bytesWrote, nom_fichier);
    } else {
    myprintf("SD | f_write error (%i)\r\n", fres);
    }

    //Be a tidy kiwi - don't forget to close your file!
    f_close(&fil);

    /*------------------------------------------
    Pilotage servomoteur
    --------------------------------------------*/
    rapport_cyclique = calcul_rapport_cyclique(yaw);
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, rapport_cyclique); 

    /*------------------------------------------
    LEDs d'alterte sur dépassement de seuils (40°) pour le tangage et le roulis
    --------------------------------------------*/
    // Vérification du Tangage (Pitch)
    if (fabs(pitch) > 40.0f) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    }

    // Vérification du Roulis (Roll)
    if (fabs(roll) > 40.0f) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    }

    /*------------------------------------------
    Partie de contrôle d'arrêt d'urgence et de limitation du nombre de mesures
    --------------------------------------------*/

    CTOP++;
    if (CTOP > 300 || stop_logging == 1) { //On s'arrête après 300 mesures = 1 min pour éviter de remplir la carte SD
      myprintf("SD | BP presse ou limite (300) atteinte, arrêt de la journalisation.\r\n");
      break;  
    }
  }
  
  }
  //demontage de la carte SD pour éviter les corruptions de données
  f_mount(NULL, "", 0);
  myprintf("SD | Carte SD demontee en toute securite.\r\n");

  // Bloque le processeur ici indefiniment
  while(1) {
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x10D19CE4;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 79;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 19999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_pitch_Pin|GPIO_roll_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : GPIO_pitch_Pin GPIO_roll_Pin */
  GPIO_InitStruct.Pin = GPIO_pitch_Pin|GPIO_roll_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BP_GPIO_EXTI8_Pin */
  GPIO_InitStruct.Pin = BP_GPIO_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BP_GPIO_EXTI8_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == BP_GPIO_EXTI8_Pin) {
    stop_logging = 1; // On lève le drapeau
  }
}
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
