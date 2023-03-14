/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  * IDGM_Test_11d_2-IGBT_AP
  *
  * Sakurai Lab., The University of Tokyo
  * Rev.00 2023/03/14 First release
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART_RX_BUFFSIZE 1024			//UART受信バッファ
#define USART_TX_BUFFSIZE 1024			//UART送信バッファ
#define ADC_BUFFSIZE 120				//ADC受信バッファサイズ, 22.5us=12.5ns*15*120

#define CMD_OK				"OK\r\n"		//コマンドOK応答文字列
#define CMD_ERR 			"ERR\r\n"		//コマンドエラー応答文字列
//コマンド文字列
#define CMD_SET_CH_No	 	"ch"			//ADCチャンネル設定
#define CMD_SET_ADC_RESO	"ar"			//ADC 解像度
#define CMD_SET_ADC_SMPLTIME	"as"		//ADC サンプリング時間
#define CMD_SET_ADC_DELAY	"ad"			//ADC測定開始遅延時間
#define CMD_SET_PWM_ARR1	"pa1"			//ダブルパルスの1つ目のARRレジスタ値
#define CMD_SET_PWM_ARR2	"pa2"			//ダブルパルスの2つ目のARRレジスタ値
#define CMD_SET_PWM_CCR1	"pc1"			//ダブルパルスの1つ目のCCRレジスタ値
#define CMD_SET_PWM_CCR2	"pc2"			//ダブルパルスの2つ目のCCRレジスタ値
#define CMD_DSP_PARA		"dsp"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim1_up;
DMA_HandleTypeDef hdma_tim3_ch4_up;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
// Potentiometer MCP4461-503E_ML
static const uint8_t MCP4461_ADDR = 0x2E << 1;
static const uint8_t REG_NVWiper0 = 0x02;
uint8_t txBuffer4461 = 0xF6;//Vg=15V
// Expander with Configuration Registers TCA9536ADTMR
static const uint8_t TCA9536A_ADDR = 0x40 << 1;
static const uint8_t REG_Configuration = 0x03;
static const uint8_t REG_OutputPort = 0x01;
uint8_t txBuffer9536AoEB = 0xF0;//OutputEnableB
//uint8_t txBuffer9536A = 0xF7;//300Ω
uint8_t txBuffer9536A = 0xFB;//100Ω
//uint8_t txBuffer9536A = 0xFD;//10Ω
//uint8_t txBuffer9536A = 0xFE;//1Ω
//uint8_t txBuffer9536A = 0xFC;//10Ω, 1Ω
//uint8_t txBuffer9536A = 0xFA;//100Ω, 1Ω
//uint8_t txBuffer9536A = 0xF9;//100Ω, 10Ω
//uint8_t txBuffer9536A = 0xF6;//300Ω, 1Ω
//uint8_t txBuffer9536A = 0xF5;//300Ω, 10Ω
//uint8_t txBuffer9536A = 0xF3;//300Ω, 100Ω
//uint8_t txBuffer9536A = 0xF8;//100Ω, 10Ω, 1Ω
//uint8_t txBuffer9536A = 0xF4;//300Ω, 10Ω, 1Ω
//uint8_t txBuffer9536A = 0xF2;//300Ω, 100Ω, 1Ω
//uint8_t txBuffer9536A = 0xF1;//300Ω, 100Ω, 10Ω
//uint8_t txBuffer9536A = 0xF0;//全ON
//uint8_t txBuffer9536A = 0xFF;//全OFF, High-Z
// ADC data Buffer
uint16_t ADCValues00[ADC_BUFFSIZE];			//ADC測定データ格納バッファ00
uint16_t ADCValues01[ADC_BUFFSIZE];			//ADC測定データ格納バッファ01
uint16_t ADCValues02[ADC_BUFFSIZE];			//ADC測定データ格納バッファ02
uint16_t ADCValues03[ADC_BUFFSIZE];			//ADC測定データ格納バッファ03
uint16_t ADCValues04[ADC_BUFFSIZE];			//ADC測定データ格納バッファ04
uint16_t ADCValues05[ADC_BUFFSIZE];			//ADC測定データ格納バッファ05
uint16_t ADCValues06[ADC_BUFFSIZE];			//ADC測定データ格納バッファ06
uint16_t ADCValues07[ADC_BUFFSIZE];			//ADC測定データ格納バッファ07
uint16_t ADCValues08[ADC_BUFFSIZE];			//ADC測定データ格納バッファ08
uint16_t ADCValues09[ADC_BUFFSIZE];			//ADC測定データ格納バッファ09

uint16_t ADCValues10[ADC_BUFFSIZE];			//ADC測定データ格納バッファ10
uint16_t ADCValues11[ADC_BUFFSIZE];			//ADC測定データ格納バッファ11
uint16_t ADCValues12[ADC_BUFFSIZE];			//ADC測定データ格納バッファ12
uint16_t ADCValues13[ADC_BUFFSIZE];			//ADC測定データ格納バッファ13
uint16_t ADCValues14[ADC_BUFFSIZE];			//ADC測定データ格納バッファ14
uint16_t ADCValues15[ADC_BUFFSIZE];			//ADC測定データ格納バッファ15
uint16_t ADCValues16[ADC_BUFFSIZE];			//ADC測定データ格納バッファ16
uint16_t ADCValues17[ADC_BUFFSIZE];			//ADC測定データ格納バッファ17
uint16_t ADCValues18[ADC_BUFFSIZE];			//ADC測定データ格納バッファ18
uint16_t ADCValues19[ADC_BUFFSIZE];			//ADC測定データ格納バッファ19

uint16_t ADCValues20[ADC_BUFFSIZE];			//ADC測定データ格納バッファ20
uint16_t ADCValues21[ADC_BUFFSIZE];			//ADC測定データ格納バッファ21
uint16_t ADCValues22[ADC_BUFFSIZE];			//ADC測定データ格納バッファ22
uint16_t ADCValues23[ADC_BUFFSIZE];			//ADC測定データ格納バッファ23
uint16_t ADCValues24[ADC_BUFFSIZE];			//ADC測定データ格納バッファ24
uint16_t ADCValues25[ADC_BUFFSIZE];			//ADC測定データ格納バッファ25
uint16_t ADCValues26[ADC_BUFFSIZE];			//ADC測定データ格納バッファ26
uint16_t ADCValues27[ADC_BUFFSIZE];			//ADC測定データ格納バッファ27
uint16_t ADCValues28[ADC_BUFFSIZE];			//ADC測定データ格納バッファ28
uint16_t ADCValues29[ADC_BUFFSIZE];			//ADC測定データ格納バッファ29
uint16_t ADCValues30[ADC_BUFFSIZE];			//ADC測定データ格納バッファ30
uint16_t ADCValues31[ADC_BUFFSIZE];			//ADC測定データ格納バッファ31
uint16_t ADCValues32[ADC_BUFFSIZE];			//ADC測定データ格納バッファ32
uint16_t ADCValues33[ADC_BUFFSIZE];			//ADC測定データ格納バッファ33
uint16_t ADCValues34[ADC_BUFFSIZE];			//ADC測定データ格納バッファ34
uint16_t ADCValues35[ADC_BUFFSIZE];			//ADC測定データ格納バッファ35
uint16_t ADCValues36[ADC_BUFFSIZE];			//ADC測定データ格納バッファ36
uint16_t ADCValues37[ADC_BUFFSIZE];			//ADC測定データ格納バッファ37
uint16_t ADCValues38[ADC_BUFFSIZE];			//ADC測定データ格納バッファ38
uint16_t ADCValues39[ADC_BUFFSIZE];			//ADC測定データ格納バッファ39

uint16_t ADCValues40[ADC_BUFFSIZE];			//ADC測定データ格納バッファ40
uint16_t ADCValues41[ADC_BUFFSIZE];			//ADC測定データ格納バッファ41
uint16_t ADCValues42[ADC_BUFFSIZE];			//ADC測定データ格納バッファ42
uint16_t ADCValues43[ADC_BUFFSIZE];			//ADC測定データ格納バッファ43
uint16_t ADCValues44[ADC_BUFFSIZE];			//ADC測定データ格納バッファ44
uint16_t ADCValues45[ADC_BUFFSIZE];			//ADC測定データ格納バッファ45
uint16_t ADCValues46[ADC_BUFFSIZE];			//ADC測定データ格納バッファ46
uint16_t ADCValues47[ADC_BUFFSIZE];			//ADC測定データ格納バッファ47
uint16_t ADCValues48[ADC_BUFFSIZE];			//ADC測定データ格納バッファ48
uint16_t ADCValues49[ADC_BUFFSIZE];			//ADC測定データ格納バッファ49
uint16_t ADCValues50[ADC_BUFFSIZE];			//ADC測定データ格納バッファ50
uint16_t ADCValues51[ADC_BUFFSIZE];			//ADC測定データ格納バッファ51
uint16_t ADCValues52[ADC_BUFFSIZE];			//ADC測定データ格納バッファ52
uint16_t ADCValues53[ADC_BUFFSIZE];			//ADC測定データ格納バッファ53
uint16_t ADCValues54[ADC_BUFFSIZE];			//ADC測定データ格納バッファ54
uint16_t ADCValues55[ADC_BUFFSIZE];			//ADC測定データ格納バッファ55
uint16_t ADCValues56[ADC_BUFFSIZE];			//ADC測定データ格納バッファ56
uint16_t ADCValues57[ADC_BUFFSIZE];			//ADC測定データ格納バッファ57
uint16_t ADCValues58[ADC_BUFFSIZE];			//ADC測定データ格納バッファ58
uint16_t ADCValues59[ADC_BUFFSIZE];			//ADC測定データ格納バッファ59
uint16_t ADCValues60[ADC_BUFFSIZE];			//ADC測定データ格納バッファ60
uint16_t ADCValues61[ADC_BUFFSIZE];			//ADC測定データ格納バッファ61
uint16_t ADCValues62[ADC_BUFFSIZE];			//ADC測定データ格納バッファ62
uint16_t ADCValues63[ADC_BUFFSIZE];			//ADC測定データ格納バッファ63
uint16_t ADCValues64[ADC_BUFFSIZE];			//ADC測定データ格納バッファ64
uint16_t ADCValues65[ADC_BUFFSIZE];			//ADC測定データ格納バッファ65
uint16_t ADCValues66[ADC_BUFFSIZE];			//ADC測定データ格納バッファ66
uint16_t ADCValues67[ADC_BUFFSIZE];			//ADC測定データ格納バッファ67
uint16_t ADCValues68[ADC_BUFFSIZE];			//ADC測定データ格納バッファ68
uint16_t ADCValues69[ADC_BUFFSIZE];			//ADC測定データ格納バッファ69
uint16_t ADCValues70[ADC_BUFFSIZE];			//ADC測定データ格納バッファ70
uint16_t ADCValues71[ADC_BUFFSIZE];			//ADC測定データ格納バッファ71
uint16_t ADCValues72[ADC_BUFFSIZE];			//ADC測定データ格納バッファ72
uint16_t ADCValues73[ADC_BUFFSIZE];			//ADC測定データ格納バッファ73
uint16_t ADCValues74[ADC_BUFFSIZE];			//ADC測定データ格納バッファ74
uint16_t ADCValues75[ADC_BUFFSIZE];			//ADC測定データ格納バッファ75
uint16_t ADCValues76[ADC_BUFFSIZE];			//ADC測定データ格納バッファ76
uint16_t ADCValues77[ADC_BUFFSIZE];			//ADC測定データ格納バッファ77
uint16_t ADCValues78[ADC_BUFFSIZE];			//ADC測定データ格納バッファ78
uint16_t ADCValues79[ADC_BUFFSIZE];			//ADC測定データ格納バッファ79

uint16_t ADCValues80[ADC_BUFFSIZE];			//ADC測定データ格納バッファ80
uint16_t ADCValues81[ADC_BUFFSIZE];			//ADC測定データ格納バッファ81
uint16_t ADCValues82[ADC_BUFFSIZE];			//ADC測定データ格納バッファ82
uint16_t ADCValues83[ADC_BUFFSIZE];			//ADC測定データ格納バッファ83
uint16_t ADCValues84[ADC_BUFFSIZE];			//ADC測定データ格納バッファ84
uint16_t ADCValues85[ADC_BUFFSIZE];			//ADC測定データ格納バッファ85
uint16_t ADCValues86[ADC_BUFFSIZE];			//ADC測定データ格納バッファ86
uint16_t ADCValues87[ADC_BUFFSIZE];			//ADC測定データ格納バッファ87
uint16_t ADCValues88[ADC_BUFFSIZE];			//ADC測定データ格納バッファ88
uint16_t ADCValues89[ADC_BUFFSIZE];			//ADC測定データ格納バッファ89
uint16_t ADCValues90[ADC_BUFFSIZE];			//ADC測定データ格納バッファ90
uint16_t ADCValues91[ADC_BUFFSIZE];			//ADC測定データ格納バッファ91
uint16_t ADCValues92[ADC_BUFFSIZE];			//ADC測定データ格納バッファ92
uint16_t ADCValues93[ADC_BUFFSIZE];			//ADC測定データ格納バッファ93
uint16_t ADCValues94[ADC_BUFFSIZE];			//ADC測定データ格納バッファ94
uint16_t ADCValues95[ADC_BUFFSIZE];			//ADC測定データ格納バッファ95
uint16_t ADCValues96[ADC_BUFFSIZE];			//ADC測定データ格納バッファ96
uint16_t ADCValues97[ADC_BUFFSIZE];			//ADC測定データ格納バッファ97
uint16_t ADCValues98[ADC_BUFFSIZE];			//ADC測定データ格納バッファ98
uint16_t ADCValues99[ADC_BUFFSIZE];			//ADC測定データ格納バッファ99
uint16_t ADCValuesA0[ADC_BUFFSIZE];			//ADC測定データ格納バッファA0
uint16_t ADCValuesA1[ADC_BUFFSIZE];			//ADC測定データ格納バッファA1
uint16_t ADCValuesA2[ADC_BUFFSIZE];			//ADC測定データ格納バッファA2
uint16_t ADCValuesA3[ADC_BUFFSIZE];			//ADC測定データ格納バッファA3
uint16_t ADCValuesA4[ADC_BUFFSIZE];			//ADC測定データ格納バッファA4
uint16_t ADCValuesA5[ADC_BUFFSIZE];			//ADC測定データ格納バッファA5
uint16_t ADCValuesA6[ADC_BUFFSIZE];			//ADC測定データ格納バッファA6
uint16_t ADCValuesA7[ADC_BUFFSIZE];			//ADC測定データ格納バッファA7
uint16_t ADCValuesA8[ADC_BUFFSIZE];			//ADC測定データ格納バッファA8
uint16_t ADCValuesA9[ADC_BUFFSIZE];			//ADC測定データ格納バッファA9
uint16_t ADCValuesB0[ADC_BUFFSIZE];			//ADC測定データ格納バッファB0
uint16_t ADCValuesB1[ADC_BUFFSIZE];			//ADC測定データ格納バッファB1
uint16_t ADCValuesB2[ADC_BUFFSIZE];			//ADC測定データ格納バッファB2
uint16_t ADCValuesB3[ADC_BUFFSIZE];			//ADC測定データ格納バッファB3
uint16_t ADCValuesB4[ADC_BUFFSIZE];			//ADC測定データ格納バッファB4
uint16_t ADCValuesB5[ADC_BUFFSIZE];			//ADC測定データ格納バッファB5
uint16_t ADCValuesB6[ADC_BUFFSIZE];			//ADC測定データ格納バッファB6
uint16_t ADCValuesB7[ADC_BUFFSIZE];			//ADC測定データ格納バッファB7
uint16_t ADCValuesB8[ADC_BUFFSIZE];			//ADC測定データ格納バッファB8
uint16_t ADCValuesB9[ADC_BUFFSIZE];			//ADC測定データ格納バッファB9

uint16_t ADCValuesC0[ADC_BUFFSIZE];			//ADC測定データ格納バッファC0
uint16_t ADCValuesC1[ADC_BUFFSIZE];			//ADC測定データ格納バッファC1
uint16_t ADCValuesC2[ADC_BUFFSIZE];			//ADC測定データ格納バッファC2
uint16_t ADCValuesC3[ADC_BUFFSIZE];			//ADC測定データ格納バッファC3
uint16_t ADCValuesC4[ADC_BUFFSIZE];			//ADC測定データ格納バッファC4
uint16_t ADCValuesC5[ADC_BUFFSIZE];			//ADC測定データ格納バッファC5
uint16_t ADCValuesC6[ADC_BUFFSIZE];			//ADC測定データ格納バッファC6
uint16_t ADCValuesC7[ADC_BUFFSIZE];			//ADC測定データ格納バッファC7
uint16_t ADCValuesC8[ADC_BUFFSIZE];			//ADC測定データ格納バッファC8
uint16_t ADCValuesC9[ADC_BUFFSIZE];			//ADC測定データ格納バッファC9
uint16_t ADCValuesD0[ADC_BUFFSIZE];			//ADC測定データ格納バッファD0
uint16_t ADCValuesD1[ADC_BUFFSIZE];			//ADC測定データ格納バッファD1
uint16_t ADCValuesD2[ADC_BUFFSIZE];			//ADC測定データ格納バッファD2
uint16_t ADCValuesD3[ADC_BUFFSIZE];			//ADC測定データ格納バッファD3
uint16_t ADCValuesD4[ADC_BUFFSIZE];			//ADC測定データ格納バッファD4
uint16_t ADCValuesD5[ADC_BUFFSIZE];			//ADC測定データ格納バッファD5
uint16_t ADCValuesD6[ADC_BUFFSIZE];			//ADC測定データ格納バッファD6
uint16_t ADCValuesD7[ADC_BUFFSIZE];			//ADC測定データ格納バッファD7
uint16_t ADCValuesD8[ADC_BUFFSIZE];			//ADC測定データ格納バッファD8
uint16_t ADCValuesD9[ADC_BUFFSIZE];			//ADC測定データ格納バッファD9
uint16_t ADCValuesE0[ADC_BUFFSIZE];			//ADC測定データ格納バッファE0
uint16_t ADCValuesE1[ADC_BUFFSIZE];			//ADC測定データ格納バッファE1
uint16_t ADCValuesE2[ADC_BUFFSIZE];			//ADC測定データ格納バッファE2
uint16_t ADCValuesE3[ADC_BUFFSIZE];			//ADC測定データ格納バッファE3
uint16_t ADCValuesE4[ADC_BUFFSIZE];			//ADC測定データ格納バッファE4
uint16_t ADCValuesE5[ADC_BUFFSIZE];			//ADC測定データ格納バッファE5
uint16_t ADCValuesE6[ADC_BUFFSIZE];			//ADC測定データ格納バッファE6
uint16_t ADCValuesE7[ADC_BUFFSIZE];			//ADC測定データ格納バッファE7
uint16_t ADCValuesE8[ADC_BUFFSIZE];			//ADC測定データ格納バッファE8
uint16_t ADCValuesE9[ADC_BUFFSIZE];			//ADC測定データ格納バッファE9
uint16_t ADCValuesF0[ADC_BUFFSIZE];			//ADC測定データ格納バッファF0
uint16_t ADCValuesF1[ADC_BUFFSIZE];			//ADC測定データ格納バッファF1
uint16_t ADCValuesF2[ADC_BUFFSIZE];			//ADC測定データ格納バッファF2
uint16_t ADCValuesF3[ADC_BUFFSIZE];			//ADC測定データ格納バッファF3
uint16_t ADCValuesF4[ADC_BUFFSIZE];			//ADC測定データ格納バッファF4
uint16_t ADCValuesF5[ADC_BUFFSIZE];			//ADC測定データ格納バッファF5
uint16_t ADCValuesF6[ADC_BUFFSIZE];			//ADC測定データ格納バッファF6
uint16_t ADCValuesF7[ADC_BUFFSIZE];			//ADC測定データ格納バッファF7
uint16_t ADCValuesF8[ADC_BUFFSIZE];			//ADC測定データ格納バッファF8
uint16_t ADCValuesF9[ADC_BUFFSIZE];			//ADC測定データ格納バッファF9

// ADC Status flag
__IO ITStatus ADCBusy = RESET;				//ADC Busy Flag
// Uart status Flag
__IO ITStatus UartTXBusy = RESET;			//UART Busy Flag
// Uart buffer
char RxBuff[USART_RX_BUFFSIZE];				//UART 受信バッファ
char TxBuff[USART_TX_BUFFSIZE];				//UART 送信バッファ
uint8_t RxData = 0;							//UART 受信データ
bool resDataFlg= false;						//UART CR/LFまで受信したかのフラグ
uint16_t RxBuffCounter = 0;					//UART受信データ数
// PWM Status flag
__IO ITStatus PWMBusy1 = RESET;				//PWM Busy flag for TIM1
__IO ITStatus PWMBusy3 = RESET;				//PWM Busy flag for TIM3

/**********************************************************************************************************************
 * パラメータ
 **********************************************************************************************************************/
/*
 * パルス設定
 * DMAバースト転送で送信するデータ、PSC, ARR, RCP, CCRの順で送ること
 * 最後の2つのPSC, ARR, RCP, CCRはダブルパルス出力後LOWに保つためのものなので変更しないこと
 * aSRC_Buffer[20] = {79, T0+T1-1, 0, T1, 79, T2+T3-1, 0, T3, 0, 0xFFF, 0, 0x1000, 0, 0xFFF, 0, 0x1000, 0, 0xFFF, 0, 0x1000};
 *                   {PRC, ARR, RCP, CCR}
 * 1div 12.5ns, 12.5ns x (79 + 1) = 1us
 */

//n double pulse test
//High-side(upper)
//加熱(Heating)
//800us周期/9us幅×1251回＝1000.8ms、0.2ms(L)+0.85ms(L) for 加熱後2ms測定開始
uint32_t aSRC_Buffer_uH[16] = {79, 799, 1251, 791, 79, 1049, 0, 1050, 0, 0xFFF, 0, 0x1000, 0, 0xFFF, 0, 0x1000};
//測定
//1.2ms(H)+1.1ms(L) for 2.5ms周期
uint32_t aSRC_Buffer_u[16] = {79, 1199, 0, 0, 79, 1099, 0, 1100, 0, 0xFFF, 0, 0x1000, 0, 0xFFF, 0, 0x1000};
//減衰(Damping)
//0.9s(H)+0.1s(L)
uint32_t aSRC_Buffer_uD[16] = {7999, 8999, 0, 0, 7999, 999, 0, 1000, 0, 0xFFF, 0, 0x1000, 0, 0xFFF, 0, 0x1000};

//Low-side
//加熱(Heating)
//100us×10010＝1000.8ms+0.2ms(H)、0.85ms(L) for 加熱後2ms測定開始
uint32_t aSRC_Buffer_H[16] = {7999, 10009, 0, 0, 79, 849, 0, 850, 0, 0xFFF, 0, 0x1000, 0, 0xFFF, 0, 0x1000};
//測定
//1ms周期/10us幅＋20us周期/10us幅＋1.318ms周期(L) for 2.5ms周期
uint32_t aSRC_Buffer[20] = {79, 999, 0, 990, 79, 19, 0, 10, 79, 1317, 0, 1318, 0, 0xFFF, 0, 0x1000, 0, 0xFFF, 0, 0x1000};
//減衰(Damping)
//1s(L)
uint32_t aSRC_Buffer_D[12] = {7999, 9999, 0, 10000, 0, 0xFFF, 0, 0x1000, 0, 0xFFF, 0, 0x1000};

//settings
//初期値
uint8_t u8ADC_ChNo = 1;						/* 測定チャンネル 1 or 2 */
uint8_t u8ADC_Resolution = 12;				/* ADC 解像度 6/8/10/12 */
uint16_t u16ADC_SMPLTime = 2;              	/* ADC Sampling time */
uint32_t u32ADC_StartDelay = 719;			/* ADC開始遅延時間 1div 12.5ns x(719+1)=9us */

/**********************************************************************************************************************
 * パラメータ
 **********************************************************************************************************************/

// for debug
__IO uint32_t ConvCpltCounter = 0;			// Debug用 ADC変換完了カウンタ
__IO uint32_t ADCErrorCounter = 0;			// Debug用 ADC変換エラーカウンタ
//
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void chkcommand2(void);
ErrorStatus TestStart_Heating(void);
ErrorStatus TestStart_Damping(void);
ErrorStatus TestStart(uint32_t, uint16_t *);
void setADC_config2(uint8_t ,uint8_t, uint16_t);
void TIM2_Init(uint32_t);

ErrorStatus  uartSendData(char *) ;
ErrorStatus  uartSendADCData(uint16_t *);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //  txBuffer4461 = REG_DATA;
  HAL_I2C_Mem_Write(&hi2c1, MCP4461_ADDR, REG_NVWiper0, 1, &txBuffer4461, 1, 1000);
  HAL_Delay(500);
  HAL_I2C_Mem_Write(&hi2c1, TCA9536A_ADDR, REG_Configuration, 1, &txBuffer9536AoEB, 1, 1000);
  HAL_Delay(500);
  HAL_I2C_Mem_Write(&hi2c1, TCA9536A_ADDR, REG_OutputPort, 1, &txBuffer9536A, 1, 1000);
  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //uart Receive Start
  //UART 受信バッファクリア
  __HAL_UART_FLUSH_DRREGISTER(&huart2);
  //UART 受信開始 (1バイト受信したら割り込み発生)
  if (HAL_UART_Receive_IT(&huart2, &RxData, 1) !=  HAL_OK)
  {
	  Error_Handler();
  }

  while (1)
  {
	  //受信data check
	  if (resDataFlg == true) {
		  //コマンド判定&実行
		  chkcommand2();
		  //受信フラグクリア
		  resDataFlg = false;
		  RxBuffCounter = 0;
		  //受信再開
		  if (HAL_UART_Receive_IT(&huart2, &RxData, 1) !=  HAL_OK)
		  {
			  Error_Handler();
		  }
	  }
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  //ADCのキャリブレーション
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x10909CEC;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 79;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 80;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB5 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/****************************************************************************************************************************************************************
 *
 *  新規追加関数
 *
 *
 ****************************************************************************************************************************************************************/
/**
  * @brief 受信コマンド解析
  * @param None
  * @retval None
  */
void chkcommand2(void)
{
	char * para;
	char paraData[30];
	uint32_t data = 0;
	char strouttemp[80];

	if ((RxBuff[0] == 'S' || RxBuff[0] == 's') && (RxBuff[1] == '\r' ) &&  (RxBuff[2] == '\n')  ) {
		TestStart_Heating();						//加熱

		//double pulse test
		TestStart(u32ADC_StartDelay, ADCValues00);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues01);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues02);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues03);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues04);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues05);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues06);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues07);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues08);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues09);	//ADC

		TestStart(u32ADC_StartDelay, ADCValues10);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues11);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues12);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues13);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues14);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues15);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues16);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues17);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues18);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues19);	//ADC

		TestStart(u32ADC_StartDelay, ADCValues20);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues21);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues22);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues23);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues24);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues25);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues26);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues27);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues28);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues29);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues30);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues31);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues32);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues33);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues34);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues35);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues36);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues37);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues38);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues39);	//ADC

		TestStart(u32ADC_StartDelay, ADCValues40);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues41);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues42);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues43);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues44);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues45);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues46);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues47);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues48);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues49);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues50);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues51);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues52);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues53);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues54);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues55);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues56);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues57);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues58);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues59);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues60);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues61);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues62);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues63);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues64);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues65);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues66);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues67);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues68);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues69);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues70);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues71);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues72);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues73);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues74);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues75);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues76);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues77);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues78);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues79);	//ADC

		TestStart(u32ADC_StartDelay, ADCValues80);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues81);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues82);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues83);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues84);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues85);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues86);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues87);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues88);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues89);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues90);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues91);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues92);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues93);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues94);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues95);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues96);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues97);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues98);	//ADC
		TestStart(u32ADC_StartDelay, ADCValues99);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesA0);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesA1);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesA2);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesA3);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesA4);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesA5);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesA6);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesA7);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesA8);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesA9);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesB0);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesB1);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesB2);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesB3);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesB4);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesB5);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesB6);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesB7);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesB8);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesB9);	//ADC

		TestStart(u32ADC_StartDelay, ADCValuesC0);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesC1);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesC2);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesC3);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesC4);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesC5);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesC6);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesC7);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesC8);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesC9);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesD0);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesD1);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesD2);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesD3);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesD4);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesD5);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesD6);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesD7);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesD8);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesD9);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesE0);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesE1);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesE2);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesE3);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesE4);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesE5);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesE6);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesE7);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesE8);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesE9);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesF0);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesF1);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesF2);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesF3);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesF4);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesF5);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesF6);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesF7);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesF8);	//ADC
		TestStart(u32ADC_StartDelay, ADCValuesF9);	//ADC

		TestStart_Damping();						//減衰

		// ADC data send UART
		uartSendADCData(ADCValues00);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues01);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues02);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues03);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues04);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues05);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues06);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues07);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues08);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues09);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms

		uartSendADCData(ADCValues10);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues11);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues12);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues13);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues14);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues15);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues16);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues17);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues18);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues19);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms

		uartSendADCData(ADCValues20);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues21);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues22);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues23);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues24);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues25);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues26);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues27);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues28);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues29);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues30);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues31);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues32);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues33);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues34);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues35);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues36);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues37);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues38);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues39);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms

		uartSendADCData(ADCValues40);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues41);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues42);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues43);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues44);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues45);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues46);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues47);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues48);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues49);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues50);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues51);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues52);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues53);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues54);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues55);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues56);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues57);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues58);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues59);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues60);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues61);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues62);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues63);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues64);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues65);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues66);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues67);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues68);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues69);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues70);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues71);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues72);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues73);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues74);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues75);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues76);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues77);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues78);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues79);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms

		uartSendADCData(ADCValues80);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues81);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues82);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues83);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues84);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues85);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues86);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues87);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues88);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues89);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues90);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues91);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues92);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues93);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues94);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues95);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues96);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues97);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues98);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValues99);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesA0);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesA1);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesA2);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesA3);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesA4);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesA5);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesA6);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesA7);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesA8);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesA9);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesB0);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesB1);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesB2);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesB3);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesB4);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesB5);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesB6);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesB7);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesB8);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesB9);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms

		uartSendADCData(ADCValuesC0);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesC1);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesC2);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesC3);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesC4);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesC5);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesC6);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesC7);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesC8);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesC9);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesD0);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesD1);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesD2);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesD3);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesD4);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesD5);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesD6);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesD7);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesD8);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesD9);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesE0);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesE1);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesE2);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesE3);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesE4);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesE5);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesE6);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesE7);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesE8);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesE9);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesF0);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesF1);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesF2);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesF3);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesF4);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesF5);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesF6);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesF7);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesF8);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms
		uartSendADCData(ADCValuesF9);	//UART
		HAL_Delay(100); 				// 転送完了待ち100ms

	}
	else if  ((RxBuff[0] == 'd' ) &&  (RxBuff[1] == 's') &&  (RxBuff[2] == 'p')&& (RxBuff[3] == '\r' ) &&  (RxBuff[4] == '\n')) {
		//設定表示
		/*
		 * 表示する文字列をstrouttempに格納し、TxBuffに結合して最終的に出力する文字列を作成する
		 */

		//TxBuff クリア
		memset((char*)TxBuff,0 ,USART_TX_BUFFSIZE);
		//送信文字列作成
		sprintf(TxBuff,"Settings......\r\n");
		sprintf(strouttemp,"ADC Channel                = %d\r\n",u8ADC_ChNo);
		strcat(TxBuff,strouttemp);
		sprintf(strouttemp,"ADC Resolution             = %d\r\n",u8ADC_Resolution);
		strcat(TxBuff,strouttemp);
		sprintf(strouttemp,"ADC Sampling Time          = %d\r\n",u16ADC_SMPLTime);
		strcat(TxBuff,strouttemp);
		sprintf(strouttemp,"ADC Start delay Time       = %ld\r\n",u32ADC_StartDelay);
		strcat(TxBuff,strouttemp);
		sprintf(strouttemp,"Pulse 1(ARR , CCR)         = %d , %d\r\n",(uint16_t)aSRC_Buffer[1],(uint16_t)aSRC_Buffer[3]);
		strcat(TxBuff,strouttemp);
		sprintf(strouttemp,"Pulse 2(ARR , CCR)         = %d , %d\r\n",(uint16_t)aSRC_Buffer[5],(uint16_t)aSRC_Buffer[7]);
		strcat(TxBuff,strouttemp);
		strcat(TxBuff,CMD_OK);
		// UART送信
		uartSendData(TxBuff);
	}
	else {

		/*コマンドcheck
		 *
		 * " "まで文字を取り出しdefine定義したコマンド文字列と一致しているか判定する
		 * 一致した場合、" "以降文字を取り出し数値変換してパラメータにセットする
		 *
		 */

		//受信データを内部バッファにコピー
		memcpy(paraData, RxBuff, strlen(RxBuff) );
		// スペースまでの位置を取得
		para = strtok(paraData, " ");
		//取り出した文字列がコマンドと一致してるか判定
		if (strcmp(para , CMD_SET_CH_No) == 0){
			// 引数を取り出し
			para = strtok(NULL, " ");
			data = strtoul(para, NULL, 10);
			//引数check
			if (data == 1 ||  data  == 2 ) {
				u8ADC_ChNo = data;
				uartSendData(CMD_OK);				// OK送信
			}
			else{
				uartSendData(CMD_ERR);				// ERR送信
			}
		}
		else if (strcmp(para , CMD_SET_ADC_RESO) == 0){
			para = strtok(NULL, " ");
			data = strtoul(para, NULL, 10);
			if (( data == 6 ) || ( data == 8 ) || (data ==10 ) || ( data == 12 )){
				u8ADC_Resolution = data;
				uartSendData(CMD_OK);
			}
			else{
				uartSendData(CMD_ERR);
			}
		}
		else if (strcmp(para , CMD_SET_ADC_SMPLTIME) == 0){
			para = strtok(NULL, " ");
			data = strtoul(para, NULL, 10);
			if (( data == 2 ) || ( data == 6 ) || (data ==12 ) || ( data == 24 ) || ( data == 47 ) || ( data == 47 )|| ( data == 92 )|| ( data == 247 )|| ( data == 640 )){
				u16ADC_SMPLTime = data;
				uartSendData(CMD_OK);
			}
			else {
				uartSendData(CMD_ERR);
			}
		}
		else if (strcmp(para , CMD_SET_ADC_DELAY) == 0){
			para = strtok(NULL, " ");
			data = strtoul(para, NULL, 10);
			if (( data >= 39) && (data <= 0xFFFFFFFF)){
				u32ADC_StartDelay = data;
				uartSendData(CMD_OK);
			}
			else {
				uartSendData(CMD_ERR);
			}
		}
		else if (strcmp(para , CMD_SET_PWM_ARR1) == 0){
			para = strtok(NULL, " ");
			data = strtoul(para, NULL, 10);
			if (( data >= 1) && (data <= 0xFFFF)){
				aSRC_Buffer[1] = data;
				uartSendData(CMD_OK);
			}
			else {
				uartSendData(CMD_ERR);
			}
		}
		else if (strcmp(para , CMD_SET_PWM_ARR2) == 0){
			para = strtok(NULL, " ");
			data = strtoul(para, NULL, 10);
			if (( data >= 1) && (data <= 0xFFFF)){
				aSRC_Buffer[5] = data;
				uartSendData(CMD_OK);
			}
			else {
				uartSendData(CMD_ERR);
			}
		}
		else if (strcmp(para , CMD_SET_PWM_CCR1) == 0){
			para = strtok(NULL, " ");
			data = strtoul(para, NULL, 10);
			if (( data >= 0) && (data <= 0xFFFF)){
				aSRC_Buffer[3] = data;
				uartSendData(CMD_OK);
			}
			else {
				uartSendData(CMD_ERR);
			}
		}
		else if (strcmp(para , CMD_SET_PWM_CCR2) == 0){
			para = strtok(NULL, " ");
			data = strtoul(para, NULL, 10);
			if (( data >= 0) && (data <= 0xFFFF)){
				aSRC_Buffer[7] = data;
				uartSendData(CMD_OK);
			}
			else {
				uartSendData(CMD_ERR);
			}
		}
		else
		{
			//コマンド以外の文字ならコマンドリストを表示
			memset((char*)TxBuff,0 ,USART_TX_BUFFSIZE);
			sprintf(TxBuff,"Command......\r\n");
			sprintf(strouttemp,"Test start(Single).................... s or S\r\n");
			strcat(TxBuff,strouttemp);
			sprintf(strouttemp,"Display Test settings ................ dsp\r\n\r\n");
			strcat(TxBuff,strouttemp);
			sprintf(strouttemp,"Set ADC Channel ...................... ch X (1/2)\r\n");
			strcat(TxBuff,strouttemp);
			sprintf(strouttemp,"Set ADC Resolution (bit) .............. ar X (6/8/10/12)\r\n");
			strcat(TxBuff,strouttemp);
			sprintf(strouttemp,"Set ADC Sampling Time (cycle) ......... as X (2/6/12/24/47/92/247/640)\r\n");
			strcat(TxBuff,strouttemp);
			sprintf(strouttemp,"Set ADC Start delay Time (cycle) ...... ad X (39-65535)\r\n");
			strcat(TxBuff,strouttemp);
			sprintf(strouttemp,"Set Pulse 1 ARR (us) .................. pa1 X (1-65535)\r\n");
			strcat(TxBuff,strouttemp);
			sprintf(strouttemp,"Set Pulse 2 ARR (us) .................. pa2 X (1-65535)\r\n");
			strcat(TxBuff,strouttemp);
			sprintf(strouttemp,"Set Pulse 1 CCR (us) .................. pc1 X (0-65535)\r\n");
			strcat(TxBuff,strouttemp);
			sprintf(strouttemp,"Set Pulse 2 CCR (us) .................. pc2 X (0-65535)\r\n");
			strcat(TxBuff,strouttemp);
			uartSendData(TxBuff);
		}
	}
}
/**
  * @brief Heating for thermal time constant test
  * @param None
  * @retval None
  */
ErrorStatus TestStart_Heating(void)
{
	ErrorStatus sts = SUCCESS;
	//前回のタイマ設定をクリアするためタイマを再設定する
	MX_TIM1_Init();
	MX_TIM3_Init();
	   // PWM start
	   //TIM1 for upper IGBT
		  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
		  {
			   /* PWM Generation Error */
			   Error_Handler();
		  }
		  //PWM DMA Burst transfer start
		  PWMBusy1 = SET;
		  HAL_TIM_DMABurst_MultiWriteStart(&htim1, TIM_DMABASE_PSC, TIM_DMA_UPDATE, (uint32_t*) aSRC_Buffer_uH, TIM_DMABURSTLENGTH_4TRANSFERS,16);

		  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
		  {
			   /* PWM Generation Error */
			   Error_Handler();
		  }
		  //PWM DMA Burst transfer start
		  PWMBusy3 = SET;
		  HAL_TIM_DMABurst_MultiWriteStart(&htim3, TIM_DMABASE_PSC, TIM_DMA_UPDATE, (uint32_t*) aSRC_Buffer_H, TIM_DMABURSTLENGTH_4TRANSFERS,16);
	 //ADCの変換とPWMの出力が終了するまで待つ
	 while(ADCBusy ==SET || PWMBusy1 == SET || PWMBusy3 == SET){

	 }
	 return sts;
}

/**
  * @brief Damping for test
  * @param None
  * @retval None
  */
ErrorStatus TestStart_Damping(void)
{
	ErrorStatus sts = SUCCESS;
	//前回のタイマ設定をクリアするためタイマを再設定する
	MX_TIM1_Init();
	MX_TIM3_Init();
	   // PWM start
	   //TIM1 for upper IGBT
		  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
		  {
			   /* PWM Generation Error */
			   Error_Handler();
		  }
		  //PWM DMA Burst transfer start
		  PWMBusy1 = SET;
		  HAL_TIM_DMABurst_MultiWriteStart(&htim1, TIM_DMABASE_PSC, TIM_DMA_UPDATE, (uint32_t*) aSRC_Buffer_uD, TIM_DMABURSTLENGTH_4TRANSFERS,16);

		  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
		  {
			   /* PWM Generation Error */
			   Error_Handler();
		  }
		  //PWM DMA Burst transfer start
		  PWMBusy3 = SET;
		  HAL_TIM_DMABurst_MultiWriteStart(&htim3, TIM_DMABASE_PSC, TIM_DMA_UPDATE, (uint32_t*) aSRC_Buffer_D, TIM_DMABURSTLENGTH_4TRANSFERS,12);
	 //ADCの変換とPWMの出力が終了するまで待つ
	 while(ADCBusy ==SET || PWMBusy1 == SET || PWMBusy3 == SET){

	 }
	 return sts;
}

/**
  * @brief double pulse test
  * @param uint32_t dtime ADC開始遅延時間
  * @retval uint16_t *ADCValues A/D変換値
  */
ErrorStatus TestStart(uint32_t dtime, uint16_t *ADCValues)
{
	ErrorStatus sts = SUCCESS;
	//前回のタイマ設定をクリアするためタイマを再設定する
	MX_TIM1_Init();
	MX_TIM3_Init();
	TIM2_Init((uint32_t)dtime);
	//ADCの測定設定を行う
	setADC_config2(u8ADC_ChNo,u8ADC_Resolution,u16ADC_SMPLTime);

	//ADC Start
	  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCValues, ADC_BUFFSIZE) != HAL_OK)
	  {
		  Error_Handler();
	  }
	   ADCBusy = SET;

	   // PWM start
	   //TIM1 for upper IGBT
		  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
		  {
			   /* PWM Generation Error */
			   Error_Handler();
		  }
		  //PWM DMA Burst transfer start
		  PWMBusy1 = SET;
		  HAL_TIM_DMABurst_MultiWriteStart(&htim1, TIM_DMABASE_PSC, TIM_DMA_UPDATE, (uint32_t*) aSRC_Buffer_u, TIM_DMABURSTLENGTH_4TRANSFERS,16);

		//TIM3 for lower IGBT
		  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
		  {
			   /* PWM Generation Error */
			   Error_Handler();
		  }
		  //PWM DMA Burst transfer start
		  PWMBusy3 = SET;
		  HAL_TIM_DMABurst_MultiWriteStart(&htim3, TIM_DMABASE_PSC, TIM_DMA_UPDATE, (uint32_t*) aSRC_Buffer, TIM_DMABURSTLENGTH_4TRANSFERS,20);
	 //TIM2 start
	 __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);					//timer reset
	 HAL_TIM_Base_Start_IT(&htim2);									// timer start
	 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);							// for debug

	 //ADCの変換とPWMの出力が終了するまで待つ
	 while(ADCBusy ==SET || PWMBusy1 == SET || PWMBusy3 == SET){

	 }
	 //Stop ADC
	 HAL_ADC_Stop_DMA(&hadc1);
	 // ADC data send UART
//	 uartSendADCData();

	 return sts;
}
/**
  * @brief   ADC初期化
  * @param	ch:使用するADC Channel
  * 		bitW:ADCの解像度
  * 		stime:ADCのサンプリング時間
  * @retval
* */
void setADC_config2(uint8_t ch ,uint8_t bitW, uint16_t stime){

	  ADC_ChannelConfTypeDef sConfig = {0};

	if ( bitW == 6) {
		hadc1.Init.Resolution = ADC_RESOLUTION_6B;
	}
	else if (bitW == 8)	{
		 hadc1.Init.Resolution = ADC_RESOLUTION_8B;
	}
	else if (bitW == 10)	{
		 hadc1.Init.Resolution = ADC_RESOLUTION_10B;
	}
	else if (bitW == 12)	{
		hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	}
	else {
		hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	}

	 if (HAL_ADC_Init(&hadc1) != HAL_OK)
	 {
		 Error_Handler();
	 }

	if ( ch == 1) {
	  sConfig.Channel = ADC_CHANNEL_1;
	}
	else if (ch == 2)	{
		 sConfig.Channel = ADC_CHANNEL_2;
	}
	else {
		 sConfig.Channel = ADC_CHANNEL_1;
	}
	if (stime ==2){
		sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	} else if (stime ==6){
		sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	} else if (stime ==12){
		sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
	} else if (stime == 24){
		sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
	} else if (stime == 47){
		sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	} else if (stime == 92){
		sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
	} else if (stime == 247){
		sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	} else if (stime == 640){
		sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	} else {
		sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	}
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;

	 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	 {
		 Error_Handler();
	 }

}
/**
  * @brief タイマ設定初期化
  * @param dtime: 設定するARRレジスタ値(オーバーフローするカウンタ値)
  * @retval None
  */
void TIM2_Init(uint32_t dtime)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period =  dtime;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;

	  sSlaveConfig.InputTrigger = TIM_TS_ITR2;	//TIM3

	  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  UART送信 (databufのdataを送信する)
  * @param databuf 送信data
   * @retval
* */
ErrorStatus  uartSendData(char *databuf) {
	uint8_t sts = SUCCESS;
	uint16_t cnt =0;

	while  (huart2.gState != HAL_UART_STATE_READY)
	{
		HAL_Delay(1); //1ms wait
		cnt ++;
		if (cnt > 1000){
			break;
		}
	}
	 if (huart2.gState == HAL_UART_STATE_READY) {
		  //not busy
		  UartTXBusy = SET; //busy set
		  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)databuf, strlen(( char * )databuf))!= HAL_OK)
		  {
			  sts = ERROR;
			  Error_Handler(); // 無限loop
		 }
	  }
	  else {
		  sts = ERROR;
	  }
	return sts;
}
/**
  * @brief  ADC DATA UART送信
  * @param none
   * @retval error status
* */
ErrorStatus  uartSendADCData(uint16_t *ADCValues) {
	ErrorStatus sts = SUCCESS;
	uint16_t cnt =0;
	char sendbuf[20];

	memset((char*)TxBuff,0 ,USART_TX_BUFFSIZE);
	sprintf(TxBuff,"ADC DATA\r\n");

	for ( int i = 0; i < ADC_BUFFSIZE ; i++) {
		sprintf(sendbuf,"%d,",ADCValues[ i ]);
		strcat(TxBuff,sendbuf);
	}
	sprintf((char*)sendbuf,"\r\nFinish!!\r\n");
	strcat((char*)TxBuff,sendbuf);

	//UARTが送信出来るようになるまで待つ
	while  (huart2.gState != HAL_UART_STATE_READY)
	{
		HAL_Delay(1); //1ms wait
		cnt ++;
		if (cnt > 1000){
			break;
		}
	}

	//送信可能なら送信する
	if (huart2.gState == HAL_UART_STATE_READY){
		  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)TxBuff, strlen(( char * )TxBuff))!= HAL_OK)
		  {
			  sts = ERROR;
			  Error_Handler(); // 無限loop
		 }
	}
	return sts;
}
/**
  * @brief  Conversion complete callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	 ConvCpltCounter++;						//debug
	 ADCBusy = RESET;						//Reset ADC Busy
	 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);	//debug
}
/**
  * @brief  ADC erorr  callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	ADCErrorCounter++;						//debug
}
/**
  * @brief  UART TX  complete  callback
  * UARTの送信が完了したときに呼ばれる
  * @param UartHandle Uart handle
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/
 huart2.gState = HAL_UART_STATE_READY;			//UARTのスタータスをREADYにする
 UartTXBusy = RESET;

}
/**
  * @brief  UART RX  complete  callback
  * UARTで受信が完了したときに呼ばれる
  * @param UartHandle Uart handle
  * @retval None
**/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{

    RxBuff[RxBuffCounter] =RxData;
    //check CR+LF
    if ((RxBuffCounter >= 1) && (RxBuff[ RxBuffCounter - 1 ] == 0x0d) && (RxBuff[ RxBuffCounter  ] == 0x0a)){
    	// CR LFまで受信したら受信完了フラグを立てる
    	resDataFlg = true;
    }
    RxBuffCounter++;
   // CR LFまで受信出来ていない場合は、再度UART受信を開始する。CR/LFまで受信出来た場合は、コマンド処理後に受信を開始する
   if (resDataFlg != true) {
	   HAL_UART_Receive_IT(&huart2, &RxData, 1);
   }

}
/**
  * @brief  UART ERROR  callback
  * UARTでエラーが起きたときに呼ばれる
  * @param UartHandle Uart handle
  * @retval None
**/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}

/**
  * @brief timer callback
  * タイマがオーバーフローしたときに呼ばれる
  * @param htim timer handle
  * @retval None
**/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == htim1.Instance) {
		//Timer1が終了したときPWM1とTIM1へのDMA転送を停止する
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_DMABurst_WriteStop(&htim1,  TIM_DMA_UPDATE);
		PWMBusy1 = RESET;
	}
	else if (htim->Instance == htim3.Instance){
		//Timer3が終了したときPWM3とTIM3へのDMA転送を停止する
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_TIM_DMABurst_WriteStop(&htim3,  TIM_DMA_UPDATE);
		PWMBusy3 = RESET;
	}
	else if (htim->Instance == htim2.Instance){
		//Timer2が終了したときTimer2を止める
		HAL_TIM_Base_Stop_IT (&htim2);
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	}
}
/**
  * @brief  PWM Pulse finished callback in non-blocking mode.
  * @param  htim timer handle
  * @retval None
**/
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
}

/****************************************************************************************************************************************************************/
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
