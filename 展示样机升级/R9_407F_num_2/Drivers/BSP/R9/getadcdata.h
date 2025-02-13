/*
 * @Author: lisir lisir@rehand.com
 * @Date: 2024-06-07 16:01:18
 * @LastEditors: lisir lisir@rehand.com
 * @LastEditTime: 2024-06-14 10:45:00
 * @FilePath: \MDK-ARMc:\Users\fu\Desktop\Code\CodeV1.1\R9_407_V1.1\R9_407_V1.1\Drivers\BSP\R9\getadcdata.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef __GETADCDATA_H
#define __GETADCDATA_H

#include "./SYSTEM/sys/sys.h"

/************************************** ҡ�����ݳ��� �� ADC ���ݽṹ�嶨�� ****************************************************/
#define yadc_max      1700
#define yadc_min      -1700
#define xadc_max      1700
#define xadc_min      -1700

#define yadc_Dim      200
#define xadc_Dim      200

/*define class of joystic*/
#define SMC35B2G 

#if defined SMC35B2G
#define X_BASEreturn	0

#elif defined SMC25B2G
#define X_BASEreturn	1

#endif  



typedef struct 
{

    uint16_t A1_ADC;
    uint16_t A2_ADC;
    uint16_t B1_ADC;
    uint16_t B2_ADC;
    uint16_t  ASH1_DIFF_AD;
    uint16_t  ASH2_DIFF_AD;
    uint16_t  BSH2_DIFF_AD;
    uint16_t  BSH1_DIFF_AD;
    uint16_t  JOYTICK_Med;
    uint16_t  JOYTICK_X;
    uint16_t  JOYTICK_Y;
    uint16_t BRAKE1_DET;
    uint16_t BRAKE2_DET;

    int32_t adc_x;
    int32_t adc_y;
    int32_t adc_xbase;
    int32_t adc_ybase;
    int32_t l_current;
    double l_currentAct;
    double r_currentAct;
    int32_t r_current;

    uint16_t lift_pos;
    uint16_t pedestal_pos;
    uint16_t backboard_pos;
    uint16_t legangle_pos;
    uint16_t leglength_pos;
    uint16_t support_pos;
    uint16_t bat_v;
		uint16_t chargeI_adc;
    double chargeI_adcAct;
		
} ADCDATA;
extern ADCDATA adcdata;
void Datareset(void);
void getadcDataInit(void);
void getadcData(void);
void getadc1Data(void);
void getadc3Data(void);
void joysticData_calculateInit(void);
void joysticData_calculate(void);
double roundToNDecimalPlaces(double num, int n) ;
#endif
