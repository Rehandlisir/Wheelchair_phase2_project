/**
 ****************************************************************************************************
 * @file        beep.h
 * @author      Lisir
 * @version     V1.0
 * @date        2021-10-14
 * @brief       �������� ��������
 * @license     Copyright (c) 2024, ���ڸ���ҽ�ƿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:F407������
 * none
 * none
 * none
 * none
 *
 * �޸�˵��
 * none
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#ifndef __COMMON_H
#define __COMMON_H

#include "./SYSTEM/sys/sys.h"
#include "math.h"

/* cubic poly interpolation struct */
typedef struct _mcl_cubic_poly_intpol_st {
	unsigned char intpol_completed;                 //Interpolation complete flag, 1: undone, 0: done
	float intpol_ts;                                //Interpolation interval, equal to control cycle
	float intpol_time;                              //Interpolation time
	float total_intpol_time;                        //Total interpolation time
	float start;                                    //Start value
	float end;                                      //End value
	float coefficients[4];                          //Interpolation coefficient
	float next_value;                               //Next interpolation value
}mcl_cubic_poly_intpol_st;


/* line poly interpolation struct */
typedef struct _mcl_line_poly_intpol_st {
	unsigned char intpol_completed;                 //Interpolation complete flag, 1: undone, 0: done
	float intpol_ts;                                //Interpolation interval, equal to control cycle
	float intpol_time;                              //Interpolation time
	float total_intpol_time;                        //Total interpolation time
	float start;                                    //Start value
	float end;                                      //End value
	float next_value;                               //Next interpolation value
}mcl_line_poly_intpol_st;

/* parabola poly interpolation struct */
typedef struct _mcl_parabola_poly_intpol_st {
	unsigned char intpol_completed;                 //Interpolation complete flag, 1: undone, 0: done
	float intpol_ts;                                //Interpolation interval, equal to control cycle
	float intpol_time;                              //Interpolation time
	float intpol_time2;                              //Interpolation time
	float total_time1;                              //Acceleration time
	float total_time2;                              //deceleration time
	float peak_time;
	float peak;                                     //Peak value
	float acc_coefficients[4];                      //Acceleration coefficient
	float dec_coefficients[4];                      //Deceleration coefficient
	float next_value;                               //Next interpolation value
}mcl_parabola_poly_intpol_st;

#define WINDOW_SIZE 10
 

typedef struct {
    float window[WINDOW_SIZE];
	int16_t window_int[WINDOW_SIZE];
    uint8_t index;
} AverageFilter;

extern AverageFilter filter_LN;
extern AverageFilter filter_RN;
extern AverageFilter filter_Lpwm;
extern AverageFilter filter_Rpwm;
extern AverageFilter filter_lspeed;
extern AverageFilter filter_rspeed;

int32_t Value_limit(int32_t min_value ,int32_t current_value ,int32_t max_value);
int32_t Value_ResetXzero(int32_t min_value, int32_t current_value, int32_t max_value);
int32_t Value_ResetYzero(int32_t min_value, int32_t current_value, int32_t max_value);
int32_t local_slopelimitx(int32_t value, int32_t increvalue,int32_t decreasvalue);
int32_t local_slopelimity(int32_t value, int32_t increvalue,int32_t decreasvalue);
int32_t remote_slopelimitx(int32_t value, int32_t increvalue,int32_t decreasvalue);
int32_t remote_slopelimity(int32_t value, int32_t increvalue,int32_t decreasvalue);
double slopelimitLDuty(double value, double increvalue,double decreasvalue);
double slopelimitRDuty(double value, double increvalue,double decreasvalue);	
float Value_limitf(float min_value ,float current_value ,float max_value);
void initializeFilter(AverageFilter* filter);
float filterValue_float(AverageFilter *filter, float input);
int32_t filterValueX_int32(AverageFilter *filter, int16_t input);
int32_t filterValueY_int32(AverageFilter *filter, int16_t input);
void filterInit(void);
/* cubic polynomial interpolation */
extern void mcl_cubic_poly_intpol_st_init(mcl_cubic_poly_intpol_st *cubic_poly_intpol, float intpol_ts);
extern unsigned char mcl_cubic_poly_intpol(mcl_cubic_poly_intpol_st *cubic_poly_intpol, float init_value, float final_value, float time);
extern unsigned char mcl_ppp_cubic_poly_intpol(mcl_cubic_poly_intpol_st *cubic_poly_intpol, 
	                                           float init_value, float final_value, float time, float init_velocity, float final_velocity);
extern float mcl_cubic_poly_intpol_updata(mcl_cubic_poly_intpol_st *cubic_poly_intpol);

/* linear interpolation */
extern void mcl_line_poly_intpol_st_init(mcl_line_poly_intpol_st *line_poly_intpol, float intpol_ts);
extern unsigned char mcl_line_poly_intpol(mcl_line_poly_intpol_st *line_poly_intpol, float init_value, float final_value, float time);
extern float mcl_line_poly_intpol_updata(mcl_line_poly_intpol_st *line_poly_intpol);

/* parabola interpolation */
extern void mcl_parabola_poly_intpol_st_init(mcl_parabola_poly_intpol_st *parabola_poly_intpol, float intpol_ts);
extern unsigned char mcl_parabola_poly_intpol(mcl_parabola_poly_intpol_st *parabola_poly_intpol, float peak, float time1, float time2, float peak_time);
extern unsigned char mcl_parabola_poly_intpol_1(mcl_parabola_poly_intpol_st *parabola_poly_intpol, float start, float peak, float last, float time1, float time2, float peak_time);
extern unsigned char mcl_parabola_poly_intpol_2(mcl_parabola_poly_intpol_st *parabola_poly_intpol, float peak, float time1);
extern unsigned char mcl_parabola_poly_intpol_3(mcl_parabola_poly_intpol_st *parabola_poly_intpol, float peak, float time2);
extern unsigned char mcl_parabola_poly_intpol_4(mcl_parabola_poly_intpol_st *parabola_poly_intpol, float init_value, float peak, float time1);

extern float mcl_parabola_poly_intpol_updata(mcl_parabola_poly_intpol_st *parabola_poly_intpol);
extern float mcl_parabola_poly_intpol_updata_2(mcl_parabola_poly_intpol_st *parabola_poly_intpol);
extern float mcl_parabola_poly_intpol_updata_3(mcl_parabola_poly_intpol_st *parabola_poly_intpol);

uint16_t low_pass_filter1(uint16_t value,float alpha);
uint16_t low_pass_filter2(uint16_t value,float alpha);
uint16_t low_pass_filter3(uint16_t value,float alpha);
uint16_t low_pass_filter4(uint16_t value,float alpha);
uint16_t low_pass_filter5(uint16_t value,float alpha);
uint16_t low_pass_filter6(uint16_t value,float alpha);
uint16_t low_pass_filter7(uint16_t value,float alpha);
int16_t low_pass_filter8(int16_t value,float alpha);
int16_t low_pass_filter9(int16_t value,float alpha);
#endif


















