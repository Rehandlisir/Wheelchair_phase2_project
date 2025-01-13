/**
 * @FilePath     : /底盘闭环Demo板项目_MAXON单速度环/R9_407F_num_2/Drivers/BSP/Curve_planing/curve.h
 * @Description  :  
 * @Author       : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Version      : 0.0.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2025-01-07 18:00:41
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/
#ifndef __Curve_H
#define __Curve_H
#include "./SYSTEM/sys/sys.h"

/* 定义电机速度曲线类型枚举 */
typedef enum 
{
  CURVE_NONE=0,  //直启
  CURVE_TRAP=1,  //梯形曲线
  CURVE_SPTA=2  //S型曲线
}SpeedCurveType;

/* 定义电机速度曲线对象 */
typedef struct
{
  float startSpeed;    //开始调速时的初始速度
  float currentSpeed;   //当前速度
  float targetSpeed;   //目标速度
  float stepSpeed;    //加速度
  float speedMax;     //最大速度
  float speedMin;     //最小速度
  uint32_t aTimes;    //调速时间
  uint32_t maxTimes;   //调速跨度
  SpeedCurveType curveMode;  //曲线类型
  float flexible;     //S曲线拉伸度
}CurveObjectType;
void MotorVelocityCurve(CurveObjectType *curve);
void CalCurveNone(CurveObjectType* trap);
void CalCurveTRAP(CurveObjectType* trap);
void CalCurveSPTA(CurveObjectType* spta);
#endif
