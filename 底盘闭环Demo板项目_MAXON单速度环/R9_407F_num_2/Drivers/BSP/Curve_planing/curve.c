/**
 * @FilePath     : /底盘闭环Demo板项目_MAXON单速度环/R9_407F_num_2/Drivers/BSP/Curve_planing/curve.c
 * @Description  :  
 * @Author       : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Version      : 0.0.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2025-03-13 11:16:03
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/
#include "./BSP/Curve_planing/curve.h"

void (*pCalCurve[])(CurveObjectType* curve) = { CalCurveNone,CalCurveTRAP,CalCurveSPTA};
 
/* 电机曲线加减速操作-------------------------------------------------------- */
void MotorVelocityCurve(CurveObjectType *curve)
{
  float temp=0;
  
  if(curve->targetSpeed>curve->speedMax)
  {
     curve->targetSpeed=curve->speedMax;
  }
  
  if(curve->targetSpeed<curve->speedMin)
  {
     curve->targetSpeed=curve->speedMin;
  }
 
  if((fabs(curve->currentSpeed-curve->startSpeed)<=curve->stepSpeed)&&(curve->maxTimes==0))
  {
     if(curve->startSpeed<curve->speedMin)
     {
       curve->startSpeed=curve->speedMin;
     }
     
     temp=fabs(curve->targetSpeed-curve->startSpeed);
     temp=temp/curve->stepSpeed;
     curve->maxTimes=(uint32_t)(temp)+1;
     curve->aTimes=0;
  }
  
  if(curve->aTimes<curve->maxTimes)
  {
     pCalCurve[curve->curveMode](curve);
     curve->aTimes++;
  }
  else
  {
     curve->currentSpeed=curve->targetSpeed;
     curve->maxTimes=0;
     curve->aTimes=0;
  }
}
 
/*S型曲线速度计算*/
void CalCurveSPTA(CurveObjectType *spta)
{
  float power=0.0;
  float speed=0.0;
  
  power=(2*((float)spta->aTimes)-((float)spta->maxTimes))/((float)spta->maxTimes);
  power=(0.0-spta->flexible)*power;
  
  speed=1.0+expf(power);
  speed=(spta->targetSpeed-spta->startSpeed)/speed;
  spta->currentSpeed=speed+spta->startSpeed;
  
  if(spta->currentSpeed>spta->speedMax)
  {
     spta->currentSpeed=spta->speedMax;
  }
  
  if(spta->currentSpeed<spta->speedMin)
  {
     spta->currentSpeed=spta->speedMin;
  }
}
/*梯形曲线速度计算*/
void CalCurveTRAP(CurveObjectType* trap)
{
    float slope = 0.0;
    slope = (trap->targetSpeed - trap->startSpeed) / trap->maxTimes;
    trap->currentSpeed = trap->startSpeed + slope * trap->aTimes;
    if (trap->currentSpeed > trap->speedMax)
    {
    trap->currentSpeed = trap->speedMax;
    }
    if (trap->currentSpeed < trap->speedMin)
    {
        trap->currentSpeed = trap->speedMin;
    }
}       

/*直启速度计算*/
 void CalCurveNone(CurveObjectType* trap)
{
    ;
}

/*Call example*/
// int main() 
// {

//     CurveObjectType curve; //电机调速曲线
    // curve.curveMode = CURVE_SPTA;
    // curve.maxTimes = 1000;
    // curve.aTimes = 0;
    // curve.currentSpeed = 8;
    // curve.startSpeed = 8;
    // curve.targetSpeed = -9;
    // curve.stepSpeed = 3;
    // curve.speedMax = 12;
    // curve.speedMin = -12;
    // curve.flexible = 8;
//     while (curve.aTimes < 1000)
//     {
//         MotorVelocityCurve(&curve);
//     }
// }