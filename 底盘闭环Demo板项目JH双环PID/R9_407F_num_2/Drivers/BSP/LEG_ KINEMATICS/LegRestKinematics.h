#ifndef __LEGK__
#define __LEGK__
#include "./SYSTEM/sys/sys.h"
#include "arm_math.h"
#include "./BSP/R9/getadcdata.h"

// #define L1  100.11
// #define L2  100.11
// #define L3  100.11

#define CONSTANTANGLE 22.8  /*连杆3 与座椅水平面夹角*/
/*举升推杆B1- AD值- 伸长量函数参数f(x) =  p1*x + p2*/
#define B1_p1 0.06752
#define B1_p2 -6.926
/*举升推杆THETA1角度- 伸长量函数参数发f(x) =a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3)*/
#define THET1_a1 226.8
#define THET1_b1 0.03201
#define THET1_c1 -0.3975   
#define THET1_a2 193.1
#define THET1_b2 0.03828
#define THET1_c2 2.494
#define THET1_a3 15
#define THET1_b3 0.05928
#define THET1_c3 4.745
/*举升推杆THETA2角度- 伸长量函数参数f(x) =a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3)*/
#define THET2_a1    336.1
#define THET2_b1    0.03145
#define THET2_c1    -0.4299 
#define THET2_a2    271.8
#define THET2_b2    0.0399
#define THET2_c2    2.371
#define THET2_a3    29.74
#define THET2_b3    0.05979
#define THET2_c3    4.654


/*座板推杆B2- AD值- 伸长量函数参数f(x) =  p1*x + p2*/
#define B2_p1 0.04809
#define B2_p2 -0.1984
/*座板推杆THETA3- 伸长量函数参数  f(x) = p1*x^3 + p2*x^2 + p3*x + p4*/
#define THET3_p1 1.448e-05
#define THET3_p2 -0.002962
#define THET3_p3 0.7204
#define THET3_p4 6.177

/*腿托旋转推杆A2- AD值- 伸长量函数参数 f(x) = p1*x^2 + p2*x + p3*/
#define A2_p1 1.088e-06
#define A2_p2 0.04495
#define A2_p3 -1.119

/*腿托旋转推杆theta4- 伸长量函数参数p1*x^3 + p2*x^2 + p3*x + p4*/
#define THET4_p1 1.336e-05
#define THET4_p2 -0.002341
#define THET4_p3  0.704
#define THET4_p4  52.29


/*腿托绝对长度- AD值函数参数 f(x) = p1*x + p2*/
#define A3_p1 0.06538
#define A3_p2 369.4


typedef struct 
{
    float32_t L1;
    float32_t L2;
    float32_t L3;
    float32_t L4; // 连杆长度 
    float32_t DeltB1;
    float32_t DeltB2;
    float32_t DeltA2; //推杆伸长量 与AD值之间的关系
    float32_t theta1deg;
    float32_t theta2deg;
    float32_t theta3deg;
    float32_t theta4deg;
    float32_t theta5deg; //测量角度 单位度
    float32_t theta1;
    float32_t theta2;
    float32_t theta3;
    float32_t theta4;
    float32_t theta5; //测量角度 单位度
    float32_t x_o;
    float32_t y_o;
    float32_t z_o;//正运动学 位置数据
    float32_t theta_x2X;

}S_LEGKINEMATICS_PRA;
extern S_LEGKINEMATICS_PRA legkinematicspra;
extern S_LEGKINEMATICS_PRA chairkinematicspra;
void legKinematics(void);
#endif

