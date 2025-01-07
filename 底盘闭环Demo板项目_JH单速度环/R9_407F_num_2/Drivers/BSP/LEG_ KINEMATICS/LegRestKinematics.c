#include "./BSP/LEG_ KINEMATICS/LegRestKinematics.h"
#include "arm_math.h"
#include "math.h"
S_LEGKINEMATICS_PRA legkinematicspra;
S_LEGKINEMATICS_PRA chairkinematicspra;
/*角度值与推杆长度映射*/
void legKinematics(void)
{
    /*腿托正运动学连杆参数*/
    legkinematicspra.L1 = 273.0;
    legkinematicspra.L2 = 310.0;
    legkinematicspra.L3 = 85.2;
    legkinematicspra.L4  =A3_p1*adcdata.leglength_pos+369.4;
    /*座椅正运动学连杆参数*/
    chairkinematicspra.L1=273.0;
    chairkinematicspra.L2=310.0;
    chairkinematicspra.L3 = 85.2;
    chairkinematicspra.L4 = 1.0;

    /*腿托正运动学角度参数*/
    legkinematicspra.DeltB1 = B1_p1*adcdata.lift_pos+B1_p2;
    legkinematicspra.DeltB2 = B2_p1*adcdata.pedestal_pos+B2_p2;
    legkinematicspra.DeltA2 = A2_p1*pow(adcdata.legangle_pos,2)+A2_p2*adcdata.legangle_pos+A2_p3;
    legkinematicspra.theta1deg=THET1_a1*arm_sin_f32(THET1_b1*legkinematicspra.DeltB1+THET1_c1) + \
    THET1_a2*arm_sin_f32(THET1_b2*legkinematicspra.DeltB1+THET1_c2)
    +THET1_a3*arm_sin_f32(THET1_b3*legkinematicspra.DeltB1+THET1_c3);
    legkinematicspra.theta1 = (180-legkinematicspra.theta1deg)*0.01745329;
    legkinematicspra.theta2deg=THET2_a1*arm_sin_f32(THET2_b1*legkinematicspra.DeltB1+THET2_c1) + \
    THET2_a2*arm_sin_f32(THET2_b2*legkinematicspra.DeltB1+THET2_c2) + \
    THET2_a3*arm_sin_f32(THET2_b3*legkinematicspra.DeltB1+THET2_c3);
    legkinematicspra.theta2 = (180+legkinematicspra.theta2deg)*0.01745329;
    legkinematicspra.theta3deg = THET3_p1*pow(legkinematicspra.DeltB2,3) + \
    THET3_p2*pow(legkinematicspra.DeltB2,2) + \
    THET3_p3*legkinematicspra.DeltB2 + THET3_p4;
    legkinematicspra.theta3 = (CONSTANTANGLE-legkinematicspra.theta3deg)*0.01745329;
    legkinematicspra.theta4deg = THET4_p1*pow(legkinematicspra.DeltA2,3) +\
    THET4_p2*pow(legkinematicspra.DeltA2,2) +\
    THET4_p3*legkinematicspra.DeltA2 +  THET4_p4+10.0;
    legkinematicspra.theta4 = (180+legkinematicspra.theta4deg)*0.01745329;
    legkinematicspra.theta5 = 0;
    /*座椅正运动学角度参数*/
    chairkinematicspra.theta1 = legkinematicspra.theta1;
    chairkinematicspra.theta2 = legkinematicspra.theta2;
    chairkinematicspra.theta3 =legkinematicspra.theta3;
    chairkinematicspra.theta4 =5.885249388;

    /*腿托正运动学计算*/
     float32_t pDataKinematicA1_0[16] = {arm_cos_f32(legkinematicspra.theta1),-arm_sin_f32(legkinematicspra.theta1),0,legkinematicspra.L1*arm_cos_f32(legkinematicspra.theta1),
                                        arm_sin_f32(legkinematicspra.theta1), arm_cos_f32(legkinematicspra.theta1),0,legkinematicspra.L1*arm_sin_f32(legkinematicspra.theta1),                                                                               
                                        0,0,1,0,0,0,0,1};                                                                                
                                                                                                                     
     float32_t pDataKinematicA2_1[16]  = {arm_cos_f32(legkinematicspra.theta2),-arm_sin_f32(legkinematicspra.theta2),0,legkinematicspra.L2*arm_cos_f32(legkinematicspra.theta2),
                                        arm_sin_f32(legkinematicspra.theta2), arm_cos_f32(legkinematicspra.theta2),0,legkinematicspra.L2*arm_sin_f32(legkinematicspra.theta2),                                                                              
                                        0,0,1,0,0,0,0,1};   
    static float32_t pDataDstA2_0[16];
    arm_matrix_instance_f32 pSrcA1_0; // 4 行 4列数据
    arm_matrix_instance_f32 pSrcA2_1; // 4 行 4列数据
    arm_matrix_instance_f32 A2_0pDst;
    pSrcA1_0.numCols = 4;
    pSrcA1_0.numRows = 4;
    pSrcA1_0.pData = pDataKinematicA1_0;
    pSrcA2_1.numCols = 4;
    pSrcA2_1.numRows = 4;
    pSrcA2_1.pData = pDataKinematicA2_1;
    A2_0pDst.numCols = 4;
    A2_0pDst.numRows = 4;
    A2_0pDst.pData = pDataDstA2_0;
    arm_mat_mult_f32(&pSrcA1_0, &pSrcA2_1, &A2_0pDst);

     float32_t pDataKinematicA3_2[16]  = {arm_cos_f32(legkinematicspra.theta3),-arm_sin_f32(legkinematicspra.theta3),0,legkinematicspra.L3*arm_cos_f32(legkinematicspra.theta3),
                                        arm_sin_f32(legkinematicspra.theta3), arm_cos_f32(legkinematicspra.theta3),0,legkinematicspra.L3*arm_sin_f32(legkinematicspra.theta3),                                                                              
                                        0,0,1,0,0,0,0,1};   
    static float32_t pDataDstA3_0[16];
    arm_matrix_instance_f32 pSrcA3_2; // 4 行 4列数据
    arm_matrix_instance_f32 A3_0pDst;
    pSrcA3_2.numCols = 4;
    pSrcA3_2.numRows = 4;
    pSrcA3_2.pData = pDataKinematicA3_2;
    A3_0pDst.numCols = 4;
    A3_0pDst.numRows = 4;
    A3_0pDst.pData = pDataDstA3_0;
    arm_mat_mult_f32(&A2_0pDst, &pSrcA3_2, &A3_0pDst);

    float32_t pDataKinematicA4_3[16]  = {arm_cos_f32(legkinematicspra.theta4),-arm_sin_f32(legkinematicspra.theta4),0,legkinematicspra.L4*arm_cos_f32(legkinematicspra.theta4),
                                        arm_sin_f32(legkinematicspra.theta4), arm_cos_f32(legkinematicspra.theta4),0,legkinematicspra.L4*arm_sin_f32(legkinematicspra.theta4),                                                                              
                                        0,0,1,0,0,0,0,1};   
    static float32_t pDataDstA4_0[16];
    arm_matrix_instance_f32 pSrcA4_3; // 4 行 4列数据
    arm_matrix_instance_f32 A4_0pDst;
    pSrcA4_3.numCols = 4;
    pSrcA4_3.numRows = 4;
    pSrcA4_3.pData = pDataKinematicA4_3;

    A4_0pDst.numCols = 4;
    A4_0pDst.numRows = 4;
    A4_0pDst.pData = pDataDstA4_0;
    arm_mat_mult_f32(&A3_0pDst, &pSrcA4_3, &A4_0pDst);
    legkinematicspra.x_o = pDataDstA4_0[3];
    legkinematicspra.y_o = pDataDstA4_0[7];
    legkinematicspra.z_o = pDataDstA4_0[11];

    // chairkinematicspra.y_o = pDataDstA3_0[7]+295 -420.0; // 参考点距离地面 距离  295  420 坐姿高度
    // chairkinematicspra.theta_x2X =acos(pDataDstA3_0[0])*57.29578 - 21.8;
    // printf("chair_yo:%f,chair_theta:%f\n",chairkinematicspra.y_o ,chairkinematicspra.theta_x2X);


    // /*****************************座板正运动学计算********************************/
     float32_t pDataKinematicT4_3[16]  = {0.9219,0.3875,0,0.9219,
                                         -0.3875, 0.9219,0,-0.3875,                                                                              
                                        0,0,1,0,0,0,0,1};    
    static float32_t pDataDstT4_0[16];
    arm_matrix_instance_f32 pSrcT4_3; // 4 行 4列数据
    arm_matrix_instance_f32 T4_0pDst;
    pSrcT4_3.numCols = 4;
    pSrcT4_3.numRows = 4;
    pSrcT4_3.pData = pDataKinematicT4_3;
    T4_0pDst.numCols = 4;
    T4_0pDst.numRows = 4;
    T4_0pDst.pData = pDataDstT4_0;

    A3_0pDst.numCols = 4;
    A3_0pDst.numRows=4;
    A3_0pDst.pData =pDataDstA3_0; 

    arm_mat_mult_f32(&A3_0pDst, &pSrcT4_3, &T4_0pDst);

    chairkinematicspra.y_o = pDataDstT4_0[7]-132.0; // 参考点距离地面 距离  295  420 坐姿高度
    chairkinematicspra.theta_x2X =acos(pDataDstT4_0[0])*57.29578;
    // printf("chair_yo:%f,chair_theta:%f\n",chairkinematicspra.y_o ,chairkinematicspra.theta_x2X);
    /******************************************************************* */

}
