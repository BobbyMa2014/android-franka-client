#include <climits>
#include <iostream>
#include <cmath>
#include "Eigen/Geometry"
#include <cstdio>
#include <cstring>
#include <cstdio>
#include <jni.h>
#include <string>

#define joint1_max 2.8973
#define joint1_min -2.8973
#define joint2_max 1.7628
#define joint2_min -1.7628
#define joint3_max 2.8973
#define joint3_min -2.8973
#define joint4_max -0.0698
#define joint4_min -3.0718
#define joint5_max 2.8973
#define joint5_min -2.8973
#define joint6_max 3.7525
#define joint6_min -0.0175
#define joint7_max 2.8973
#define joint7_min -2.8973
#define Initial_state {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI, M_PI_4}
#define qx 0
#define qy -0.7
#define qz 0
#define qw -0.7

using namespace Eigen;

__unused void Fun_IK(double x, double y, double z) {
    double receive[7] = Initial_state;
    // MatrixXd m(2,2);//<double, 3, 3> R720;
    Matrix<double, 3, 3> R720;
    //R720 << cos(qx)*cos(qy)*cos(qz)-sin(qx)*sin(qz),-cos(qx)*cos(qy)*sin(qz)-sin(qx)*cos(qz),cos(qx)*sin(qy),
    //    sin(qx)* cos(qy)* cos(qz) +cos(qx) * sin(qz),  -sin(qx) * cos(qy) * sin(qz) + cos(qx) * cos(qz),sin(qx)*cos(qy),
    //    -sin(qy)*cos(qz),sin(qy)*sin(qz),cos(qy);
    // R720 << cos(alpha) * cos(beta), cos(alpha)* sin(beta)* sin(gamma) - sin(alpha) * cos(gamma), cos(alpha)* sin(beta)* cos(gamma) + sin(alpha) * sin(gamma),
    //     sin(alpha)* cos(beta), sin(alpha)* sin(beta)* sin(gamma) + cos(alpha) * cos(gamma), sin(alpha)* sin(beta)* cos(gamma) - cos(alpha) * sin(gamma),
    //     -sin(beta), cos(beta)* sin(gamma), cos(beta)* cos(gamma);
    R720 << 1 - 2 * (qz * qz + qy * qy), 2 * (qx * qy - qw * qz), 2 * (qw * qy + qx * qz),
            2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
            2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy);

    //R720 << 1 - 2 * (qz * qz + qy * qy), 2 * (qx * qy + qw * qz), 2 * (qx * qz - qw * qy),
    //   2 * (qx * qy - qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qw * qx + qy * qz),
    //   2 * (qw * qy + qx * qz), 2 * (qy * qz - qw * qx), 1 - 2 * (qx * qx + qy * qy);

    double d_0_A = 0.333, d_A_B = 0.316, d_B_C = 0.384, d_D_f = 0.107, d_C = 0.088;
    //double d_effector = 0.066
    //d_D_f = d_D_f + d_effector;
    //Vector3d V_C(-d_C,0,-d_D_f);
    //Vector3d V_C(-d_C, 0, 0);
    Vector3d V_C_D_Fcart(-d_C, 0, -d_D_f);
    double x_C = R720.row(0).dot(V_C_D_Fcart) + x;
    double y_C = R720.row(1).dot(V_C_D_Fcart) + y;
    double z_C = R720.row(2).dot(V_C_D_Fcart) + z;
    /*
    double x_D = -d_D_f * cos(qx) * sin(qy) + x;
    double y_D = -d_D_f * sin(qx) * sin(qy) + y;
    double z_D = -d_D_f * cos(qy) + z;*/
    double Se = 0;
    double d_A_C = sqrt(x_C * x_C + y_C * y_C + (z_C - 0.333) * (z_C - 0.333));
    double d_C_z0 = sqrt(x_C * x_C + y_C * y_C);
    double x_E = x_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double y_E = y_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double z_E = 0.333 + (z_C - 0.333) * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double t = d_A_B * sin(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C)));
    double x_B = (t * cos(Se)) * (-x_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (y_C / d_C_z0) + x_E;
    double y_B = (t * cos(Se)) * (-y_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (x_C / d_C_z0) + y_E;
    double z_B = (t * cos(Se)) * ((y_C * y_C + x_C * x_C) / (d_C_z0 * d_A_C)) + z_E;
    //  Vector3d V_A_B0(x_B, y_B, 0);
    double Theta_1 = acos(x_B / sqrt(x_B * x_B + y_B * y_B));
    Theta_1 = acos(x_C / sqrt(x_C * x_C + y_C * y_C));
    // if (x_B * y_B < 0)
    //      Theta_1 = -Theta_1;

    if (y_C < 0)
        Theta_1 = -Theta_1;

    double Theta_4 = acos((d_A_B * d_A_B + d_B_C * d_B_C - d_A_C * d_A_C) / (2 * d_A_B * d_B_C));
    Theta_4 = Theta_4 - M_PI;

    double theta_temp = 0, d_temp = 0, aphla_temp = 0, a_temp = 0;
    theta_temp = Theta_1;
    aphla_temp = 0;
    Matrix<double, 3, 3> R120;
    R120 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    // R120 << R120.transpose();

    Vector3d V_A_B(x_B - 0, y_B - 0, z_B - 0.333);
    Vector3d V_A_B_1cart = R120.transpose() * V_A_B;
    double Theta_2 = acos(V_A_B_1cart(2) / sqrt(V_A_B_1cart.dot(V_A_B_1cart)));
    if (V_A_B_1cart(0) < 0)
        Theta_2 = -Theta_2;

    //Vector3d V_D_virtual(-d_C, 0, 0);
    //   Vector3d V_D_virtual(-d_C, 0, 0);
    // double x_D_virtual = R720.row(0)*( V_D_virtual)+x;
    //  double y_D_virtual = R720.row(1)*(V_D_virtual)+y;
    //  double z_D_virtual = R720.row(2)*(V_D_virtual)+z;
    Vector3d V_B_C(x_C - x_B, y_C - y_B, z_C - z_B);
    //   Vector3d V_D_virtual_C(x_C- x_D_virtual,y_C- y_D_virtual,z_C- z_D_virtual);
    //Vector3d V_C_D(x_D-x_C,y_D-y_C,z_D-z_C);
    Vector3d V_D_original(0, 0, -d_D_f);
    double x_D_original = R720.row(0) * (V_D_original)+x;
    double y_D_original = R720.row(1) * (V_D_original)+y;
    double z_D_original = R720.row(2) * (V_D_original)+z;

    Vector3d V_D_D_original(x_D_original - x, y_D_original - y, z_D_original - z);
    double Theta_6 = acos(V_B_C.dot(V_D_D_original) / sqrt(V_B_C.dot(V_B_C) * V_D_D_original.dot(V_D_D_original)));
    //Theta_6 = joint6_max - Theta_6;
    //printf("The B is %f,%f,%f, The C is %f,%f,%f, The D_V is %f,%f,%f \n", x_B, y_B, z_B, x_C, y_C, z_C, x_D_virtual, y_D_virtual, z_D_virtual);

    Matrix<double, 3, 3> R221;
    theta_temp = Theta_2;
    aphla_temp = -M_PI / 2;
    R221 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    // R221 << R221.transpose();
    Matrix<double, 3, 3> R220;
    R220 = R120 * R221;
    Vector3d V_B_C_2cart = R220.transpose() * V_B_C; // Vector_BC in the second cart. May error
    // R022* V_B_C
    double Theta_3 = acos(V_B_C_2cart(0) / sqrt(V_B_C_2cart(0) * V_B_C_2cart(0) + V_B_C_2cart(2) * V_B_C_2cart(2)));
    if (V_B_C_2cart(2) > 0) {
        Theta_3 = -Theta_3;
    }
    Matrix<double, 3, 3> R322;

    theta_temp = Theta_3;
    aphla_temp = M_PI / 2;
    R322 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    //R322 << R322.transpose();
    Matrix<double, 3, 3> R423;
    theta_temp = Theta_4;
    aphla_temp = M_PI / 2;
    R423 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    // R423 << R423.transpose();

    Matrix<double, 3, 3> R420;
    R420 = R220 * R322 * R423;

    Vector3d V_C_D(x - x_C, y - y_C, z - z_C);

    Vector3d V_C_D_4cart = R420.transpose() * V_C_D; // Vector_CD in the 4 cart. May error
    double Theta_5 = acos(V_C_D_4cart(0) / sqrt(V_C_D_4cart(0) * V_C_D_4cart(0) + V_C_D_4cart(2) * V_C_D_4cart(2)));
    if (V_C_D_4cart(2) > 0)
        Theta_5 = -Theta_5;
    double Theta_7 = 0;

    if (Theta_7 <= joint7_max && Theta_7 >= joint7_min &&
        Theta_6 <= joint6_max && Theta_6 >= joint6_min &&
        Theta_5 <= joint5_max && Theta_5 >= joint5_min &&
        Theta_4 <= joint4_max && Theta_4 >= joint4_min &&
        Theta_3 <= joint3_max && Theta_3 >= joint3_min &&
        Theta_2 <= joint2_max && Theta_2 >= joint2_min &&
        Theta_1 <= joint1_max && Theta_1 >= joint1_min)//&& fabs(Theta_3)<0.01 )
    {
        receive[0] = Theta_1;
        receive[1] = Theta_2;
        receive[2] = Theta_3;
        receive[3] = Theta_4;
        receive[4] = Theta_5;
        receive[5] = Theta_6;
        receive[6] = Theta_7;
    }
    // Theta_return = { Theta_1,Theta_2,Theta_3,Theta_4,Theta_5,Theta_6,Theta_7};
    // return  0;
    //  return Theta_return;

    if (receive[0] > joint1_max)
        receive[0] = joint1_max;
    if (receive[0] < joint1_min)
        receive[0] = joint1_min;
    if (receive[1] > joint2_max)
        receive[1] = joint2_max;
    if (receive[1] < joint2_min)
        receive[1] = joint2_min;
    if (receive[2] > joint3_max)
        receive[2] = joint3_max;
    if (receive[2] < joint3_min)
        receive[2] = joint3_min;
    if (receive[3] > joint4_max)
        receive[3] = joint4_max;
    if (receive[3] < joint4_min)
        receive[3] = joint4_min;
    if (receive[4] > joint5_max)
        receive[4] = joint5_max;
    if (receive[4] < joint5_min)
        receive[4] = joint5_min;
    if (receive[5] > joint6_max)
        receive[5] = joint6_max;
    if (receive[5] < joint6_min)
        receive[5] = joint6_min;
    if (receive[6] > joint7_max)
        receive[6] = joint7_max;
    if (receive[6] < joint7_min)
        receive[6] = joint7_min;
    //   return Theta_return[0],Theta_return[1],Theta_return[2],Theta_return[3],Theta_return[4],Theta_return[5],Theta_return[6];
}

extern "C"
JNIEXPORT jdouble JNICALL
Java_com_example_myapplication3_MainActivity_IK_1joint1(JNIEnv *env, jobject thiz, jdouble x,
                                                        jdouble y, jdouble z) {
    double receive[7] = Initial_state;
    Matrix<double, 3, 3> R720;
    R720 << 1 - 2 * (qz * qz + qy * qy), 2 * (qx * qy - qw * qz), 2 * (qw * qy + qx * qz),
            2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
            2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy);
    double d_0_A = 0.333, d_A_B = 0.316, d_B_C = 0.384, d_D_f = 0.107, d_C = 0.088;
    Vector3d V_C_D_Fcart(-d_C, 0, -d_D_f);
    double x_C = R720.row(0).dot(V_C_D_Fcart) + x;
    double y_C = R720.row(1).dot(V_C_D_Fcart) + y;
    double z_C = R720.row(2).dot(V_C_D_Fcart) + z;
    double Se = 0;
    double d_A_C = sqrt(x_C * x_C + y_C * y_C + (z_C - 0.333) * (z_C - 0.333));
    double d_C_z0 = sqrt(x_C * x_C + y_C * y_C);
    double x_E = x_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double y_E = y_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double z_E = 0.333 + (z_C - 0.333) * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double t = d_A_B * sin(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C)));
    double x_B = (t * cos(Se)) * (-x_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (y_C / d_C_z0) + x_E;
    double y_B = (t * cos(Se)) * (-y_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (x_C / d_C_z0) + y_E;
    double z_B = (t * cos(Se)) * ((y_C * y_C + x_C * x_C) / (d_C_z0 * d_A_C)) + z_E;
    double Theta_1 = acos(x_B / sqrt(x_B * x_B + y_B * y_B));
    Theta_1 = acos(x_C / sqrt(x_C * x_C + y_C * y_C));
    if (y_C < 0)
        Theta_1 = -Theta_1;
    double Theta_4 = acos((d_A_B * d_A_B + d_B_C * d_B_C - d_A_C * d_A_C) / (2 * d_A_B * d_B_C));
    Theta_4 = Theta_4 - M_PI;
    double theta_temp = 0, d_temp = 0, aphla_temp = 0, a_temp = 0;
    theta_temp = Theta_1;
    aphla_temp = 0;
    Matrix<double, 3, 3> R120;
    R120 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Vector3d V_A_B(x_B - 0, y_B - 0, z_B - 0.333);
    Vector3d V_A_B_1cart = R120.transpose() * V_A_B;
    double Theta_2 = acos(V_A_B_1cart(2) / sqrt(V_A_B_1cart.dot(V_A_B_1cart)));
    if (V_A_B_1cart(0) < 0)
        Theta_2 = -Theta_2;
    Vector3d V_B_C(x_C - x_B, y_C - y_B, z_C - z_B);
    Vector3d V_D_original(0, 0, -d_D_f);
    double x_D_original = R720.row(0) * (V_D_original)+x;
    double y_D_original = R720.row(1) * (V_D_original)+y;
    double z_D_original = R720.row(2) * (V_D_original)+z;
    Vector3d V_D_D_original(x_D_original - x, y_D_original - y, z_D_original - z);
    double Theta_6 = acos(V_B_C.dot(V_D_D_original) / sqrt(V_B_C.dot(V_B_C) * V_D_D_original.dot(V_D_D_original)));
    Matrix<double, 3, 3> R221;
    theta_temp = Theta_2;
    aphla_temp = -M_PI / 2;
    R221 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R220;
    R220 = R120 * R221;
    Vector3d V_B_C_2cart = R220.transpose() * V_B_C; // Vector_BC in the second cart. May error
    double Theta_3 = acos(V_B_C_2cart(0) / sqrt(V_B_C_2cart(0) * V_B_C_2cart(0) + V_B_C_2cart(2) * V_B_C_2cart(2)));
    if (V_B_C_2cart(2) > 0) {
        Theta_3 = -Theta_3;
    }
    Matrix<double, 3, 3> R322;
    theta_temp = Theta_3;
    aphla_temp = M_PI / 2;
    R322 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R423;
    theta_temp = Theta_4;
    aphla_temp = M_PI / 2;
    R423 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R420;
    R420 = R220 * R322 * R423;
    Vector3d V_C_D(x - x_C, y - y_C, z - z_C);
    Vector3d V_C_D_4cart = R420.transpose() * V_C_D; // Vector_CD in the 4 cart. May error
    double Theta_5 = acos(V_C_D_4cart(0) / sqrt(V_C_D_4cart(0) * V_C_D_4cart(0) + V_C_D_4cart(2) * V_C_D_4cart(2)));
    if (V_C_D_4cart(2) > 0)
        Theta_5 = -Theta_5;
    double Theta_7 = 0;
    if (Theta_7 <= joint7_max && Theta_7 >= joint7_min &&
        Theta_6 <= joint6_max && Theta_6 >= joint6_min &&
        Theta_5 <= joint5_max && Theta_5 >= joint5_min &&
        Theta_4 <= joint4_max && Theta_4 >= joint4_min &&
        Theta_3 <= joint3_max && Theta_3 >= joint3_min &&
        Theta_2 <= joint2_max && Theta_2 >= joint2_min &&
        Theta_1 <= joint1_max && Theta_1 >= joint1_min)//&& fabs(Theta_3)<0.01 )
    {
        receive[0] = Theta_1;
        receive[1] = Theta_2;
        receive[2] = Theta_3;
        receive[3] = Theta_4;
        receive[4] = Theta_5;
        receive[5] = Theta_6;
        receive[6] = Theta_7;
    }
    if (receive[0] > joint1_max)
        receive[0] = joint1_max;
    if (receive[0] < joint1_min)
        receive[0] = joint1_min;
    return receive[0];
}

extern "C"
JNIEXPORT jdouble JNICALL
Java_com_example_myapplication3_MainActivity_IK_1joint2(JNIEnv *env, jobject thiz, jdouble x,
                                                        jdouble y, jdouble z) {
    double receive[7] = Initial_state;
    Matrix<double, 3, 3> R720;
    R720 << 1 - 2 * (qz * qz + qy * qy), 2 * (qx * qy - qw * qz), 2 * (qw * qy + qx * qz),
            2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
            2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy);
    double d_0_A = 0.333, d_A_B = 0.316, d_B_C = 0.384, d_D_f = 0.107, d_C = 0.088;
    Vector3d V_C_D_Fcart(-d_C, 0, -d_D_f);
    double x_C = R720.row(0).dot(V_C_D_Fcart) + x;
    double y_C = R720.row(1).dot(V_C_D_Fcart) + y;
    double z_C = R720.row(2).dot(V_C_D_Fcart) + z;
    double Se = 0;
    double d_A_C = sqrt(x_C * x_C + y_C * y_C + (z_C - 0.333) * (z_C - 0.333));
    double d_C_z0 = sqrt(x_C * x_C + y_C * y_C);
    double x_E = x_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double y_E = y_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double z_E = 0.333 + (z_C - 0.333) * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double t = d_A_B * sin(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C)));
    double x_B = (t * cos(Se)) * (-x_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (y_C / d_C_z0) + x_E;
    double y_B = (t * cos(Se)) * (-y_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (x_C / d_C_z0) + y_E;
    double z_B = (t * cos(Se)) * ((y_C * y_C + x_C * x_C) / (d_C_z0 * d_A_C)) + z_E;
    double Theta_1 = acos(x_B / sqrt(x_B * x_B + y_B * y_B));
    Theta_1 = acos(x_C / sqrt(x_C * x_C + y_C * y_C));
    if (y_C < 0)
        Theta_1 = -Theta_1;
    double Theta_4 = acos((d_A_B * d_A_B + d_B_C * d_B_C - d_A_C * d_A_C) / (2 * d_A_B * d_B_C));
    Theta_4 = Theta_4 - M_PI;
    double theta_temp = 0, d_temp = 0, aphla_temp = 0, a_temp = 0;
    theta_temp = Theta_1;
    aphla_temp = 0;
    Matrix<double, 3, 3> R120;
    R120 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Vector3d V_A_B(x_B - 0, y_B - 0, z_B - 0.333);
    Vector3d V_A_B_1cart = R120.transpose() * V_A_B;
    double Theta_2 = acos(V_A_B_1cart(2) / sqrt(V_A_B_1cart.dot(V_A_B_1cart)));
    if (V_A_B_1cart(0) < 0)
        Theta_2 = -Theta_2;
    Vector3d V_B_C(x_C - x_B, y_C - y_B, z_C - z_B);
    Vector3d V_D_original(0, 0, -d_D_f);
    double x_D_original = R720.row(0) * (V_D_original)+x;
    double y_D_original = R720.row(1) * (V_D_original)+y;
    double z_D_original = R720.row(2) * (V_D_original)+z;
    Vector3d V_D_D_original(x_D_original - x, y_D_original - y, z_D_original - z);
    double Theta_6 = acos(V_B_C.dot(V_D_D_original) / sqrt(V_B_C.dot(V_B_C) * V_D_D_original.dot(V_D_D_original)));
    Matrix<double, 3, 3> R221;
    theta_temp = Theta_2;
    aphla_temp = -M_PI / 2;
    R221 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R220;
    R220 = R120 * R221;
    Vector3d V_B_C_2cart = R220.transpose() * V_B_C; // Vector_BC in the second cart. May error
    double Theta_3 = acos(V_B_C_2cart(0) / sqrt(V_B_C_2cart(0) * V_B_C_2cart(0) + V_B_C_2cart(2) * V_B_C_2cart(2)));
    if (V_B_C_2cart(2) > 0) {
        Theta_3 = -Theta_3;
    }
    Matrix<double, 3, 3> R322;
    theta_temp = Theta_3;
    aphla_temp = M_PI / 2;
    R322 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R423;
    theta_temp = Theta_4;
    aphla_temp = M_PI / 2;
    R423 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R420;
    R420 = R220 * R322 * R423;
    Vector3d V_C_D(x - x_C, y - y_C, z - z_C);
    Vector3d V_C_D_4cart = R420.transpose() * V_C_D; // Vector_CD in the 4 cart. May error
    double Theta_5 = acos(V_C_D_4cart(0) / sqrt(V_C_D_4cart(0) * V_C_D_4cart(0) + V_C_D_4cart(2) * V_C_D_4cart(2)));
    if (V_C_D_4cart(2) > 0)
        Theta_5 = -Theta_5;
    double Theta_7 = 0;
    if (Theta_7 <= joint7_max && Theta_7 >= joint7_min &&
        Theta_6 <= joint6_max && Theta_6 >= joint6_min &&
        Theta_5 <= joint5_max && Theta_5 >= joint5_min &&
        Theta_4 <= joint4_max && Theta_4 >= joint4_min &&
        Theta_3 <= joint3_max && Theta_3 >= joint3_min &&
        Theta_2 <= joint2_max && Theta_2 >= joint2_min &&
        Theta_1 <= joint1_max && Theta_1 >= joint1_min)//&& fabs(Theta_3)<0.01 )
    {
        receive[0] = Theta_1;
        receive[1] = Theta_2;
        receive[2] = Theta_3;
        receive[3] = Theta_4;
        receive[4] = Theta_5;
        receive[5] = Theta_6;
        receive[6] = Theta_7;
    }
    if (receive[1] > joint2_max)
        receive[1] = joint2_max;
    if (receive[1] < joint2_min)
        receive[1] = joint2_min;
    return receive[1];
}

extern "C"
JNIEXPORT jdouble JNICALL
Java_com_example_myapplication3_MainActivity_IK_1joint3(JNIEnv *env, jobject thiz, jdouble x,
                                                        jdouble y, jdouble z) {
    double receive[7] = Initial_state;
    Matrix<double, 3, 3> R720;
    R720 << 1 - 2 * (qz * qz + qy * qy), 2 * (qx * qy - qw * qz), 2 * (qw * qy + qx * qz),
            2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
            2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy);
    double d_0_A = 0.333, d_A_B = 0.316, d_B_C = 0.384, d_D_f = 0.107, d_C = 0.088;
    Vector3d V_C_D_Fcart(-d_C, 0, -d_D_f);
    double x_C = R720.row(0).dot(V_C_D_Fcart) + x;
    double y_C = R720.row(1).dot(V_C_D_Fcart) + y;
    double z_C = R720.row(2).dot(V_C_D_Fcart) + z;
    double Se = 0;
    double d_A_C = sqrt(x_C * x_C + y_C * y_C + (z_C - 0.333) * (z_C - 0.333));
    double d_C_z0 = sqrt(x_C * x_C + y_C * y_C);
    double x_E = x_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double y_E = y_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double z_E = 0.333 + (z_C - 0.333) * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double t = d_A_B * sin(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C)));
    double x_B = (t * cos(Se)) * (-x_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (y_C / d_C_z0) + x_E;
    double y_B = (t * cos(Se)) * (-y_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (x_C / d_C_z0) + y_E;
    double z_B = (t * cos(Se)) * ((y_C * y_C + x_C * x_C) / (d_C_z0 * d_A_C)) + z_E;
    double Theta_1 = acos(x_B / sqrt(x_B * x_B + y_B * y_B));
    Theta_1 = acos(x_C / sqrt(x_C * x_C + y_C * y_C));
    if (y_C < 0)
        Theta_1 = -Theta_1;
    double Theta_4 = acos((d_A_B * d_A_B + d_B_C * d_B_C - d_A_C * d_A_C) / (2 * d_A_B * d_B_C));
    Theta_4 = Theta_4 - M_PI;
    double theta_temp = 0, d_temp = 0, aphla_temp = 0, a_temp = 0;
    theta_temp = Theta_1;
    aphla_temp = 0;
    Matrix<double, 3, 3> R120;
    R120 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Vector3d V_A_B(x_B - 0, y_B - 0, z_B - 0.333);
    Vector3d V_A_B_1cart = R120.transpose() * V_A_B;
    double Theta_2 = acos(V_A_B_1cart(2) / sqrt(V_A_B_1cart.dot(V_A_B_1cart)));
    if (V_A_B_1cart(0) < 0)
        Theta_2 = -Theta_2;
    Vector3d V_B_C(x_C - x_B, y_C - y_B, z_C - z_B);
    Vector3d V_D_original(0, 0, -d_D_f);
    double x_D_original = R720.row(0) * (V_D_original)+x;
    double y_D_original = R720.row(1) * (V_D_original)+y;
    double z_D_original = R720.row(2) * (V_D_original)+z;
    Vector3d V_D_D_original(x_D_original - x, y_D_original - y, z_D_original - z);
    double Theta_6 = acos(V_B_C.dot(V_D_D_original) / sqrt(V_B_C.dot(V_B_C) * V_D_D_original.dot(V_D_D_original)));
    Matrix<double, 3, 3> R221;
    theta_temp = Theta_2;
    aphla_temp = -M_PI / 2;
    R221 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R220;
    R220 = R120 * R221;
    Vector3d V_B_C_2cart = R220.transpose() * V_B_C; // Vector_BC in the second cart. May error
    double Theta_3 = acos(V_B_C_2cart(0) / sqrt(V_B_C_2cart(0) * V_B_C_2cart(0) + V_B_C_2cart(2) * V_B_C_2cart(2)));
    if (V_B_C_2cart(2) > 0) {
        Theta_3 = -Theta_3;
    }
    Matrix<double, 3, 3> R322;
    theta_temp = Theta_3;
    aphla_temp = M_PI / 2;
    R322 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R423;
    theta_temp = Theta_4;
    aphla_temp = M_PI / 2;
    R423 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R420;
    R420 = R220 * R322 * R423;
    Vector3d V_C_D(x - x_C, y - y_C, z - z_C);
    Vector3d V_C_D_4cart = R420.transpose() * V_C_D; // Vector_CD in the 4 cart. May error
    double Theta_5 = acos(V_C_D_4cart(0) / sqrt(V_C_D_4cart(0) * V_C_D_4cart(0) + V_C_D_4cart(2) * V_C_D_4cart(2)));
    if (V_C_D_4cart(2) > 0)
        Theta_5 = -Theta_5;
    double Theta_7 = 0;
    if (Theta_7 <= joint7_max && Theta_7 >= joint7_min &&
        Theta_6 <= joint6_max && Theta_6 >= joint6_min &&
        Theta_5 <= joint5_max && Theta_5 >= joint5_min &&
        Theta_4 <= joint4_max && Theta_4 >= joint4_min &&
        Theta_3 <= joint3_max && Theta_3 >= joint3_min &&
        Theta_2 <= joint2_max && Theta_2 >= joint2_min &&
        Theta_1 <= joint1_max && Theta_1 >= joint1_min)//&& fabs(Theta_3)<0.01 )
    {
        receive[0] = Theta_1;
        receive[1] = Theta_2;
        receive[2] = Theta_3;
        receive[3] = Theta_4;
        receive[4] = Theta_5;
        receive[5] = Theta_6;
        receive[6] = Theta_7;
    }
    if (receive[2] > joint3_max)
        receive[2] = joint3_max;
    if (receive[2] < joint3_min)
        receive[2] = joint3_min;
    return receive[2];
}

extern "C"
JNIEXPORT jdouble JNICALL
Java_com_example_myapplication3_MainActivity_IK_1joint4(JNIEnv *env, jobject thiz, jdouble x,
                                                        jdouble y, jdouble z) {
    double receive[7] = Initial_state;
    Matrix<double, 3, 3> R720;
    R720 << 1 - 2 * (qz * qz + qy * qy), 2 * (qx * qy - qw * qz), 2 * (qw * qy + qx * qz),
            2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
            2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy);
    double d_0_A = 0.333, d_A_B = 0.316, d_B_C = 0.384, d_D_f = 0.107, d_C = 0.088;
    Vector3d V_C_D_Fcart(-d_C, 0, -d_D_f);
    double x_C = R720.row(0).dot(V_C_D_Fcart) + x;
    double y_C = R720.row(1).dot(V_C_D_Fcart) + y;
    double z_C = R720.row(2).dot(V_C_D_Fcart) + z;
    double Se = 0;
    double d_A_C = sqrt(x_C * x_C + y_C * y_C + (z_C - 0.333) * (z_C - 0.333));
    double d_C_z0 = sqrt(x_C * x_C + y_C * y_C);
    double x_E = x_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double y_E = y_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double z_E = 0.333 + (z_C - 0.333) * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double t = d_A_B * sin(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C)));
    double x_B = (t * cos(Se)) * (-x_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (y_C / d_C_z0) + x_E;
    double y_B = (t * cos(Se)) * (-y_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (x_C / d_C_z0) + y_E;
    double z_B = (t * cos(Se)) * ((y_C * y_C + x_C * x_C) / (d_C_z0 * d_A_C)) + z_E;
    double Theta_1 = acos(x_B / sqrt(x_B * x_B + y_B * y_B));
    Theta_1 = acos(x_C / sqrt(x_C * x_C + y_C * y_C));
    if (y_C < 0)
        Theta_1 = -Theta_1;
    double Theta_4 = acos((d_A_B * d_A_B + d_B_C * d_B_C - d_A_C * d_A_C) / (2 * d_A_B * d_B_C));
    Theta_4 = Theta_4 - M_PI;
    double theta_temp = 0, d_temp = 0, aphla_temp = 0, a_temp = 0;
    theta_temp = Theta_1;
    aphla_temp = 0;
    Matrix<double, 3, 3> R120;
    R120 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Vector3d V_A_B(x_B - 0, y_B - 0, z_B - 0.333);
    Vector3d V_A_B_1cart = R120.transpose() * V_A_B;
    double Theta_2 = acos(V_A_B_1cart(2) / sqrt(V_A_B_1cart.dot(V_A_B_1cart)));
    if (V_A_B_1cart(0) < 0)
        Theta_2 = -Theta_2;
    Vector3d V_B_C(x_C - x_B, y_C - y_B, z_C - z_B);
    Vector3d V_D_original(0, 0, -d_D_f);
    double x_D_original = R720.row(0) * (V_D_original)+x;
    double y_D_original = R720.row(1) * (V_D_original)+y;
    double z_D_original = R720.row(2) * (V_D_original)+z;
    Vector3d V_D_D_original(x_D_original - x, y_D_original - y, z_D_original - z);
    double Theta_6 = acos(V_B_C.dot(V_D_D_original) / sqrt(V_B_C.dot(V_B_C) * V_D_D_original.dot(V_D_D_original)));
    Matrix<double, 3, 3> R221;
    theta_temp = Theta_2;
    aphla_temp = -M_PI / 2;
    R221 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R220;
    R220 = R120 * R221;
    Vector3d V_B_C_2cart = R220.transpose() * V_B_C; // Vector_BC in the second cart. May error
    double Theta_3 = acos(V_B_C_2cart(0) / sqrt(V_B_C_2cart(0) * V_B_C_2cart(0) + V_B_C_2cart(2) * V_B_C_2cart(2)));
    if (V_B_C_2cart(2) > 0) {
        Theta_3 = -Theta_3;
    }
    Matrix<double, 3, 3> R322;
    theta_temp = Theta_3;
    aphla_temp = M_PI / 2;
    R322 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R423;
    theta_temp = Theta_4;
    aphla_temp = M_PI / 2;
    R423 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R420;
    R420 = R220 * R322 * R423;
    Vector3d V_C_D(x - x_C, y - y_C, z - z_C);
    Vector3d V_C_D_4cart = R420.transpose() * V_C_D; // Vector_CD in the 4 cart. May error
    double Theta_5 = acos(V_C_D_4cart(0) / sqrt(V_C_D_4cart(0) * V_C_D_4cart(0) + V_C_D_4cart(2) * V_C_D_4cart(2)));
    if (V_C_D_4cart(2) > 0)
        Theta_5 = -Theta_5;
    double Theta_7 = 0;
    if (Theta_7 <= joint7_max && Theta_7 >= joint7_min &&
        Theta_6 <= joint6_max && Theta_6 >= joint6_min &&
        Theta_5 <= joint5_max && Theta_5 >= joint5_min &&
        Theta_4 <= joint4_max && Theta_4 >= joint4_min &&
        Theta_3 <= joint3_max && Theta_3 >= joint3_min &&
        Theta_2 <= joint2_max && Theta_2 >= joint2_min &&
        Theta_1 <= joint1_max && Theta_1 >= joint1_min)//&& fabs(Theta_3)<0.01 )
    {
        receive[0] = Theta_1;
        receive[1] = Theta_2;
        receive[2] = Theta_3;
        receive[3] = Theta_4;
        receive[4] = Theta_5;
        receive[5] = Theta_6;
        receive[6] = Theta_7;
    }
    if (receive[3] > joint4_max)
        receive[3] = joint4_max;
    if (receive[3] < joint4_min)
        receive[3] = joint4_min;
    return receive[3];
}

extern "C"
JNIEXPORT jdouble JNICALL
Java_com_example_myapplication3_MainActivity_IK_1joint5(JNIEnv *env, jobject thiz, jdouble x,
                                                        jdouble y, jdouble z) {
    double receive[7] = Initial_state;
    Matrix<double, 3, 3> R720;
    R720 << 1 - 2 * (qz * qz + qy * qy), 2 * (qx * qy - qw * qz), 2 * (qw * qy + qx * qz),
            2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
            2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy);
    double d_0_A = 0.333, d_A_B = 0.316, d_B_C = 0.384, d_D_f = 0.107, d_C = 0.088;
    Vector3d V_C_D_Fcart(-d_C, 0, -d_D_f);
    double x_C = R720.row(0).dot(V_C_D_Fcart) + x;
    double y_C = R720.row(1).dot(V_C_D_Fcart) + y;
    double z_C = R720.row(2).dot(V_C_D_Fcart) + z;
    double Se = 0;
    double d_A_C = sqrt(x_C * x_C + y_C * y_C + (z_C - 0.333) * (z_C - 0.333));
    double d_C_z0 = sqrt(x_C * x_C + y_C * y_C);
    double x_E = x_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double y_E = y_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double z_E = 0.333 + (z_C - 0.333) * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double t = d_A_B * sin(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C)));
    double x_B = (t * cos(Se)) * (-x_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (y_C / d_C_z0) + x_E;
    double y_B = (t * cos(Se)) * (-y_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (x_C / d_C_z0) + y_E;
    double z_B = (t * cos(Se)) * ((y_C * y_C + x_C * x_C) / (d_C_z0 * d_A_C)) + z_E;
    double Theta_1 = acos(x_B / sqrt(x_B * x_B + y_B * y_B));
    Theta_1 = acos(x_C / sqrt(x_C * x_C + y_C * y_C));
    if (y_C < 0)
        Theta_1 = -Theta_1;
    double Theta_4 = acos((d_A_B * d_A_B + d_B_C * d_B_C - d_A_C * d_A_C) / (2 * d_A_B * d_B_C));
    Theta_4 = Theta_4 - M_PI;
    double theta_temp = 0, d_temp = 0, aphla_temp = 0, a_temp = 0;
    theta_temp = Theta_1;
    aphla_temp = 0;
    Matrix<double, 3, 3> R120;
    R120 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Vector3d V_A_B(x_B - 0, y_B - 0, z_B - 0.333);
    Vector3d V_A_B_1cart = R120.transpose() * V_A_B;
    double Theta_2 = acos(V_A_B_1cart(2) / sqrt(V_A_B_1cart.dot(V_A_B_1cart)));
    if (V_A_B_1cart(0) < 0)
        Theta_2 = -Theta_2;
    Vector3d V_B_C(x_C - x_B, y_C - y_B, z_C - z_B);
    Vector3d V_D_original(0, 0, -d_D_f);
    double x_D_original = R720.row(0) * (V_D_original)+x;
    double y_D_original = R720.row(1) * (V_D_original)+y;
    double z_D_original = R720.row(2) * (V_D_original)+z;
    Vector3d V_D_D_original(x_D_original - x, y_D_original - y, z_D_original - z);
    double Theta_6 = acos(V_B_C.dot(V_D_D_original) / sqrt(V_B_C.dot(V_B_C) * V_D_D_original.dot(V_D_D_original)));
    Matrix<double, 3, 3> R221;
    theta_temp = Theta_2;
    aphla_temp = -M_PI / 2;
    R221 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R220;
    R220 = R120 * R221;
    Vector3d V_B_C_2cart = R220.transpose() * V_B_C; // Vector_BC in the second cart. May error
    double Theta_3 = acos(V_B_C_2cart(0) / sqrt(V_B_C_2cart(0) * V_B_C_2cart(0) + V_B_C_2cart(2) * V_B_C_2cart(2)));
    if (V_B_C_2cart(2) > 0) {
        Theta_3 = -Theta_3;
    }
    Matrix<double, 3, 3> R322;
    theta_temp = Theta_3;
    aphla_temp = M_PI / 2;
    R322 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R423;
    theta_temp = Theta_4;
    aphla_temp = M_PI / 2;
    R423 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R420;
    R420 = R220 * R322 * R423;
    Vector3d V_C_D(x - x_C, y - y_C, z - z_C);
    Vector3d V_C_D_4cart = R420.transpose() * V_C_D; // Vector_CD in the 4 cart. May error
    double Theta_5 = acos(V_C_D_4cart(0) / sqrt(V_C_D_4cart(0) * V_C_D_4cart(0) + V_C_D_4cart(2) * V_C_D_4cart(2)));
    if (V_C_D_4cart(2) > 0)
        Theta_5 = -Theta_5;
    double Theta_7 = 0;
    if (Theta_7 <= joint7_max && Theta_7 >= joint7_min &&
        Theta_6 <= joint6_max && Theta_6 >= joint6_min &&
        Theta_5 <= joint5_max && Theta_5 >= joint5_min &&
        Theta_4 <= joint4_max && Theta_4 >= joint4_min &&
        Theta_3 <= joint3_max && Theta_3 >= joint3_min &&
        Theta_2 <= joint2_max && Theta_2 >= joint2_min &&
        Theta_1 <= joint1_max && Theta_1 >= joint1_min)//&& fabs(Theta_3)<0.01 )
    {
        receive[0] = Theta_1;
        receive[1] = Theta_2;
        receive[2] = Theta_3;
        receive[3] = Theta_4;
        receive[4] = Theta_5;
        receive[5] = Theta_6;
        receive[6] = Theta_7;
    }
    if (receive[4] > joint5_max)
        receive[4] = joint5_max;
    if (receive[4] < joint5_min)
        receive[4] = joint5_min;
    return receive[4];
}

extern "C"
JNIEXPORT jdouble JNICALL
Java_com_example_myapplication3_MainActivity_IK_1joint6(JNIEnv *env, jobject thiz, jdouble x,
                                                        jdouble y, jdouble z) {
    double receive[7] = Initial_state;
    Matrix<double, 3, 3> R720;
    R720 << 1 - 2 * (qz * qz + qy * qy), 2 * (qx * qy - qw * qz), 2 * (qw * qy + qx * qz),
            2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
            2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy);
    double d_0_A = 0.333, d_A_B = 0.316, d_B_C = 0.384, d_D_f = 0.107, d_C = 0.088;
    Vector3d V_C_D_Fcart(-d_C, 0, -d_D_f);
    double x_C = R720.row(0).dot(V_C_D_Fcart) + x;
    double y_C = R720.row(1).dot(V_C_D_Fcart) + y;
    double z_C = R720.row(2).dot(V_C_D_Fcart) + z;
    double Se = 0;
    double d_A_C = sqrt(x_C * x_C + y_C * y_C + (z_C - 0.333) * (z_C - 0.333));
    double d_C_z0 = sqrt(x_C * x_C + y_C * y_C);
    double x_E = x_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double y_E = y_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double z_E = 0.333 + (z_C - 0.333) * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double t = d_A_B * sin(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C)));
    double x_B = (t * cos(Se)) * (-x_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (y_C / d_C_z0) + x_E;
    double y_B = (t * cos(Se)) * (-y_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (x_C / d_C_z0) + y_E;
    double z_B = (t * cos(Se)) * ((y_C * y_C + x_C * x_C) / (d_C_z0 * d_A_C)) + z_E;
    double Theta_1 = acos(x_B / sqrt(x_B * x_B + y_B * y_B));
    Theta_1 = acos(x_C / sqrt(x_C * x_C + y_C * y_C));
    if (y_C < 0)
        Theta_1 = -Theta_1;
    double Theta_4 = acos((d_A_B * d_A_B + d_B_C * d_B_C - d_A_C * d_A_C) / (2 * d_A_B * d_B_C));
    Theta_4 = Theta_4 - M_PI;
    double theta_temp = 0, d_temp = 0, aphla_temp = 0, a_temp = 0;
    theta_temp = Theta_1;
    aphla_temp = 0;
    Matrix<double, 3, 3> R120;
    R120 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Vector3d V_A_B(x_B - 0, y_B - 0, z_B - 0.333);
    Vector3d V_A_B_1cart = R120.transpose() * V_A_B;
    double Theta_2 = acos(V_A_B_1cart(2) / sqrt(V_A_B_1cart.dot(V_A_B_1cart)));
    if (V_A_B_1cart(0) < 0)
        Theta_2 = -Theta_2;
    Vector3d V_B_C(x_C - x_B, y_C - y_B, z_C - z_B);
    Vector3d V_D_original(0, 0, -d_D_f);
    double x_D_original = R720.row(0) * (V_D_original)+x;
    double y_D_original = R720.row(1) * (V_D_original)+y;
    double z_D_original = R720.row(2) * (V_D_original)+z;
    Vector3d V_D_D_original(x_D_original - x, y_D_original - y, z_D_original - z);
    double Theta_6 = acos(V_B_C.dot(V_D_D_original) / sqrt(V_B_C.dot(V_B_C) * V_D_D_original.dot(V_D_D_original)));
    Matrix<double, 3, 3> R221;
    theta_temp = Theta_2;
    aphla_temp = -M_PI / 2;
    R221 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R220;
    R220 = R120 * R221;
    Vector3d V_B_C_2cart = R220.transpose() * V_B_C; // Vector_BC in the second cart. May error
    double Theta_3 = acos(V_B_C_2cart(0) / sqrt(V_B_C_2cart(0) * V_B_C_2cart(0) + V_B_C_2cart(2) * V_B_C_2cart(2)));
    if (V_B_C_2cart(2) > 0) {
        Theta_3 = -Theta_3;
    }
    Matrix<double, 3, 3> R322;
    theta_temp = Theta_3;
    aphla_temp = M_PI / 2;
    R322 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R423;
    theta_temp = Theta_4;
    aphla_temp = M_PI / 2;
    R423 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R420;
    R420 = R220 * R322 * R423;
    Vector3d V_C_D(x - x_C, y - y_C, z - z_C);
    Vector3d V_C_D_4cart = R420.transpose() * V_C_D; // Vector_CD in the 4 cart. May error
    double Theta_5 = acos(V_C_D_4cart(0) / sqrt(V_C_D_4cart(0) * V_C_D_4cart(0) + V_C_D_4cart(2) * V_C_D_4cart(2)));
    if (V_C_D_4cart(2) > 0)
        Theta_5 = -Theta_5;
    double Theta_7 = 0;
    if (Theta_7 <= joint7_max && Theta_7 >= joint7_min &&
        Theta_6 <= joint6_max && Theta_6 >= joint6_min &&
        Theta_5 <= joint5_max && Theta_5 >= joint5_min &&
        Theta_4 <= joint4_max && Theta_4 >= joint4_min &&
        Theta_3 <= joint3_max && Theta_3 >= joint3_min &&
        Theta_2 <= joint2_max && Theta_2 >= joint2_min &&
        Theta_1 <= joint1_max && Theta_1 >= joint1_min)//&& fabs(Theta_3)<0.01 )
    {
        receive[0] = Theta_1;
        receive[1] = Theta_2;
        receive[2] = Theta_3;
        receive[3] = Theta_4;
        receive[4] = Theta_5;
        receive[5] = Theta_6;
        receive[6] = Theta_7;
    }
    if (receive[5] > joint6_max)
        receive[5] = joint6_max;
    if (receive[5] < joint6_min)
        receive[5] = joint6_min;
    return receive[5];
}

extern "C"
JNIEXPORT jdouble JNICALL
Java_com_example_myapplication3_MainActivity_IK_1joint7(JNIEnv *env, jobject thiz, jdouble x,
                                                        jdouble y, jdouble z) {
    double receive[7] = Initial_state;
    Matrix<double, 3, 3> R720;
    R720 << 1 - 2 * (qz * qz + qy * qy), 2 * (qx * qy - qw * qz), 2 * (qw * qy + qx * qz),
            2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
            2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy);
    double d_0_A = 0.333, d_A_B = 0.316, d_B_C = 0.384, d_D_f = 0.107, d_C = 0.088;
    Vector3d V_C_D_Fcart(-d_C, 0, -d_D_f);
    double x_C = R720.row(0).dot(V_C_D_Fcart) + x;
    double y_C = R720.row(1).dot(V_C_D_Fcart) + y;
    double z_C = R720.row(2).dot(V_C_D_Fcart) + z;
    double Se = 0;
    double d_A_C = sqrt(x_C * x_C + y_C * y_C + (z_C - 0.333) * (z_C - 0.333));
    double d_C_z0 = sqrt(x_C * x_C + y_C * y_C);
    double x_E = x_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double y_E = y_C * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double z_E = 0.333 + (z_C - 0.333) * d_A_B * cos(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C))) / d_A_C;
    double t = d_A_B * sin(acos((d_A_B * d_A_B + d_A_C * d_A_C - d_B_C * d_B_C) / (2 * d_A_B * d_A_C)));
    double x_B = (t * cos(Se)) * (-x_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (y_C / d_C_z0) + x_E;
    double y_B = (t * cos(Se)) * (-y_C * (z_C - 0.333) / (d_C_z0 * d_A_C)) + (t * sin(Se)) * (x_C / d_C_z0) + y_E;
    double z_B = (t * cos(Se)) * ((y_C * y_C + x_C * x_C) / (d_C_z0 * d_A_C)) + z_E;
    double Theta_1 = acos(x_B / sqrt(x_B * x_B + y_B * y_B));
    Theta_1 = acos(x_C / sqrt(x_C * x_C + y_C * y_C));
    if (y_C < 0)
        Theta_1 = -Theta_1;
    double Theta_4 = acos((d_A_B * d_A_B + d_B_C * d_B_C - d_A_C * d_A_C) / (2 * d_A_B * d_B_C));
    Theta_4 = Theta_4 - M_PI;
    double theta_temp = 0, d_temp = 0, aphla_temp = 0, a_temp = 0;
    theta_temp = Theta_1;
    aphla_temp = 0;
    Matrix<double, 3, 3> R120;
    R120 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Vector3d V_A_B(x_B - 0, y_B - 0, z_B - 0.333);
    Vector3d V_A_B_1cart = R120.transpose() * V_A_B;
    double Theta_2 = acos(V_A_B_1cart(2) / sqrt(V_A_B_1cart.dot(V_A_B_1cart)));
    if (V_A_B_1cart(0) < 0)
        Theta_2 = -Theta_2;
    Vector3d V_B_C(x_C - x_B, y_C - y_B, z_C - z_B);
    Vector3d V_D_original(0, 0, -d_D_f);
    double x_D_original = R720.row(0) * (V_D_original)+x;
    double y_D_original = R720.row(1) * (V_D_original)+y;
    double z_D_original = R720.row(2) * (V_D_original)+z;
    Vector3d V_D_D_original(x_D_original - x, y_D_original - y, z_D_original - z);
    double Theta_6 = acos(V_B_C.dot(V_D_D_original) / sqrt(V_B_C.dot(V_B_C) * V_D_D_original.dot(V_D_D_original)));
    Matrix<double, 3, 3> R221;
    theta_temp = Theta_2;
    aphla_temp = -M_PI / 2;
    R221 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R220;
    R220 = R120 * R221;
    Vector3d V_B_C_2cart = R220.transpose() * V_B_C; // Vector_BC in the second cart. May error
    double Theta_3 = acos(V_B_C_2cart(0) / sqrt(V_B_C_2cart(0) * V_B_C_2cart(0) + V_B_C_2cart(2) * V_B_C_2cart(2)));
    if (V_B_C_2cart(2) > 0) {
        Theta_3 = -Theta_3;
    }
    Matrix<double, 3, 3> R322;
    theta_temp = Theta_3;
    aphla_temp = M_PI / 2;
    R322 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R423;
    theta_temp = Theta_4;
    aphla_temp = M_PI / 2;
    R423 << cos(theta_temp), -sin(theta_temp), 0,
            sin(theta_temp)* cos(aphla_temp), cos(theta_temp)* cos(aphla_temp), -sin(aphla_temp),
            sin(theta_temp)* sin(aphla_temp), cos(theta_temp)* sin(aphla_temp), cos(aphla_temp);
    Matrix<double, 3, 3> R420;
    R420 = R220 * R322 * R423;
    Vector3d V_C_D(x - x_C, y - y_C, z - z_C);
    Vector3d V_C_D_4cart = R420.transpose() * V_C_D; // Vector_CD in the 4 cart. May error
    double Theta_5 = acos(V_C_D_4cart(0) / sqrt(V_C_D_4cart(0) * V_C_D_4cart(0) + V_C_D_4cart(2) * V_C_D_4cart(2)));
    if (V_C_D_4cart(2) > 0)
        Theta_5 = -Theta_5;
    double Theta_7 = 0;
    if (Theta_7 <= joint7_max && Theta_7 >= joint7_min &&
        Theta_6 <= joint6_max && Theta_6 >= joint6_min &&
        Theta_5 <= joint5_max && Theta_5 >= joint5_min &&
        Theta_4 <= joint4_max && Theta_4 >= joint4_min &&
        Theta_3 <= joint3_max && Theta_3 >= joint3_min &&
        Theta_2 <= joint2_max && Theta_2 >= joint2_min &&
        Theta_1 <= joint1_max && Theta_1 >= joint1_min)//&& fabs(Theta_3)<0.01 )
    {
        receive[0] = Theta_1;
        receive[1] = Theta_2;
        receive[2] = Theta_3;
        receive[3] = Theta_4;
        receive[4] = Theta_5;
        receive[5] = Theta_6;
        receive[6] = Theta_7;
    }
    if (receive[6] > joint7_max)
        receive[6] = joint7_max;
    if (receive[6] < joint7_min)
        receive[6] = joint7_min;
    return receive[6];
}