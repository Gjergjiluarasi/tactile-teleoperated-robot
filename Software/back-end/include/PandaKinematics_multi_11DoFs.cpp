#include "PandaKinematics_multi_11DoFs.hh"

// void jointsVelocityLimit(Vector8d &jointsVelocityValue, Vector8d jointsPositionValue) {
//     double limitedJointVelocityValue = 0, minJointVelocityValue = 0, maxJointVelocityValue = 0;

//     for (size_t i = 0; i < 7; i++) {
//         switch (i + 1) {
//             case 1:
//                 maxJointVelocityValue =
//                     std::min(Q_MAX_1_2_3_4,
//                              std::max(0.0, Q_MAX_OFFSET_1_4 +
//                                                std::sqrt(std::max(0.0, 12.0 * (2.7437 - jointsPositionValue(i))))));
//                 minJointVelocityValue =
//                     std::max(Q_MIN_1_2_3_4,
//                              std::min(0.0, Q_MIN_OFFSET_1_4 -
//                                                std::sqrt(std::max(0.0, 12.0 * (2.7437 + jointsPositionValue(i))))));
//                 jointsVelocityValue(i) =
//                     std::max(minJointVelocityValue, std::min(maxJointVelocityValue, jointsVelocityValue(i)));
//                 break;

//             case 2:
//                 maxJointVelocityValue =
//                     std::min(Q_MAX_1_2_3_4,
//                              std::max(0.0, Q_MAX_OFFSET_2_3 +
//                                                std::sqrt(std::max(0.0, 5.17 * (1.7837 - jointsPositionValue(i))))));
//                 minJointVelocityValue =
//                     std::max(Q_MIN_1_2_3_4,
//                              std::min(0.0, Q_MIN_OFFSET_2_3 -
//                                                std::sqrt(std::max(0.0, 5.17 * (1.7837 + jointsPositionValue(i))))));
//                 jointsVelocityValue(i) =
//                     std::max(minJointVelocityValue, std::min(maxJointVelocityValue, jointsVelocityValue(i)));
//                 break;

//             case 3:
//                 maxJointVelocityValue =
//                     std::min(Q_MAX_1_2_3_4,
//                              std::max(0.0, Q_MAX_OFFSET_2_3 +
//                                                std::sqrt(std::max(0.0, 7.0 * (2.9007 - jointsPositionValue(i))))));
//                 minJointVelocityValue =
//                     std::max(Q_MIN_1_2_3_4,
//                              std::min(0.0, Q_MIN_OFFSET_2_3 -
//                                                std::sqrt(std::max(0.0, 7.0 * (2.9007 + jointsPositionValue(i))))));
//                 jointsVelocityValue(i) =
//                     std::max(minJointVelocityValue, std::min(maxJointVelocityValue, jointsVelocityValue(i)));
//                 break;

//             case 4:
//                 maxJointVelocityValue =
//                     std::min(Q_MAX_1_2_3_4,
//                              std::max(0.0, Q_MAX_OFFSET_1_4 +
//                                                std::sqrt(std::max(0.0, 8.0 * (-0.1518 - jointsPositionValue(i))))));
//                 minJointVelocityValue =
//                     std::max(Q_MIN_1_2_3_4,
//                              std::min(0.0, Q_MIN_OFFSET_1_4 -
//                                                std::sqrt(std::max(0.0, 8.0 * (-3.0421 + jointsPositionValue(i))))));
//                 // jointsVelocityValue(i) = std::max(minJointVelocityValue, std::min(maxJointVelocityValue,
//                 // jointsVelocityValue(i)));
//                 break;

//             case 5:
//                 maxJointVelocityValue = std::min(
//                     Q_MAX_5_7, std::max(0.0, Q_MAX_OFFSET_5_6_7 +
//                                                  std::sqrt(std::max(0.0, 34.0 * (2.8065 - jointsPositionValue(i))))));
//                 minJointVelocityValue = std::max(
//                     Q_MIN_5_7, std::min(0.0, Q_MIN_OFFSET_5_6_7 -
//                                                  std::sqrt(std::max(0.0, 34.0 * (2.8065 + jointsPositionValue(i))))));
//                 break;

//             case 6:
//                 maxJointVelocityValue = std::min(
//                     Q_MAX_6, std::max(0.0, Q_MAX_OFFSET_5_6_7 +
//                                                std::sqrt(std::max(0.0, 11.0 * (4.5169 - jointsPositionValue(i))))));
//                 minJointVelocityValue = std::max(
//                     Q_MIN_6, std::min(0.0, Q_MIN_OFFSET_5_6_7 -
//                                                std::sqrt(std::max(0.0, 11.0 * (-0.5445 + jointsPositionValue(i))))));
//                 break;

//             case 7:
//                 maxJointVelocityValue = std::min(
//                     Q_MAX_5_7, std::max(0.0, Q_MAX_OFFSET_5_6_7 +
//                                                  std::sqrt(std::max(0.0, 34.0 * (3.0159 - jointsPositionValue(i))))));
//                 minJointVelocityValue = std::max(
//                     Q_MIN_5_7, std::min(0.0, Q_MIN_OFFSET_5_6_7 -
//                                                  std::sqrt(std::max(0.0, 34.0 * (3.0159 + jointsPositionValue(i))))));
//                 jointsVelocityValue(i) =
//                     std::max(minJointVelocityValue, std::min(maxJointVelocityValue, jointsVelocityValue(i)));
//                 break;

//             default:
//                 break;
//         }
//         // Update limited joint velocities
//     }
// }

void rcmFK(std::array<double, 7> q, double eta, Matrix4d &XX, Matrix4d &XXp, Matrix<double, 6, 11> &JJe6, Matrix<double, 3, 11> &JJp3,std::array<double, 3> q_maxon  ) // Vector7d q xx
{
     Matrix4d Ab0, A01, A12, A23, A34, A45, A56, A67, A7f, Afr, Arp, Ape, Afr1, Afr2, Afr3, Ape2, Ape3, Ape4,Ape5, Apef;
    Matrix4d Ab1, Ab2, Ab3, Ab4, Ab5, Ab6, Ab7, Abf, Abp, Abe, Abe2, Abe3, Abef;
    Matrix3d R1, R2, R3, R4, R5, R6, R7;
    Matrix<double, 6, 11> Je, Jp;     
    Matrix<double, 3, 8> Je3, Jp_new; 
    Matrix<double, 5, 5> eye5;
    Matrix<double, 8, 8> eye8;
    eye5.setIdentity();
    eye8.setIdentity();

    double C1, S1, C2, S2, C3, S3, C4, S4, C5, S5, C6, S6, C7, S7, C8, S8, C9, S9, C10, S10;
    double d1{0.333}, d3{0.316}, d5{0.384}, df{0.107};
    double a3{0.0825}, a4{-0.0825}, a6{0.088}, ag{0.0105}, ag2{0.0270};
    // double l(0.173); // tool length
    Vector3d z0, z1, z2, z3, z4, z5, z6, z7, zp, zer, z8, z9, z10;
    Vector3d p0, p1, p2, p3, p4, p5, p6, p7, pp, pe, pe2, pe3, pef;
    zer.setZero(); // zero vector


    C1 = cos(q[0]);
    S1 = sin(q[0]);
    C2 = cos(q[1]);
    S2 = sin(q[1]);
    C3 = cos(q[2]);
    S3 = sin(q[2]);
    C4 = cos(q[3]);
    S4 = sin(q[3]);
    C5 = cos(q[4]);
    S5 = sin(q[4]);
    C6 = cos(q[5]);
    S6 = sin(q[5]);
    C7 = cos(q[6]);
    S7 = sin(q[6]);
    //Maxon Motor
    S8 = sin(q_maxon[1]);
    C8 = cos(q_maxon[1]);
    S9 = sin(q_maxon[2]+M_PI/2.0);
    C9 = cos(q_maxon[2]+M_PI / 2.0);
    S10 = sin(q_maxon[3]);
    S10 = cos(q_maxon[3]);


    Ab0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    A01 << C1, -S1, 0, 0,
        S1, C1, 0, 0,
        0, 0, 1, d1,
        0, 0, 0, 1;

    A12 << C2, -S2, 0, 0,
        0, 0, 1, 0,
        -S2, -C2, 0, 0,
        0, 0, 0, 1;

    A23 << C3, -S3, 0, 0,
        0, 0, -1, -d3,
        S3, C3, 0, 0,
        0, 0, 0, 1;

    A34 << C4, -S4, 0, a3,
        0, 0, -1, 0,
        S4, C4, 0, 0,
        0, 0, 0, 1;

    A45 << C5, -S5, 0, a4,
        0, 0, 1, d5,
        -S5, -C5, 0, 0,
        0, 0, 0, 1;

    A56 << C6, -S6, 0, 0,
        0, 0, -1, 0,
        S6, C6, 0, 0,
        0, 0, 0, 1;

    A67 << C7, -S7, 0, a6,
        0, 0, -1, 0,
        S7, C7, 0, 0,
        0, 0, 0, 1;

    A7f << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, df,
        0, 0, 0, 1; // d7+l

    // Afr << cos(alpha_endo), 0, sin(alpha_endo), dx_endo,
    //     0, 1, 0, dy_endo,
    //     -sin(alpha_endo), 0, cos(alpha_endo), dz_endo,
    //     0, 0, 0, 1;

    Afr1 << cos(beta_endo), -sin(beta_endo), 0, 0,
        sin(beta_endo), cos(beta_endo), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Afr2 << 1, 0, 0, dx_endo,
        0, 1, 0, 0,
        0, 0, 0, dz_endo,
        0, 0, 0, 1;

    Afr3 << 0, -1, 0, 0,
        0, 0, -1, dy_endo,
        1, 0, 0, 0,
        0, 0, 0, 1;

    Afr = Afr1 * Afr2 * Afr3;
    Arp << 1, 0, 0, 0,
        0, 0, -1, -eta,
        0, 1, 0, 0,
        0, 0, 0, 1;

    Ape << C8, -S8,  0, 0,
           S8,  C8,  0, 0,
            0,   0,  1, tool_length - eta,
            0,   0,  0, 1;

    Ape2 << C9, -S9,  0,  0,
	     0,   0,  -1, 0,
            S9,   C9,  0, 0,
             0,    0,  0, 1;

    Ape3 <<  C10, -S10,  0, ag,
               0,    0,  1,  0,
            -S10, -C10,  0,  0,
               0,    0,  0,  1;

    
   Ape4 << cos(alpha_axis), 0, sin(alpha_axis), 0,
                         0, 1,               0, 0,
          -sin(alpha_axis), 0, cos(alpha_axis), 0,
                         0, 0,               0, 1;

    // Ape5 << 1, 0, 0, 0,
    //         0, cos(alpha_axis), -sin(alpha_axis) ,0,
    //         0, sin(alpha_axis), cos(alpha_axis) ,0,
    //         0, 0,               0, 1;

    Ape5.setIdentity();

   Apef <<      1, 0,  0,   0,
                0, 1,  0,   0,
                0, 0,  1, ag2,
                0, 0,  0,   1;

// Ape4.setIdentity();
// Apef.setIdentity();


    // jacobian

    Ab1 = Ab0 * A01;
    Ab2 = Ab1 * A12;
    Ab3 = Ab2 * A23;
    Ab4 = Ab3 * A34;
    Ab5 = Ab4 * A45;
    Ab6 = Ab5 * A56;
    Ab7 = Ab6 * A67;
    Abp = Ab7 * A7f * Afr * Arp;
    Abe = Abp * Ape;
    Abe2= Abe * Ape2;
    Abe3= Abe2 * Ape3;
    Abef= Abe3 * Ape4 * Ape5*Apef;

    z0 = Ab0.block<3, 1>(0, 2);
    z1 = Ab1.block<3, 1>(0, 2);
    z2 = Ab2.block<3, 1>(0, 2);
    z3 = Ab3.block<3, 1>(0, 2);
    z4 = Ab4.block<3, 1>(0, 2);
    z5 = Ab5.block<3, 1>(0, 2);
    z6 = Ab6.block<3, 1>(0, 2);
    z7 = Ab7.block<3, 1>(0, 2);
    zp = Abp.block<3, 1>(0, 2);
    z8 = Abe.block<3, 1>(0, 2);
    z9 = Abe2.block<3, 1>(0, 2); 
    z10= Abe3.block<3, 1>(0, 2); 

    p0 = Ab0.block<3, 1>(0, 3);
    p1 = Ab1.block<3, 1>(0, 3);
    p2 = Ab2.block<3, 1>(0, 3);
    p3 = Ab3.block<3, 1>(0, 3);
    p4 = Ab4.block<3, 1>(0, 3);
    p5 = Ab5.block<3, 1>(0, 3);
    p6 = Ab6.block<3, 1>(0, 3);
    p7 = Ab7.block<3, 1>(0, 3);
    pp = Abp.block<3, 1>(0, 3);
    pe = Abe.block<3, 1>(0, 3);
    pe2= Abe2.block<3, 1>(0, 3);
    pe3= Abe3.block<3, 1>(0, 3);
    pef= Abef.block<3, 1>(0, 3);

    Je << z1.cross(pef - p1), z2.cross(pef - p2), z3.cross(pef - p3), z4.cross(pef - p4), z5.cross(pef - p5), z6.cross(pef - p6), z7.cross(pef - p7), zer, z8.cross(pef - pe), z9.cross(pef - pe2), z10.cross(pef - pe3), 
        z1, z2, z3, z4, z5, z6, z7, zer, z8, z9, z10;

        // Je << z1.cross(pef - p1), z2.cross(pef - p2), z3.cross(pef - p3), z4.cross(pef - p4), z5.cross(pef - p5), z6.cross(pef - p6), z7.cross(pef - p7), zer, zer, zer, zer, 
        // z1, z2, z3, z4, z5, z6, z7, zer, zer, zer, zer;

    Jp << z1.cross(pp - p1), z2.cross(pp - p2), z3.cross(pp - p3), z4.cross(pp - p4), z5.cross(pp - p5), z6.cross(pp - p6), z7.cross(pp - p7), zp, zer, zer, zer,
        z1, z2, z3, z4, z5, z6, z7, zer, zer, zer, zer;


    JJe6 = Je;
    XX = Abef;
    XXp = Abp;
    JJp3 = Jp.block<3, 11>(0, 0);
}