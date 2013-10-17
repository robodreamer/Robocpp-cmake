/*
 * Huboplus.cpp
 * This file is part of <Robocpp>
 *
 * Copyright (C) 2013 - Andy Park <andypark@purdue.edu>
 *
 * <Robocpp> is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * <Robocpp> is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with <Robocpp>. If not, see <http://www.gnu.org/licenses/>.
 */

/*
Author: Andy Park (andypark@purdue.edu)
Date: August, 2013
*/

#include "Robocpp.h"


using namespace std;
using namespace arma;

Huboplus::Huboplus(void){
    initialize();
}


void Huboplus::initialize(void){
    NB = 26;
    NJ = NB - 1;
    NF = 4;
    NLimb = 4;
    grav << 0 << 0 << 9.81 <<endr;

    //        # Contact State
    BODY = 0;
    LF = 2;
    RF = 3;

    //        # Joint Index
    J_HPY = 1;

    J_LHY = 2;
    J_LHR = 3;
    J_LHP = 4;
    J_LKP = 5;
    J_LAP = 6;
    J_LAR = 7;
    J_LF = J_LAR;

    J_RHY = 8;
    J_RHR = 9;
    J_RHP = 10;
    J_RKP = 11;
    J_RAP = 12;
    J_RAR = 13;
    J_RF = J_RAR;

    J_LSP = 14;
    J_LSR = 15;
    J_LSY = 16;
    J_LEP = 17;
    J_LWY = 18;
    J_LWP = 19;
    J_LH = J_LWP;

    J_RSP = 20;
    J_RSR = 21;
    J_RSY = 22;
    J_REP = 23;
    J_RWY = 24;
    J_RWP = 25;
    J_RH = J_RWP;

    Frame_LF = 101;
    Frame_RF = 102;
    Frame_LH = 103;
    Frame_RH = 104;

    //        # Body Index
    B_Torso = 0;
    B_Hip = 1;

    B_LHY = 2;
    B_LHR = 3;
    B_LHP = 4;
    B_LKP = 5;
    B_LAP = 6;
    B_LAR = 7;

    B_RHY = 8;
    B_RHR = 9;
    B_RHP = 10;
    B_RKP = 11;
    B_RAP = 12;
    B_RAR = 13;

    B_LSP = 14;
    B_LSR = 15;
    B_LSY = 16;
    B_LEP = 17;
    B_LWY = 18;
    B_LWP = 19;

    B_RSP = 20;
    B_RSR = 21;
    B_RSY = 22;
    B_REP = 23;
    B_RWY = 24;
    B_RWP = 25;

    //        # Frame Index
    F_HPY = 1;

    F_LHY = 3;
    F_LHR = 4;
    F_LHP = 5;
    F_LKP = 6;
    F_LAP = 7;
    F_LAR = 8;
    F_LF = F_LAR;

    F_RHY = 10;
    F_RHR = 11;
    F_RHP = 12;
    F_RKP = 13;
    F_RAP = 14;
    F_RAR = 15;
    F_RF = F_RAR;

    F_LSP = 17;
    F_LSR = 18;
    F_LSY = 19;
    F_LEP = 20;
    F_LWY = 21;
    F_LWP = 22;
    F_LH = F_LWP;

    F_RSP = 24;
    F_RSR = 25;
    F_RSY = 26;
    F_REP = 27;
    F_RWY = 28;
    F_RWP = 29;
    F_RH = F_RWP;

    // Bodies
    body_com = zeros(NB,3);
    body_mass = zeros<vec>(NB,1);
    body_I = zeros(NB,6);
    body_parent = zeros<vec>(NB,1);
    body_no_child_frames = zeros<vec>(NB,1);
    body_child_frame = -100*ones(NB,3);

    mat tmp;
    int K = 0;

    //======Bodies======//
    // Body 1: Body Torso
    K = K + 1;
    tmp << -0.012258116293 << -0.002473149928 << -0.048635606065 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 7.503338799400;
    tmp << 0.102019217897 << 0.083272393713 << 0.086493014809 << 0.000024546177 << -0.001317325511 << 0.007130222433 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = -1;
    body_no_child_frames(K-1) = 3;
    tmp << J_HPY << J_LSP << J_RSP <<endr;
    body_child_frame.row(K-1) = tmp;

    // Body 2: Body Hip
    K = K + 1;
    tmp << -0.013850358943 << 0.000007806523 << -0.035461283862 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 3.417193968804;
    tmp <<  0.018034853826 << 0.009123817212 << 0.022051361197 << 0.000031753060 <<  -0.000069061669 << -0.000513489041 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_Torso;
    body_no_child_frames(K-1) = 2;
    tmp << J_LHY << J_RHY <<endr;
    body_child_frame(K-1,span(0,1)) = tmp;

    // Body 3: Body LHY
    K = K + 1;
    tmp << 0.034706500276 << 0.000037480007 << -0.072615273181 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 0.826125012988;
    tmp << 0.001518175906 << 0.002863333718 << 0.002114373967 << 0.000002135861 << -0.000000157525 << 0.000434128327 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_Hip;
    body_no_child_frames(K-1) = 1;
    //tmp << J_LHR <<endr;
    body_child_frame(K-1,0) = J_LHR;

    // Body 4: Body LHR
    K = K + 1;
    tmp << -0.049747912596 << 0.012531116599 << -0.015643664572 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 1.932656684780;
    tmp << 0.004034872008 << 0.003327066518 << 0.003171733387 << 0.000079452031 << -0.000551964047 << -0.000125342881 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_LHY;
    body_no_child_frames(K-1) = 1;
    //tmp << J_LHP <<endr;
    body_child_frame(K-1,0) = J_LHP;

    // Body 5: Body LHP
    K = K + 1;
    tmp << 0.019504891350 << -0.059577480789 << -0.175201757627 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 2.820095294731;
    tmp << 0.029510226255 << 0.027377052037 << 0.008380352842 << 0.000184398776 << 0.000656580164 << -0.000370291227 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_LHR;
    body_no_child_frames(K-1) = 1;
    //tmp << J_LKP <<endr;
    body_child_frame(K-1,0) = J_LKP;

    // Body 6: Body LKP
    K = K + 1;
    tmp << 0.012825433376 << -0.007275670525 << -0.171431348038 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 1.809116607048;
    tmp << 0.023215644043 << 0.020834230825 << 0.005920398408 << 0.000251648144 << 0.002780676346 << -0.001293430587 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_LHP;
    body_no_child_frames(K-1) = 1;
    //tmp << J_LAP <<endr;
    body_child_frame(K-1,0) = J_LAP;

    // Body 7: Body LAP
    K = K + 1;
    tmp << 0.019870246084 << -0.045969255056 << 0.011506941050 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 1.635010626057;
    tmp << 0.002316586138 << 0.003304106079 << 0.002794946081 <<  0.000018740179 << 0.000063815303 << 0.000369899000 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_LKP;
    body_no_child_frames(K-1) = 1;
    //    tmp << J_LAR <<endr;
    body_child_frame(K-1,0) = J_LAR;

    // Body 8: Body LAR
    K = K + 1;
    tmp << -0.051509407797 << 0.002163982128 << -0.069388069491 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 1.20318;
    tmp << 0.002951826559 << 0.005247924231 << 0.005168173927 << 0.000032321137 << 0.000059540355 << 0.000141769152 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_LAP;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_LF <<endr;
    body_child_frame(K-1,0) = Frame_LF;

    // Body 9: Body RHY
    K = K + 1;
    tmp << 0.034706500276 << -0.000037480007 << -0.072615273181 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 0.826125012988;
    tmp << 0.001518175906 << 0.002863333718 << 0.002114373967 << -0.000002135861 << 0.000000157525 << 0.000434128327 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_Hip;
    body_no_child_frames(K-1) = 1;
    //tmp << J_RHR <<endr;
    body_child_frame(K-1,0) = J_RHR;

    // Body 10: Body RHR
    K = K + 1;
    tmp << -0.049747912596 << -0.012531116599 << -0.015643664572 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 1.932656684780;
    tmp << 0.004034872008 << 0.003327066518 << 0.003171733387 << -0.000079452031 << 0.000551964047 << -0.000125342881 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_RHY;
    body_no_child_frames(K-1) = 1;
    //tmp << J_RHP <<endr;
    body_child_frame(K-1,0) = J_RHP;

    // Body 11: Body RHP
    K = K + 1;
    tmp << 0.019504891350 << 0.059577480789 << -0.175201757627 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 2.820095294731;
    tmp << 0.029510226255 << 0.027377052037 << 0.008380352842 << -0.000184398776 << -0.000656580164 << -0.000370291227 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_RHR;
    body_no_child_frames(K-1) = 1;
    //tmp << J_RKP <<endr;
    body_child_frame(K-1,0) = J_RKP;

    // Body 12: Body RKP
    K = K + 1;
    tmp << 0.012825433376 << 0.007275670525 << -0.171431348038 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 1.809116607048;
    tmp << 0.023215644043 << 0.020834230825 << 0.005920398408 << -0.000251648144 << -0.002780676346 << -0.001293430587 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_RHP;
    body_no_child_frames(K-1) = 1;
    //tmp << J_RAP <<endr;
    body_child_frame(K-1,0) = J_RAP;

    // Body 13: Body RAP
    K = K + 1;
    tmp << 0.019870246084 << 0.045969255056 << 0.011506941050 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 1.635010626057;
    tmp << 0.002316586138 << 0.003304106079 << 0.002794946081 << -0.000018740179 << -0.000063815303 << 0.000369899000 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_RKP;
    body_no_child_frames(K-1) = 1;
    //tmp << J_RAR <<endr;
    body_child_frame(K-1,0) = J_RAR;

    // Body 14: Body RAR
    K = K + 1;
    tmp << -0.051509407797 << -0.002163982128 << -0.069388069491 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 1.20318;
    tmp << 0.002951826559 << 0.005247924231 << 0.005168173927 << -0.000032321137 << -0.000059540355 << 0.000141769152 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_RAP;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_RF <<endr;
    body_child_frame(K-1,0) = Frame_RF;

    // Body 15: Body LSP
    K = K + 1;
    tmp << 0.016720721524 << 0.057040679167 << 0.000009918731 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 0.646654803406;
    tmp << 0.000893730666 << 0.000930159822 << 0.001243225119 << -0.000156483989 << 0.000000157095 << 0.000000498007 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_Torso;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_RF <<endr;
    body_child_frame(K-1,0) = J_LSR;

    // Body 16: Body LSR
    K = K + 1;
    tmp << -0.036207973807 << -0.000083124345 << -0.001266848323 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 0.413367772877;
    tmp << 0.000269283340 << 0.000376789510 << 0.000360354691 << 0.000000452571 << 0.000000993935 << 0.000003236618 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_LSP;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_RF <<endr;
    body_child_frame(K-1,0) = J_LSY;

    // Body 17: Body LSY
    K = K + 1;
    tmp << 0.004561211264 << 0.000031057603 << -0.090782377537 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 1.154608561577;
    tmp << 0.003654788538 << 0.003647313707 << 0.000606330376 << 0.000002054362 << -0.000030964786 << 0.000418448220 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_LSR;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_RF <<endr;
    body_child_frame(K-1,0) = J_LEP;

    // Body 18: Body LEP
    K = K + 1;
    tmp << -0.021734910228 << 0.017831394355 << -0.061087268246 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 0.440940029986;
    tmp << 0.000502700471 << 0.000494150112 << 0.000131786442 << 0.000003645865 << 0.000007655278 << -0.000014138784 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_LSY;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_RF <<endr;
    body_child_frame(K-1,0) = J_LWY;

    // Body 19: Body LWY
    K = K + 1;
    tmp << -0.001220192079 << 0.000635233335 << -0.023804318643 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 0.537738149550;
    tmp << 0.000634569731 << 0.000563917510 << 0.000305422235 << -0.000000462486 << 0.000035404433 << 0.000006206816 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_LEP;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_RF <<endr;
    body_child_frame(K-1,0) = J_LWP;

    // Body 20: Body LWP
    K = K + 1;
    tmp << -0.000258199377 << 0.011441986656 << -0.046147242821 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 0.164392998329;
    tmp << 0.000097432742 << 0.000144494281 << 0.000122681345 << -0.000000522155 << 0.000009492703 << -0.000001173170 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_LWY;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_RF <<endr;
    body_child_frame(K-1,0) = Frame_LH;

    // Body 21: Body RSP
    K = K + 1;
    tmp << 0.016720721524 << -0.057040679167 << 0.000009918731 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 0.646654803406;
    tmp << 0.000893730666 << 0.000930159822 << 0.001243225119 << 0.000156483989 << -0.000000157095 << 0.000000498007 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_Torso;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_RF <<endr;
    body_child_frame(K-1,0) = J_RSR;

    // Body 22: Body RSR
    K = K + 1;
    tmp << -0.036207973807 << 0.000083124345 << -0.001266848323 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 0.413367772877;
    tmp << 0.000269283340 << 0.000376789510 << 0.000360354691 << -0.000000452571 << -0.000000993935 << 0.000003236618 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_RSP;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_RF <<endr;
    body_child_frame(K-1,0) = J_RSY;

    // Body 23: Body RSY
    K = K + 1;
    tmp << 0.004561211264 << -0.000031057603 << -0.090782377537 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 1.154608561577;
    tmp << 0.003654788538 << 0.003647313707 << 0.000606330376 << -0.000002054362 << 0.000030964786 << 0.000418448220 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_RSR;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_RF <<endr;
    body_child_frame(K-1,0) = J_REP;

    // Body 24: Body REP
    K = K + 1;
    tmp << -0.021734910228 << -0.017831394355 << -0.061087268246 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 0.440940029986;
    tmp << 0.000502700471 << 0.000494150112 << 0.000131786442 << -0.000003645865 << -0.000007655278 << -0.000014138784 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_RSY;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_RF <<endr;
    body_child_frame(K-1,0) = J_RWY;

    // Body 25: Body RWY
    K = K + 1;
    tmp << -0.001220192079 << -0.000635233335 << -0.023804318643 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 0.537738149550;
    tmp << 0.000634569731 << 0.000563917510 << 0.000305422235 << 0.000000462486 << -0.000035404433 << 0.000006206816 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_REP;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_RF <<endr;
    body_child_frame(K-1,0) = J_RWP;

    // Body 27: Body RWP
    K = K + 1;
    tmp << -0.000258199377 << -0.011441986656 << -0.046147242821 <<endr;
    body_com.row(K-1) = tmp;
    body_mass(K-1) = 0.164392998329;
    tmp << 0.000097432742 << 0.000144494281 << 0.000122681345 << 0.000000522155 << -0.000009492703 << -0.000009492703 <<endr;
    body_I.row(K-1) = tmp;
    body_parent(K-1) = B_RWY;
    body_no_child_frames(K-1) = 1;
    //tmp << Frame_RF <<endr;
    body_child_frame(K-1,0) = Frame_RH;

    // Joints
    joint_axis = zeros(NJ,3);
    joint_translation = zeros(NJ,3);
    joint_rotation = zeros(NJ,3);
    joint_q_min = zeros<vec>(NJ,1);
    joint_q_max = zeros<vec>(NJ,1);

    K = 0;
    //======Joints======//
    // Joint 1: HPY
    K = K + 1;
    tmp << 0 << 0 << 1 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.0122662 << 0.00245974 << -0.15262 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -3.14158;
    joint_q_max(K-1) = 3.14158;

    // Joint 2: LHY
    K = K + 1;
    tmp << 0 << 0 << 1 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.000125136 << 0.0885155 << -0.0764689 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -1.7453;
    joint_q_max(K-1) = 1.7453;

    // Joint 3: LHR
    K = K + 1;
    tmp << 1 << 0 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.0520001 << 1.50498e-06 << -0.091004 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -0.785; //-0.174;
    joint_q_max(K-1) = 0.785;

    // Joint 4: LHP
    K = K + 1;
    tmp << 0 << 1 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << -0.0529074 << 0.0655748 << 9.99159e-05 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -1.5708;
    joint_q_max(K-1) = 1.5708;

    // Joint 5: LKP
    K = K + 1;
    tmp << 0 << 1 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.000766364 << -0.0445011 << -0.280007 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -0.174;
    joint_q_max(K-1) = 2.61;

    // Joint 6: LAP
    K = K + 1;
    tmp << 0 << 1 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 7.20248e-06 << 0.0247555 << -0.279942 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -1.5708;
    joint_q_max(K-1) = 1.5708;

    // Joint 7: LAR
    K = K + 1;
    tmp << 1 << 0 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.0 << -0.0466006 << -1.04e-10 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -1.5708;
    joint_q_max(K-1) = 1.5708;

    // Joint 8: RHY
    K = K + 1;
    tmp << 0 << 0 << 1 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.000125136 << -0.0884999 << -0.0764689 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -1.7453;
    joint_q_max(K-1) = 1.7453;

    // Joint 9: RHR
    K = K + 1;
    tmp << 1 << 0 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.0520001 << 7.3455e-05 << -0.091004 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -0.785;
    joint_q_max(K-1) = 0.785;// 0.174;

    // Joint 10: RHP
    K = K + 1;
    tmp << 0 << 1 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << -0.0529074 << -0.0655748 << 9.99159e-05 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -1.5708;
    joint_q_max(K-1) = 1.5708;

    // Joint 11: RKP
    K = K + 1;
    tmp << 0 << 1 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.000766364 << 0.0445011 << -0.280007 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -0.174;
    joint_q_max(K-1) = 2.61;

    // Joint 12: RAP
    K = K + 1;
    tmp << 0 << 1 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 7.20248e-06 << -0.0247555 << -0.279942 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -1.5708;
    joint_q_max(K-1) = 1.5708;

    // Joint 13: RAR
    K = K + 1;
    tmp << 1 << 0 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.0 << 0.0466006 << -1.04e-10 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -1.5708;
    joint_q_max(K-1) = 1.5708;

    // Joint 14: LSP
    K = K + 1;
    tmp << 0 << 1 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.0122581 << 0.143973 << 0.0486356 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -3.1415;
    joint_q_max(K-1) = 3.1415;

    // Joint 15: LSR
    K = K + 1;
    tmp << 1 << 0 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.0269 << 0.072 << 0 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -0.26;
    joint_q_max(K-1) = 3.1415;

    // Joint 16: LSY
    K = K + 1;
    tmp << 0 << 0 << 1 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << -0.0269 << -0.000166249 << -0.0245 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -3.1415;
    joint_q_max(K-1) = 3.1415;

    // Joint 17: LEP
    K = K + 1;
    tmp << 0 << 1 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.0219936 << -0.018 << -0.157505 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -2.96;
    joint_q_max(K-1) = 0;

    // Joint 18: LWY
    K = K + 1;
    tmp << 0 << 0 << 1 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << -0.0219906 << 0.018 << -0.111408 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -3.1415;
    joint_q_max(K-1) = 3.1415;

    // Joint 19: LWP
    K = K + 1;
    tmp << 0 << 1 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 1.72973e-06 << -0.00872953 << -0.05255 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -3.1415;
    joint_q_max(K-1) = 3.1415;

    // Joint 20: RSP
    K = K + 1;
    tmp << 0 << 1 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.0122581 << -0.139027 << 0.0486356 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -3.1415;
    joint_q_max(K-1) = 3.1415;

    // Joint 21: RSR
    K = K + 1;
    tmp << 1 << 0 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.0269 << -0.072 << 0 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -3.1415;
    joint_q_max(K-1) = 0.26;

    // Joint 22: RSY
    K = K + 1;
    tmp << 0 << 0 << 1 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << -0.0269 << 0 << -0.0245 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -3.1415;
    joint_q_max(K-1) = 3.1415;

    // Joint 23: REP
    K = K + 1;
    tmp << 0 << 1 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 0.0219936 << 0.018 << -0.157505 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -2.96;
    joint_q_max(K-1) = 0;

    // Joint 25: RWY
    K = K + 1;
    tmp << 0 << 0 << 1 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << -0.0219906 << -0.018 << -0.111408 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -3.1415;
    joint_q_max(K-1) = 3.1415;

    // Joint 26: RWP
    K = K + 1;
    tmp << 0 << 1 << 0 <<endr;
    joint_axis.row(K-1) = tmp;
    tmp << 1.72973e-06 << 0.01 << -0.05255 <<endr;
    joint_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    joint_rotation.row(K-1) = tmp;
    joint_q_min(K-1) = -3.1415;
    joint_q_max(K-1) = 3.1415;

    // Frames
    frame_axis = zeros<vec>(NF + 100, 1);
    frame_translation = zeros(NF + 100,3);
    frame_rotation = zeros(NF + 100,3);

    K = 100;
    //======Frames======//
    // Frame 1: LF
    K = K + 1;
    frame_axis.row(K-1) = -1;
    tmp << 0 << 0 << 0 <<endr;
//    tmp << 0 << 0 << -0.001*94.48 <<endr;
    frame_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    frame_rotation.row(K-1) = tmp;

    // Frame 2: RF
    K = K + 1;
    frame_axis.row(K-1) = -1;
    tmp << 0 << 0 << 0 <<endr;
//    tmp << 0 << 0 << -0.001*94.48 <<endr;
    frame_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    frame_rotation.row(K-1) = tmp;

    // Frame 3: LH
    K = K + 1;
    frame_axis.row(K-1) = -1;
    tmp << 0 << 0 << 0 <<endr;
//    tmp << 0 << 0 <<  -0.001*122.2 <<endr;
    frame_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    frame_rotation.row(K-1) = tmp;

    // Frame 4: RH
    K = K + 1;
    frame_axis.row(K-1) = -1;
    tmp << 0 << 0 << 0 <<endr;
//    tmp << 0 << 0 << -0.001*122.2  <<endr;
    frame_translation.row(K-1) = tmp;
    tmp << 0 << 0 << 0 <<endr;
    frame_rotation.row(K-1) = tmp;


    // for index matching (Joint I -> Joint I-1)
    body_child_frame = body_child_frame-1;



}

//// get frame list
// each frame - corresponding joint and frame
vec get_frame_list(Huboplus &self){
    self.frame_list = -100*ones(100,1);

    //vec frame_list = zeros<vec>(1,1);
    int cnt, index_child_frame;

    cnt = 0;
    for (int i=0;i<self.NB;i++){
        self.frame_list(cnt) = i-1;
        cnt++;
        for (int j=0;j<self.body_no_child_frames(i);j++){
            index_child_frame = self.body_child_frame(i,j);
            if (index_child_frame >= 100){
                self.frame_list(cnt) = index_child_frame;
                cnt++;
            }
        }
    }
    return self.frame_list;
}


//// get bf and cf
// bf - body reference frame index
// cf - child frame index
vec get_frame_list_bf_cf(Huboplus& self){
    self.body_bf = -100*ones(self.NB, 1);
    self.body_cf = -100*ones(self.NB, 3);

    //vec frame_list = zeros<vec>(1,1);
    int index_child_frame;

    for (int i=0;i<self.NB;i++){
        self.body_bf(i) = find_index(self.frame_list, i-1);

        for (int j=0;j<self.body_no_child_frames(i);j++){
            index_child_frame = find_index(self.frame_list,self.body_child_frame(i,j));
            self.body_cf(i,j) = index_child_frame;
        }
    }
    return self.frame_list;
}

//// get body list
// the body to which each frame belongs
vec get_body_list(Huboplus& self){
    self.body_list = -100*ones(100,1);

    int index_body;

    //cout << self.frame_list << endl << endl;
    for (int k=0;k<int(self.frame_list.n_rows);k++){
        index_body = 0;
        for (int i=0;i<self.NB;i++){
            for (int j=0;j<self.body_no_child_frames(i);j++){
                if (self.body_cf(i,j) == k){
                    index_body = i;
                }
            }
        }
        self.body_list(k) = index_body;
    }
    return self.body_list;
}


//// generate body path w.r.t base
mat generate_body_path(Huboplus& self){
    self.body_path = -1*ones(self.NB,8);

    int i, K;

    for (int I=0;I<self.NB;I++){
        i = 0;
        self.body_path(I,i) = I;
        i = 1;
        K = I;
        while (K > -1){
            if (self.body_parent(K) > -1){
                self.body_path(I,i) = self.body_parent(K);
            }
            K = self.body_parent(K);
            i++;
        }
    }

    return self.body_path;
}


//// generate body path w.r.t LF
mat generate_body_path_LF(Huboplus& self){
    self.body_path_LF = -1*ones(self.NB,20);

    int body_index_FOOT = self.B_LAR;
    int base_body = body_index_FOOT;

    int index_tmp1, index_tmp2, index_tmp3, value_tmp;
    mat tmp, tmp1, body_path_tmp1;

    for (int I=0;I<self.NB;I++){
        // find the index of the current body in the list for base body
        index_tmp1 = find_index_mat(self.body_path.row(base_body),I);
        // if non-empty
        if (index_tmp1 > -1){
            // reverse the path from the base to the Ith body
            tmp = get_submat(self.body_path.row(base_body),index_tmp1,0);
            self.body_path_LF(I,span(0,tmp.n_cols-1)) = tmp;
        }
        else{
            // if not, combine both paths
            // index of body 0 in the parent body list of body I
            index_tmp2 = find_index_mat(self.body_path.row(I),0);
            // index of the common body if found earlier than the last body
            value_tmp = self.body_path(I,index_tmp2-1);
            index_tmp3 = find_index_mat(self.body_path.row(base_body), value_tmp);

            // if it wasn't found
            if (index_tmp3 == -100){
                // parent body path of body I without body 0
                body_path_tmp1 = self.body_path(I,span(0,index_tmp2-1));
                // append the body path of base body until the common body
                tmp1 = join_rows(body_path_tmp1,get_submat(self.body_path.row(base_body),self.body_path.n_cols-1,0));
            }
            // if found
            else{
                // parent body path of body I without common body
                body_path_tmp1 = self.body_path(I,span(0,index_tmp2-2));

                // append the body path of base body until the common body
                tmp1 = join_rows(body_path_tmp1,get_submat(self.body_path.row(base_body),index_tmp3,0));
            }
            self.body_path_LF(I,span(0,tmp1.n_cols-1)) = tmp1;
        }
    }
    return self.body_path_LF;
}

//// generate body path w.r.t RF
mat generate_body_path_RF(Huboplus& self){
    self.body_path_RF = -1*ones(self.NB,20);

    int body_index_FOOT = self.B_RAR;
    int base_body = body_index_FOOT;

    int index_tmp1, index_tmp2, index_tmp3, value_tmp;
    mat tmp, tmp1, body_path_tmp1;

    for (int I=0;I<self.NB;I++){
        // find the index of the current body in the list for base body
        index_tmp1 = find_index_mat(self.body_path.row(base_body),I);
        // if non-empty
        if (index_tmp1 > -1){
            // reverse the path from the base to the Ith body
            tmp = get_submat(self.body_path.row(base_body),index_tmp1,0);
            self.body_path_RF(I,span(0,tmp.n_cols-1)) = tmp;
        }
        else{
            // if not, combine both paths
            // index of body 0 in the parent body list of body I
            index_tmp2 = find_index_mat(self.body_path.row(I),0);
            // index of the common body if found earlier than the last body
            value_tmp = self.body_path(I,index_tmp2-1);
            index_tmp3 = find_index_mat(self.body_path.row(base_body), value_tmp);

            // if it wasn't found
            if (index_tmp3 == -100){
                // parent body path of body I without body 0
                body_path_tmp1 = self.body_path(I,span(0,index_tmp2-1));
                // append the body path of base body until the common body
                tmp1 = join_rows(body_path_tmp1,get_submat(self.body_path.row(base_body),self.body_path.n_cols-1,0));
            }
            // if found
            else{
                // parent body path of body I without body 0
                body_path_tmp1 = self.body_path(I,span(0,index_tmp2-2));

                // append the body path of base body until the common body
                tmp1 = join_rows(body_path_tmp1,get_submat(self.body_path.row(base_body),index_tmp3,0));
            }
            self.body_path_RF(I,span(0,tmp1.n_cols-1)) = tmp1;
        }
    }
    return self.body_path_RF;
}

/* /// Remove the body 0 and body 1 intelligently
we only remove the body from the list which is parent for both next
body in the list and the previous in the list meaning that both bodies
are connected through child frames that belong to the corresponding
body so that the joint frame of that body doesn't need to be considered
in describing the motion constituted by the joint rotations */

/// Filter for body_path_LF
mat filter_body_path_LF(Huboplus& self){
    self.body_path_LF_tmp = -1*ones(self.NB,20);
    mat body_path_LF_I;
    mat body_path_tmp = -1*ones(1,20);
    int n_bodies, cnt, index_body;

    for (int I=0;I<self.NB;I++){
        cnt = 0;
        body_path_LF_I = self.body_path_LF.row(I);
        n_bodies = find_index_mat(body_path_LF_I, -1);
        for (int k=0;k<n_bodies;k++){
            index_body = body_path_LF_I(0,k);
            if (k > 0 && k < n_bodies-1){
                if (index_body == self.body_parent(body_path_LF_I(0,k-1))&& index_body==self.body_parent(body_path_LF_I(0,k+1))){
                    //                    cout << "same" <<endl;
                }
                else{
                    body_path_tmp(0,cnt) = body_path_LF_I(0,k);
                    cnt ++;
                }
            }
            else{
                body_path_tmp(0,cnt) = body_path_LF_I(0,k);
                cnt ++;
            }
        }
        self.body_path_LF_tmp(I,span(0,cnt-1)) = body_path_tmp(0,span(0,cnt-1));
    }
    self.body_path_LF = self.body_path_LF_tmp;
    return self.body_path_LF;
}

/// Filter for body_path_RF
mat filter_body_path_RF(Huboplus& self){
    self.body_path_RF_tmp = -1*ones(self.NB,20);
    mat body_path_RF_I;
    mat body_path_tmp = -1*ones(1,20);
    int n_bodies, cnt, index_body;

    for (int I=0;I<self.NB;I++){
        cnt = 0;
        body_path_RF_I = self.body_path_RF.row(I);
        n_bodies = find_index_mat(body_path_RF_I, -1);
        for (int k=0;k<n_bodies;k++){
            index_body = body_path_RF_I(0,k);
            if (k > 0 && k < n_bodies-1){
                if (index_body == self.body_parent(body_path_RF_I(0,k-1))&& index_body==self.body_parent(body_path_RF_I(0,k+1))){
                    //                    cout << "same" <<endl;
                }
                else{
                    body_path_tmp(0,cnt) = body_path_RF_I(0,k);
                    cnt ++;
                }
            }
            else{
                body_path_tmp(0,cnt) = body_path_RF_I(0,k);
                cnt ++;
            }
        }
        self.body_path_RF_tmp(I,span(0,cnt-1)) = body_path_tmp(0,span(0,cnt-1));
    }
    self.body_path_RF = self.body_path_RF_tmp;
    return self.body_path_RF_tmp;
}

//// Homogeneous Transformation Matrix
// written on 08-26-13
mat HT_tree(Huboplus& self, vec q_data, int index_frame){
    int index, i_joint;
    double q, x, y, z;
    mat T, T_i;
    mat R_axis;

    //cout<<self.frame_list<<endl;
    index = self.frame_list(index_frame);

    // a fixed frame
    if(index >= 100){
        i_joint = self.body_list(index_frame) - 1;
        q = q_data(i_joint);;
        R_axis = angvec2tr(q, self.joint_axis.row(i_joint));

        x = self.frame_translation(index, 0);
        y = self.frame_translation(index, 1);
        z = self.frame_translation(index, 2);

        T = transl2tr(x,y,z);

        T_i = R_axis*T;
    }
    // joints
    else{
        i_joint = self.body_list(index_frame) - 1;
        // if first joint in a branch
        if (i_joint < 0){
            R_axis = trotz(0);
        }
        // not the first joint in a branch
        else{
            q = q_data(i_joint);
            R_axis = angvec2tr(q, self.joint_axis.row(i_joint));
        }

        x = self.joint_translation(index, 0);
        y = self.joint_translation(index, 1);
        z = self.joint_translation(index, 2);

        T = transl2tr(x,y,z);

        T_i = R_axis*T;
    }

    return T_i;
}

//// Forward Kinematics
// written on 08-26-13
mat FK_tree(Huboplus& self, vec q_data, int index_frame, int reference){
    mat T_FK, T_0_REF;
    int index_joint, F_REF;
    T_FK = trotz(0);

    // backward recursion
    // until it reaches the first frame in the branch
    while (index_frame > -1){
        //        cout << "index_frame: " << index_frame << endl;
        //        cout << "HT: " << HT_tree(self, q_data, index_frame) << endl;
        T_FK = HT_tree(self, q_data, index_frame)*T_FK;
        // find the joint index that moves the body which the frame belongs to
        index_joint = self.body_list(index_frame) - 1;
        // find the index of the frame for the joint
        if (index_joint >= 0){
            index_frame = find_index(self.frame_list, index_joint);
        }
        // stop at the base
        else{
            index_frame = -1;
        }
    }

    // reference 0: body, 2: LF, 3: RF
    if (reference >= 2){
        if (reference == 2){
            F_REF = self.F_LF;
        }
        else if (reference == 3){
            F_REF = self.F_RF;
        }

        T_0_REF = FK_tree(self,q_data,F_REF);
        T_FK = inv(T_0_REF)*T_FK;
    }
    return T_FK;
}

//// Jacobian Computation
// written on 08-26-13
mat J_tree(Huboplus& self, vec q_data, int index_frame, int reference){
    mat Jacob, T_0_E, R_0_E, R_E_0, p_k, T_bi, p_bi, u_bi_i_1, R_0_bi, u_i_1, w_k, v_k, mask_tmp, body_path_B, mask, T_0_B, R_0_B, T_0_k, T_B_k, p_B_k, body_path;
    mat T_0_bi, T_B_bi, p_B_bi, u_B_i_1, u_0_i_1;
    int I, index_bi, cnt, F_REF;

    Jacob = zeros<mat>(6, self.NJ);
    I = self.body_list(index_frame);

    // body is the REF
    if (reference < 2){
        T_0_E = FK_tree(self, q_data, index_frame);
        R_0_E = t2r(T_0_E);
        R_E_0 = R_0_E.t();
        p_k = tr2transl(T_0_E);

        while (I > 0){
            index_bi = self.body_bf(I);
            //            cout << "index_bi: " << index_bi << endl;
            T_bi = FK_tree(self, q_data, index_bi);
            //            cout << "T_bi: " << T_bi << endl;
            p_bi = tr2transl(T_bi);
            //            cout << "p_bi: " << p_bi << endl;
            u_bi_i_1 = self.joint_axis.row(I-1).t();
            R_0_bi = t2r(T_bi);
            // u_i_1 is w.r.t b(i)
            u_i_1 = R_0_bi*u_bi_i_1;
            w_k = u_i_1;
            //            cout << "w_k: " << w_k << endl;
            v_k = cross(u_i_1, p_k-p_bi);
            //            cout << "v_k: " << v_k << endl;

            Jacob.col(I-1) = join_cols(v_k, w_k);
            I = self.body_parent(I);
        }

        // if the reference is the end-effector
        if (reference == 1){
            //            cout << join_cols(join_rows(R_E_0,zeros(3,3)), join_rows(zeros(3,3),R_E_0))<<endl;
            Jacob = join_cols(join_rows(R_E_0,zeros(3,3)), join_rows(zeros(3,3),R_E_0))*Jacob;
        }
    }
    // if the reference is one of the feet
    else if (reference >= 2){
        /* the joint rotations in the support leg chain (from support foot to
        the waist) causes the movement in the negative direction w.r.t the
        support foot frame, so we need to incorporate the signs in the computation. */

        mask_tmp = ones(self.NJ,1);
        mask_tmp(self.J_HPY-1,0) = -mask_tmp(self.J_HPY-1,0);

        // reference is LF
        if (reference == 2){
            F_REF = self.F_LF;
            body_path_B = self.body_path_LF;
            mask_tmp(span(self.J_LHY-1,self.J_LF-1),0) = -mask_tmp(span(self.J_LHY-1,self.J_LF-1),0);

        }
        // reference is RF
        else if (reference == 3){
            F_REF = self.F_RF;
            body_path_B = self.body_path_RF;
            mask_tmp(span(self.J_RHY-1,self.J_RF-1),0) = -mask_tmp(span(self.J_RHY-1,self.J_RF-1),0);
        }

        mask = diagmat(mask_tmp);
        //        cout << "mask: "  <<endl << mask << endl;

        // transformation from base to the support foot frame
        T_0_B = FK_tree(self, q_data, F_REF);
        R_0_B = t2r(T_0_B);
        T_0_k = FK_tree(self, q_data, index_frame);
        T_B_k = inv(T_0_B)*T_0_k;
        p_B_k = tr2transl(T_B_k);

        // compute the jacobian
        cnt = 0;
        body_path = body_path_B.row(I);
        //        cout << body_path << endl;
        I = body_path(0,cnt);
        while (I >= 0){
            if (I > 0){
                index_bi = self.body_bf(I);
                //                cout << "index_bi: " << index_bi << endl;
                T_0_bi = FK_tree(self, q_data, index_bi);
                //                cout << "T_0_bi: " << T_0_bi << endl;
                R_0_bi = t2r(T_0_bi);
                T_B_bi = inv(T_0_B)*T_0_bi;
                p_B_bi = tr2transl(T_B_bi);
                //                cout << "p_B_bi: " << p_B_bi << endl;

                u_bi_i_1 = self.joint_axis.row(I-1).t();
                u_0_i_1 = R_0_bi*u_bi_i_1;
                u_B_i_1 = R_0_B.t()*u_0_i_1;

                w_k = u_B_i_1;
                //                cout << "w_k: " << w_k << endl;
                v_k = cross(u_B_i_1, (p_B_k-p_B_bi));
                //                cout << "v_k: " << v_k << endl;
                Jacob.col(I-1) = join_cols(v_k, w_k);
            }
            cnt += 1;
            I = body_path(0, cnt);
        }

        // Apply the mask
        Jacob = Jacob*mask;
    }

    return Jacob;
}

//// CoM Computation
// written on 08-26-13
mat CoM(Huboplus& self, vec q_data, int reference){
    double M_robot;
    mat CoM_robot, T_bi, R_q_i_1, s_i_star, p_CoM_i, tmp, T_0_REF;
    int bi, K, F_REF;

    M_robot = 0;
    CoM_robot = zeros(1,3);
    for (int i=1; i<self.NB; i++){
        M_robot = M_robot + self.body_mass(i);
        bi = self.body_bf(i);

        // location of body reference frame of body i
        T_bi = FK_tree(self, q_data, bi);
        //cout<< "bi: " << bi << endl;
        //T_bi.print();

        // rotation by joint i-1
        R_q_i_1 = angvec2r(q_data(i-1), self.joint_axis.row(i-1));

        // position of center of mass of body i rotated by joint i-1
        // w.r.t bi frame
        s_i_star = R_q_i_1*self.body_com.row(i).t();
        tmp << 1 << endr;
        //cout << "s_i_star: " << s_i_star << endl;
        p_CoM_i = T_bi*join_cols(s_i_star, tmp);
        //cout << i <<" "<< "p_com_i: " << p_CoM_i << endl;
        CoM_robot = CoM_robot + self.body_mass(i)*p_CoM_i(span(0,2),0).t();
        //CoM_robot.print();
    }

    // body 1
    K = 0;
    M_robot = M_robot + self.body_mass(K);
    p_CoM_i = self.body_com.row(K).t();
    CoM_robot = CoM_robot + self.body_mass(K)*p_CoM_i(span(0,2),0).t();

    // compute CoM
    CoM_robot = CoM_robot/M_robot;

    if (reference > 1){
        if (reference == 2){
            F_REF = self.F_LF;
        }
        else if (reference == 3){
            F_REF = self.F_RF;
        }

        T_0_REF = FK_tree(self, q_data, F_REF);
        //        cout << "CoM_robot: " << CoM_robot << endl;
        CoM_robot = inv(T_0_REF)*join_cols(CoM_robot.t(), tmp);
        CoM_robot = CoM_robot(span(0,2),0).t();
    }

    return CoM_robot;
}

//// CoM Jacobian Computation
// written on 08-26-13
mat J_com_tree(Huboplus& self, vec q_data, int reference){

    // initialize J_mi
    cube Jacob_mi_L = zeros<cube>(3, self.NJ, self.NB);
    mat Jacob_com_L = zeros(3, self.NJ);

    mat Jacob, Jacob_L, R_q_i_1, T_bi, s_i_star, p_com_i, tmp, R_bi, R_bi_base;
    mat T_0_bi, p_bi, u_bi_i_1, R_0_bi, u_i_1, w_mi, v_mi;
    int bi, I, index_bi;
    mat mask_tmp, T_0_B, R_0_B, T_B_0, R_B_0, T_B_bi, p_B_com_i, R_B_bi, R_bi_B,body_path_B, mask, body_path, x, y, p_B_bi, u_0_i_1, u_B_i_1, w_k, v_k, p_B_k;
    int F_REF, cnt;

    // if the reference is body
    if (reference < 1){
        for (int i=1; i<self.NB; i++){
            Jacob = zeros(6, self.NJ);
            Jacob_L = zeros(3, self.NJ);

            // reference frame of body i
            bi = self.body_bf(i);
            //            cout << "bi: " << bi << endl;

            // location of body reference frame of body i
            T_bi = FK_tree(self, q_data, bi);
            //            cout << i << ", T_bi: " << endl << T_bi << endl;

            // rotation by joint i-1
            R_q_i_1 = angvec2r(q_data(i-1), self.joint_axis.row(i-1));

            // position of center of mass of body i rotated by joint i-1
            // w.r.t bi frame
            s_i_star = R_q_i_1*self.body_com.row(i).t();
            //            cout << "s_i_star: " << s_i_star << endl;

            // p_com_i = position of com of body i w.r.t base frame
            tmp << 1 <<endr;
            p_com_i = T_bi*join_cols(s_i_star,tmp);
            p_com_i = p_com_i(span(0,2),0);

            // rotation matrix of b(i) w.r.t base frame
            R_bi = t2r(T_bi);
            // rotation matrix of base frame w.r.t b(i) frame
            R_bi_base = R_bi.t();

            // computation of J_mi
            I = i;
            while (I>0){
                index_bi = self.body_bf(I);
                T_0_bi = FK_tree(self, q_data, index_bi);
                p_bi = tr2transl(T_0_bi);
                u_bi_i_1 = self.joint_axis.row(I-1).t();
                R_0_bi = t2r(T_0_bi);
                u_i_1 = R_0_bi*u_bi_i_1;

                w_mi = u_i_1;
                v_mi = cross(u_i_1, (p_com_i - p_bi));
                Jacob.col(I-1) = join_cols(v_mi, w_mi);

                // the parent body becomes next body associated with kth frame
                I = self.body_parent(I);
            }

            // J_i
            Jacob_L = Jacob.rows(0,2);
            Jacob_mi_L.slice(i) = Jacob_L;

        }
    }

    // if the reference is the feet
    else if (reference > 1){
        mask_tmp = ones(self.NJ,1);
        mask_tmp(self.J_HPY-1,0) = -mask_tmp(self.J_HPY-1,0);

        // reference is LF
        if (reference == 2){
            F_REF = self.F_LF;
            body_path_B = self.body_path_LF;
            mask_tmp(span(self.J_LHY-1,self.J_LF-1),0) = -mask_tmp(span(self.J_LHY-1,self.J_LF-1),0);

        }
        // reference is RF
        else if (reference == 3){
            F_REF = self.F_RF;
            body_path_B = self.body_path_RF;
            mask_tmp(span(self.J_RHY-1,self.J_RF-1),0) = -mask_tmp(span(self.J_RHY-1,self.J_RF-1),0);
        }

        //cout<< "mask: " << mask_tmp.t() << endl;
        mask = diagmat(mask_tmp);
        //        cout << "mask: "  <<endl << mask << endl;

        // transformation from base to the support foot frame
        T_0_B = FK_tree(self, q_data, F_REF);
        R_0_B = t2r(T_0_B);

        for (int i=0; i<self.NB; i++){
            Jacob = zeros(6, self.NJ);
            Jacob_L = zeros(3, self.NJ);

            // for body 1
            if (i==0){

                // location of body reference frame of body 1 w.r.t base
                T_B_0 = inv(T_0_B);
                // position of center of mass of body i rotated by joint i-1
                // w.r.t bi frame
                s_i_star = self.body_com.row(i).t();
                // p_com_i = position of com of body i w.r.t base frame
                tmp << 1 <<endr;
                p_B_com_i = T_B_0*join_cols(s_i_star,tmp);
                p_B_com_i = p_B_com_i(span(0,2),0);
                //                cout << "p_com_i: " << p_com_i << endl;

                // rotation matrix of base w.r.t support foot frame
                R_B_0 = t2r(T_B_0);

                // rotation matrix of support foot frame w.r.t base frame
                R_0_B = R_B_0.t();

            }
            else{

                // reference frame of body i
                bi = self.body_bf(i);
                //            cout << "bi: " << bi << endl;

                // location of body reference frame of body i
                T_B_bi = inv(T_0_B)*FK_tree(self, q_data, bi);
                //            cout << i << ", T_bi: " << endl << T_bi << endl;

                // rotation by joint i-1
                R_q_i_1 = angvec2r(q_data(i-1), self.joint_axis.row(i-1));

                // position of center of mass of body i rotated by joint i-1
                // w.r.t bi frame
                s_i_star = R_q_i_1*self.body_com.row(i).t();
                //            cout << "s_i_star: " << s_i_star << endl;

                // p_B_com_i = position of com of body i w.r.t base frame
                tmp << 1 <<endr;
                //cout << join_cols(s_i_star, tmp) << endl;
                p_B_com_i = T_B_bi*join_cols(s_i_star, tmp);
                p_B_com_i = p_B_com_i(span(0,2),0);


                // rotation matrix of b(i) w.r.t support foot frame
                R_B_bi = t2r(T_B_bi);
                // rotation matrix of support foot frame w.r.t b(i) frame
                R_bi_B = R_B_bi.t();
            }

            /// computation of J_mi w.r.t support foot frame
            // get the body path from body i to the body of support foot
            body_path = body_path_B.row(i);

            /*
                check if body path to the body i is a subset of body path of
                body 0. If yes, the index starts from the second one in the
                path since the body movement is expressed w.r.t its child
                joint frame instead of the body reference frame.
            */

            // check if the body path of body I (x) is a subset of body path of body 1 (y)
            y = body_path_B.row(0);
            x = body_path;

            if (issubset(y, x) == true){
                cnt = 1;
            }
            else{
                cnt = 0;
            }


            I = body_path(0,cnt);
            while (I >= 0){
                if (I > 0){
                    index_bi = self.body_bf(I);
                    //                cout << "index_bi: " << index_bi << endl;
                    T_0_bi = FK_tree(self, q_data, index_bi);
                    //                cout << "T_0_bi: " << T_0_bi << endl;
                    R_0_bi = t2r(T_0_bi);
                    T_B_bi = inv(T_0_B)*T_0_bi;
                    p_B_bi = tr2transl(T_B_bi);
                    //                cout << "p_B_bi: " << p_B_bi << endl;

                    u_bi_i_1 = self.joint_axis.row(I-1).t();
                    u_0_i_1 = R_0_bi*u_bi_i_1;
                    u_B_i_1 = R_0_B.t()*u_0_i_1;

                    w_k = u_B_i_1;
                    //                cout << "w_k: " << w_k << endl;
                    v_k = cross(u_B_i_1, (p_B_com_i-p_B_bi));
                    //                cout << "v_k: " << v_k << endl;
                    Jacob.col(I-1) = join_cols(v_k, w_k);
                }
                cnt += 1;
                I = body_path(0, cnt);
            }

            // Apply the mask
            Jacob = Jacob*mask;
            Jacob_L = Jacob.rows(0,2);
            //cout << i << " Jacob_L: " << Jacob_L << endl;
            Jacob_mi_L.slice(i) = Jacob_L;

        }
    }

    // compute Jcom_L
    double M_robot = 0;
    for (int i=0; i<self.NB; i++){
        M_robot = M_robot + self.body_mass(i);
        // accumulate mi*Jmi_L
        Jacob_com_L = Jacob_com_L + self.body_mass(i)*Jacob_mi_L.slice(i);
    }
    // Jcom_L = sum_i(mi*Jmi_L)/M_robot
    Jacob_com_L = Jacob_com_L/M_robot;


    return Jacob_com_L;
}

//// joint limit checking (clamping)
// written on 08-28-13
mat joint_limit_check(Huboplus& self, mat q_data){
    int flag = 0;
    double q_min, q_max;
    for (int I=0; I<self.NJ; I++){
        q_min = self.joint_q_min(I);
        q_max = self.joint_q_max(I);

        if (q_data(I,0) > q_max){
            q_data(I,0) = q_max;
            flag = 1;
        }
        else if (q_data(I,0) < q_min){
            q_data(I,0) = q_min;
            flag = 1;
        }

        if (flag == 1){
            cout << "joint " << I+1 << " limit has reached" << endl;
        }
        flag = 0;
    }

    return q_data;
}


// Initializes robot class
void robot_init (Huboplus& robot){
    get_frame_list(robot);
    get_frame_list_bf_cf(robot);
    get_body_list(robot);
    generate_body_path(robot);
    generate_body_path_LF(robot);
    generate_body_path_RF(robot);
    filter_body_path_LF(robot);
    filter_body_path_RF(robot);
}

//// CoM motion generation
// written on 09-02-13
mat generate_com_motion(Huboplus& self, vec q_data, int ContactState, int N, double margin_x, double margin_y, int mode){
    mat CoM_init = CoM(self, q_data, ContactState);
    //cout << "CoM_init" << CoM_init << endl;
    mat CoM_des = CoM_init;
    mat CoM_zero = CoM(self, zeros(self.NJ,1), ContactState);
    //cout << "CoM_init_zero" << CoM_zero << endl;
    CoM_des(0,0) = margin_x;

    if (ContactState == self.LF){
        CoM_des(0,1) = - margin_y ;
    }
    else if(ContactState == self.RF){
        CoM_des(0,1) = + margin_y ;
    }

    vec xdata;
    if (mode == 0){
        xdata = CosTraj(N, CoM_init(0,0), CoM_des(0,0));
    }
    // no movement in x direction
    else if (mode == 1){
        xdata = CosTraj(N, CoM_init(0,0), CoM_init(0,0));
    }

    // y direction
    vec ydata = CosTraj(N, CoM_init(0,1), CoM_des(0,1));

    //cout<<"xdata: " << xdata.t() << endl;
    //cout<<"ydata: " << ydata.t() << endl;

    // generate CoM trajectory
    mat com_data = zeros(N,3);

    mat com_tmp;
    for (int I=0; I<N; I++){
        com_tmp = CoM_init;
        com_tmp(0,0) = xdata(I);
        com_tmp(0,1) = ydata(I);
        com_data.row(I) = com_tmp;
    }

    return com_data;
}

//// generate motion to desired CoM from current CoM
// written on 09-02-13
mat com_motion_des(Huboplus& self, vec q_data, int ContactState, int N, mat CoM_des){
    mat CoM_init = CoM(self, q_data, ContactState);

    vec xdata = CosTraj(N, CoM_init(0,0), CoM_des(0,0));
    vec ydata = CosTraj(N, CoM_init(0,1), CoM_des(0,1));
    vec zdata = CosTraj(N, CoM_init(0,2), CoM_des(0,2));

    // generate CoM trajectory
    mat com_data = zeros(N, 3);

    mat com_tmp;
    for (int I=0; I<N; I++){
        com_tmp = CoM_init;
        com_tmp(0,0) = xdata(I);
        com_tmp(0,1) = ydata(I);
        com_tmp(0,2) = zdata(I);
        com_data.row(I) = com_tmp;
    }

    return com_data;
}

//// generate stepping motion
// written on 09-02-13
cube generate_stepping_motion(Huboplus& self, vec q_data, int ContactState, double stepsize_h, double stepheight, int N, double stepsize_v, int climbing_mode, double stepcollision, double stepsize_y, int N_col, double rotation_z){

    cube Tdata = zeros<cube>(4,4,N);

    mat T_F_init;
    if (ContactState == self.LF){
        T_F_init = FK_tree(self, q_data, self.F_RF, ContactState);
    }
    else if (ContactState == self.RF){
        T_F_init = FK_tree(self, q_data, self.F_LF, ContactState);
    }

    vec xdata = zeros<vec>(N,1);

    if (climbing_mode == 0){
        xdata = CosTraj(N, T_F_init(0,3), T_F_init(0,3) + stepsize_h);
    }
    else{
        // generate stepping motion in x direction avoiding the collision with the rung
        xdata.rows(0,N-(N_col)-1) = CycloidTraj(N-N_col, T_F_init(0,3), stepcollision, T_F_init(0,3) + stepsize_h);
        xdata.rows(N-N_col, xdata.n_rows-1) = xdata(N-N_col-1)*ones<vec>(N_col,1);
    }

    // side direction
    vec ydata = CosTraj(N, T_F_init(1,3), T_F_init(1,3) + stepsize_y);

    // z direction
    vec zdata = CycloidTraj(N, T_F_init(2,3), stepheight, stepsize_v);

    // rotation data
    vec ang_data = CosTraj(N, 0, rotation_z);

    mat T_F_tmp, R_F_tmp;
    // generate swing foot trajectory
    for (int I=0; I<N; I++){
        T_F_tmp = T_F_init;
        R_F_tmp = t2r(T_F_tmp);
        T_F_tmp(0,3) = xdata(I);
        T_F_tmp(1,3) = ydata(I);
        T_F_tmp(2,3) = zdata(I);
        R_F_tmp = R_F_tmp*rotz(ang_data(I));
        T_F_tmp(span(0,2),span(0,2)) = R_F_tmp;
        Tdata.slice(I) = T_F_tmp;

    }

    return Tdata;

}

/// Save to Hubo-Ach format
// written on 08-27-13
void motion2ach(Huboplus& self, mat qdata, string file_name, int send2ach){
    mat qdata_ach = zeros(qdata.n_rows, 40);

//    // send to Hubo-Ach
//    if (send2ach == 1){
//        // convert to motion with offset
//        qdata = Model2Robot(self, qdata);
//    }

    vec joint_map;
    joint_map<<JA_WST<<JA_LHY<<JA_LHR<<JA_LHP<<JA_LKN<<JA_LAP<<JA_LAR
            <<JA_RHY<<JA_RHR<<JA_RHP<<JA_RKN<<JA_RAP<<JA_RAR
           <<JA_LSP<<JA_LSR<<JA_LSY<<JA_LEB<<JA_LWY<<JA_LWP
          <<JA_RSP<<JA_RSR<<JA_RSY<<JA_REB<<JA_RWY<<JA_RWP<<endr;

    //cout << "joint_map: " << joint_map << endl;

    for (int i=0; i<int(qdata.n_rows); i++){
        for (int j=0; j<self.NJ; j++){
            qdata_ach(i,joint_map(j)) = qdata(i,j);
        }
    }

    //cout << shape(qdata_ach) << endl;

    // save into a file
    qdata_ach.save(file_name, raw_ascii);

    // send to Hubo-Ach
    if (send2ach == 1){
        int mode = 2; // does what? 2: virtual
        bool compliance=false;
        bool pause=false;

        char *filename=new char[file_name.size()+1];
        filename[file_name.size()]=0;
        memcpy(filename,file_name.c_str(),file_name.size());
        //char *filename = file_name;

        cout << "sending motion to Hubo-Ach" << endl;
        //runTrajFunction(filename, mode, compliance, pause);
        system(("sudo /home/andypark/Projects/hubo-read-trajectory/hubo-read-trajectory -s -f 200 -n /home/andypark/workspace/Robocpp/build-Robocpp-Desktop_Qt_5_1_0_GCC_64bit-Release/"+file_name).c_str());
//        system(("sudo /home/hubo/andypark/hubo-read-trajectory/hubo-read-trajectory -f 200 -n /home/hubo/andypark/Robocpp/build-Robocpp-Desktop_Qt_5_1_1_GCC_64bit-Release/"+file_name).c_str());
    }

}

// Read current joint angle from Hubo-Ach
vec ReadAchJoint(Huboplus& self){

    int mode = 1; // 1: actual, 2: virtual

    vec encoderValues = getEncoderValues_new();

    vec q_data = zeros<vec>(self.NJ,1);

    vec joint_map;
    joint_map<<JA_WST<<JA_LHY<<JA_LHR<<JA_LHP<<JA_LKN<<JA_LAP<<JA_LAR
            <<JA_RHY<<JA_RHR<<JA_RHP<<JA_RKN<<JA_RAP<<JA_RAR
           <<JA_LSP<<JA_LSR<<JA_LSY<<JA_LEB<<JA_LWY<<JA_LWP
          <<JA_RSP<<JA_RSR<<JA_RSY<<JA_REB<<JA_RWY<<JA_RWP<<endr;

    for (int i=0; i<self.NJ; i++){
            q_data(i) = encoderValues(joint_map(i));
    }

    return q_data;
}

// convert the joint angle of the model to Hubo with offset
mat Model2Robot(Huboplus& self, mat q_data){
    mat q_data1;
    q_data1 = q_data;

    double Offset_LSR = deg2rad(20);
    double Offset_RSR = -deg2rad(20);

    for (int i=0; i < q_data.n_rows; i++){

        q_data1(i, self.J_LSR-1) = q_data(i, self.J_LSR-1) - Offset_LSR;
        q_data1(i, self.J_RSR-1) = q_data(i, self.J_RSR-1) - Offset_RSR;

    }

    return q_data1;
}

// convert the joint angle of the hubo to model with offset
mat Robot2Model(Huboplus& self, mat q_data){
    mat q_data1;
    q_data1 = q_data;

    double Offset_LSR = deg2rad(20);
    double Offset_RSR = -deg2rad(20);

    for (int i=0; i < q_data.n_rows; i++){

        q_data1(i, self.J_LSR-1) = q_data(i, self.J_LSR-1) + Offset_LSR;
        q_data1(i, self.J_RSR-1) = q_data(i, self.J_RSR-1) + Offset_RSR;

    }

    return q_data1;
}
