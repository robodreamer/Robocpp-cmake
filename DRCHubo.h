/*
 * DRCHubo.h
 * This file is part of <Robocpp>
 *
 * Copyright (C) 2013 - Andy Park (andypark@purdue.edu)
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

#ifndef DRCHUBO_H
#define DRCHUBO_H

#include <iostream>
#include <cstdio>
#include <armadillo>
#include <ctime>
#include <math.h>
#include <iomanip>
#include "Robocpp.h"
#include "hubo-read-trajectory-as-func.h"

// Define constants for DoFs to control and finger state
#define ALL_JOINTS 40
#define BODY_JOINTS 26
#define EXCEPT_FINGERS 30
#define FINGER_CLOSE 1
#define FINGER_OPEN -1
#define FINGER_NEUTRAL 0

// Define constants for joint names
#define JA_RHY 0
#define JA_RHR 1
#define JA_RHP 2
#define JA_RKN 3
#define JA_RAP 4
#define JA_RAR 5

#define JA_LHY 6
#define JA_LHR 7
#define JA_LHP 8
#define JA_LKN 9
#define JA_LAP 10
#define JA_LAR 11

#define JA_RSP 12
#define JA_RSR 13
#define JA_RSY 14
#define JA_REB 15
#define JA_RWY 16
#define JA_RWR 17
#define JA_RWP 18

#define JA_LSP 19
#define JA_LSR 20
#define JA_LSY 21
#define JA_LEB 22
#define JA_LWY 23
#define JA_LWR 24
#define JA_LWP 25

#define JA_NKY 26
#define JA_NK1 27
#define JA_NK2 28
#define JA_WST 29

#define JA_RF1 30
#define JA_RF2 31
#define JA_RF3 32
#define JA_RF4 33
#define JA_RF5 34

#define JA_LF1 35
#define JA_LF2 36
#define JA_LF3 37
#define JA_LF4 38
#define JA_LF5 39


using namespace std;
using namespace arma;

class DRCHubo
{

public:
    int NB, NJ, NF, NLimb;
    vec grav;
    int BODY, LF, RF;
    int J_HPY;
    int J_LHY, J_LHR, J_LHP, J_LKP, J_LAP, J_LAR, J_LF;
    int J_RHY, J_RHR, J_RHP, J_RKP, J_RAP, J_RAR, J_RF;
    int J_LSP, J_LSR, J_LSY, J_LEP, J_LWY, J_LWP, J_LWR, J_LH;
    int J_RSP, J_RSR, J_RSY, J_REP, J_RWY, J_RWP, J_RWR, J_RH;
    int B_Torso, B_Hip;
    int B_LHY, B_LHR, B_LHP, B_LKP, B_LAP, B_LAR, B_LF;
    int B_RHY, B_RHR, B_RHP, B_RKP, B_RAP, B_RAR, B_RF;
    int B_LSP, B_LSR, B_LSY, B_LEP, B_LWY, B_LWP, B_LWR, B_LH;
    int B_RSP, B_RSR, B_RSY, B_REP, B_RWY, B_RWP, B_RWR, B_RH;
    int Frame_LF, Frame_RF, Frame_LH, Frame_RH;
    int F_HPY;
    int F_LHY, F_LHR, F_LHP, F_LKP, F_LAP, F_LAR, F_LF;
    int F_RHY, F_RHR, F_RHP, F_RKP, F_RAP, F_RAR, F_RF;
    int F_LSP, F_LSR, F_LSY, F_LEP, F_LWY, F_LWP, F_LWR, F_LH;
    int F_RSP, F_RSR, F_RSY, F_REP, F_RWY, F_RWP, F_RWR, F_RH;

    //Bodies
    mat body_com, body_I, body_child_frame;
    vec body_mass, body_parent, body_no_child_frames;

    //Joints
    mat joint_axis, joint_translation, joint_rotation;
    vec joint_q_min, joint_q_max;

    //Frames
    vec frame_axis;
    mat frame_translation, frame_rotation;

    //Lists
    vec frame_list, body_list, body_bf;

    mat body_cf, body_path, body_path_LF, body_path_RF, body_path_LF_tmp, body_path_RF_tmp;

    DRCHubo();
    void initialize();

};

//// get frame list
// each frame - corresponding joint and frame
vec get_frame_list(DRCHubo& self);

//// get bf and cf
// bf - body reference frame index
// cf - child frame index
vec get_frame_list_bf_cf(DRCHubo& self);

//// get body list
// the body to which each frame belongs
vec get_body_list(DRCHubo& self);

//// generate body path w.r.t base
mat generate_body_path(DRCHubo& self);

//// generate body path w.r.t LF
mat generate_body_path_LF(DRCHubo& self);

//// generate body path w.r.t RF
mat generate_body_path_RF(DRCHubo& self);

/// Filter for body_path_LF
mat filter_body_path_LF(DRCHubo& self);

/// Filter for body_path_RF
mat filter_body_path_RF(DRCHubo& self);

//// Homogeneous Transformation Matrix
// written on 08-26-13
mat HT_tree(DRCHubo& self, vec q_data, int index_frame);

//// Forward Kinematics
// written on 08-26-13
mat FK_tree(DRCHubo& self, vec q_data, int index_frame, int reference = 0);

//// Jacobian Computation
// written on 08-26-13
mat J_tree(DRCHubo& self, vec q_data, int index_frame, int reference = 0);

//// CoM Computation
// written on 08-26-13
mat CoM(DRCHubo& self, vec q_data, int reference = 0);

//// CoM Jacobian Computation
// written on 08-26-13
mat J_com_tree(DRCHubo& self, vec q_data, int reference = 0);

//// CoM motion generation
// written on 09-02-13
mat generate_com_motion(DRCHubo& self, vec q_data, int ContactState, int N, double margin_x, double margin_y, int mode = 0);

//// generate motion to desired CoM from current CoM
// written on 09-02-13
mat com_motion_des(DRCHubo& self, vec q_data, int ContactState, int N, mat CoM_des);

//// generate stepping motion
// written on 09-02-13
cube generate_stepping_motion(DRCHubo& self, vec q_data, int ContactState, double stepsize_h, double stepheight, int N, double stepsize_v, int climbing_mode = 0, double stepcollision = 0.001*50, double stepsize_y = 0.0, int N_col = 10, double rotation_z = 0);

//// joint limit checking (clamping)
// written on 08-28-13
mat joint_limit_check(DRCHubo& self, mat q_data);

// Initializes robot class
void robot_init (DRCHubo& robot);

// Read current joint angle from Hubo-Ach
vec ReadAchJoint(DRCHubo& robot);

/// Save to Hubo-Ach format
void motion2ach(DRCHubo& self, mat qdata, string file_name, int send2ach, int mode, bool compliance = false, int NJoints = BODY_JOINTS, int finger_state = FINGER_NEUTRAL);

// convert the joint angle of the model to Hubo with offset
mat Model2Robot(DRCHubo& self, mat q_data);

// convert the joint angle of the hubo to model with offset
mat Robot2Model(DRCHubo& self, mat q_data);

// map the string to joint index (Hubo-Ach)
int map_joint_str2num(string JOINT_NAME);

/// Send the motion in Hubo-Ach format to Hubo-ach
// written on 08-27-13
#define ACTUAL 1
#define VIRTUAL 2
void data2ach(mat qdata_ach, string file_name, int send2ach, bool compliance, int mode = ACTUAL);

#endif // DRCHUBO_H
