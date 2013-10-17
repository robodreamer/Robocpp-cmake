/*
 * main.cpp
 * This file is part of <Robocpp>
 *
 * Copyright (C) 2013 - Andy Park <andypark@purdue.edu>
 *
 * <Robocpp> is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * <program name> is distributed in the hope that it will be useful,
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

//#include <iostream>
#include "Robocpp.h"
#include "DRCHubo.h"
#include "Huboplus.h"

#define REAL_ROBOT 30
#define SIMULATION 5

void Tutorial();
void Test();


int main(int argc, char *argv[])
{
    clock_t t;
    t = clock();

    int select = 0;
    int program_mode = 2; // 0:no-run 1:pre-specified, 2:user-selection

    // Tutorial
    //Tutorial();
    // simple tests
    int test = 0;

    // print out the argument
    if(test == 1){


        cout << "number of argument: " << argc << endl;
        cout << "arguments: " << endl;
        for (int i=0; i<argc; i++){
            cout <<" "<< argv[i];
        }
        cout << endl << endl;

    }
    // print a selected line
    else if (test == 2){

        string line;
        int desired_line = 3;
        char filename[] = "../data.txt";
        ifstream inputfile(filename);
        for(int i = 0; i < desired_line; ++i){
           getline(inputfile, line);
        }

        // diplay specific line
        cout << desired_line <<": " <<endl;
        cout << line << endl;

        // find a specific index
        istringstream iss (line);

        double val;
        vec temp_data = zeros<vec>(40,1);
        for (int i=0; i<40; i++)
        {
            iss >> val;
            //cout << i+1 << ": " << val << endl;
            temp_data(i) = val;
        }

        int JOINT_INDEX = 1;
        cout << "data that we want: "<< temp_data(JOINT_INDEX) << endl;


        cout <<"test function: " << FindJointAngFile(filename, 2, 5) << endl;
    }

    int INDEX_JOINT;
    double JOINT_ANGLE;
    vec JOINT_DATA;
    bool compliance;

    // check if the hubo-ach should run in virtual mode or not
    // by default, it is for real robot
    double speed_motion = REAL_ROBOT;
    int MODE = ACTUAL;

    // check the status flag file (line number 3)
    int status_flag = FindJointAngFile("../status_flag.txt", 3, 0);

    if (status_flag == 1){
        cout<<"RUNNING IN VIRTUAL MODE"<< endl;
        speed_motion = SIMULATION;
        MODE = VIRTUAL;
    }
    else if(status_flag == 2){
        cout<<"RUNNING IN ACTUAL MODE"<< endl;
        speed_motion = REAL_ROBOT;
        MODE = ACTUAL;
    }

    // pre-specified
    if(program_mode == 1){
        select = 3;
    }
    // user-specified
    else if (program_mode == 2){
        // if no argument print out this
        if (argc == 1){
            // Help menu

            cout << endl << "--------------------- Robocpp --------------------" << endl;
            cout << "|  Functions written for ladder climbing event   |" << endl;
            cout << "|                                                |" << endl;
            cout << "|  (Support: H. Andy Park  andypark@purdue.edu)  |" << endl;
            cout << "--------------------------------------------------" << endl << endl;
            cout << " Usages: " << endl;
            cout << "1. Init Position Correction" << endl;
            cout << "\t $sudo ./Robocpp InitPos" << endl << endl;
            cout << "2. Init Orientation Correction" << endl;
            cout << "\t $sudo ./Robocpp InitOri" << endl << endl;
            cout << "3. Grasping Position/Orientation Correction" << endl;
            cout << "\t $sudo ./Robocpp Grasp" << endl << endl;
            cout << "4. Go to init configuration from current one" << endl;
            cout << "\t $sudo ./Robocpp init" << endl << endl;
            cout << "for 3 and 4, enable compliance with \"-c\" flag" << endl;
            cout << endl << " Additional Features: " << endl;
            cout << "@ Get encoder/ref reading for a specific joint" << endl;
            cout << "\t $sudo ./Robocpp read JOINT_NAME" << endl << endl;
            cout << "@ Get joint angle for a specific joint in a motion file w/ specific line number" << endl;
            cout << "\t $sudo ./Robocpp readfile JOINT_NAME FILENAME LINE_NUMBER" << endl << endl;
            cout << "@ Move a joint from current position to a desired position" << endl;
            cout << "\t $sudo ./Robocpp move JOINT_NAME DESIRED_POSITION" << endl << endl;
            cout << "@ Move a joint from current position to the desired position" << endl;
            cout << "\t in a specific line number in a specific file" << endl;
            cout << "\t $sudo ./Robocpp movefile JOINT_NAME FILENAME LINE_NUMBER" << endl << endl;
            cout << "@ Move a joint from current position to the reference position" << endl;
            cout << "\t $sudo ./Robocpp move2ref JOINT_NAME" << endl << endl;


        }
        else if(argc == 2){
            if (argv[1] == string("InitPos")){
                select = 1;
            }
            else if (argv[1] == string("InitOri")){
                select = 2;
            }
            // with compliance disabled
            else if (argv[1] == string("Grasp")){
                compliance = false;
                select = 3;
            }
            // with compliance disabled
            else if (argv[1] == string("init")){
                compliance = false;
                select = 100;
            }
            else{
                cout << "invalid command" << endl;
                select = 0;
            }

        }
        else if(argc == 3){
            // read the current encoder reading
            if (argv[1] == string("read")){
                // map second argument to the joint index
                INDEX_JOINT = map_joint_str2num(argv[2]);
                cout << "Index Joint: " << INDEX_JOINT << endl;

                // if valid joint name
                if(INDEX_JOINT > -100){
                    select = 101;
                }
                // if the joint name is invalid
                else{
                    cout << "invalid joint name" << endl;
                    select = 0;
                }
            }
            // move selected joint from the current position to the reference position
            else if (argv[1] == string("move2ref")){
                // map second argument to the joint index
                INDEX_JOINT = map_joint_str2num(argv[2]);
                cout << "Index Joint: " << INDEX_JOINT << endl;

                // if valid joint name
                if(INDEX_JOINT > -100){
                    select = 103;
                }
                // if the joint name is invalid
                else{
                    cout << "invalid joint name" << endl;
                    select = 0;
                }
            }
            // with compliance enabled
            else if (argv[1] == string("Grasp")){
                if(argv[2] == string("-c")){
                    compliance = true;
                    cout << "Compliance Enabled" << endl;
                    select = 3;
                }
                else{
                    cout << "invalid command" << endl;
                    exit(1);
                }
            }
            // with compliance enabled
            else if (argv[1] == string("init")){

                if(argv[2] == string("-c")){
                    compliance = true;
                    cout << "Compliance Enabled" << endl;
                    select = 100;
                }
                else{
                    cout << "invalid command" << endl;
                    exit(1);
                }
            }

            else{
                cout << "invalid command" << endl;
                select = 0;
            }

        }
        else if(argc == 4){
            if (argv[1] == string("move")){
                // map second argument to the joint index
                INDEX_JOINT = map_joint_str2num(argv[2]);
                // if valid joint name
                if(INDEX_JOINT > -100){
                    JOINT_ANGLE = str2double(argv[3]);
                    select = 102;
                }
                // if the joint name is invalid
                else{
                    cout << "invalid joint name" << endl;
                    select = 0;
                }
            }
            else{
                cout << "invalid command" << endl;
                select = 0;
            }
        }
        else if(argc ==5){
            if (argv[1] == string("readfile")){
                // map second argument to the joint index
                INDEX_JOINT = map_joint_str2num(argv[2]);
                //cout << INDEX_JOINT << endl;
                // if valid joint name
                if(INDEX_JOINT > -100 && INDEX_JOINT <100){
                    JOINT_ANGLE = FindJointAngFile(argv[3],str2int(argv[4]),INDEX_JOINT);
                    // display joint angle
                    cout << "Joint Angle: " << JOINT_ANGLE << endl;
                    //select = 102;
                }
                // ALL JOINTS
                else if(INDEX_JOINT == 100){
                    JOINT_DATA = FindJointDataFile(argv[3],str2int(argv[4]));
                    for (int i=0; i<40; i++){
                        cout << i+1 << ": " << JOINT_DATA(i) << endl;
                    }
                    //select = 104;
                }
                // if the joint name is invalid
                else{
                    cout << "invalid joint name" << endl;
                    select = 0;
                }
            }
            else if (argv[1] == string("movefile")){
                // map second argument to the joint index
                INDEX_JOINT = map_joint_str2num(argv[2]);
                // if valid joint name
                if(INDEX_JOINT > -100){
                    JOINT_ANGLE = FindJointAngFile(argv[3],str2int(argv[4]),INDEX_JOINT);
                    // display joint angle
                    cout << "Joint Angle: " << JOINT_ANGLE << endl;
                    select = 102;
                }
                else if(INDEX_JOINT == 100){
                    JOINT_DATA = FindJointDataFile(argv[3],str2int(argv[4]));
                    for (int i=0; i<40; i++){
                        cout << i << ": " << JOINT_DATA(i) << endl;
                    }
                    select = 104;
                }
                // if the joint name is invalid
                else{
                    cout << "invalid joint name" << endl;
                    select = 0;
                }
            }

            else{
                cout << "invalid command" << endl;
                select = 0;
            }
        }

        else{
            cout << "too many arguments" << endl;
        }
    }


    //select = 1;



    ////// InitPos
    /// 09-27-13 Init Foot Position Correction (DRC-Hubo)
    /// interactive
    /// added margin and slower speed
    if (select == 1){
        DRCHubo robot;
        robot_init(robot);

        vec q_data = zeros<vec>(robot.NJ,1);
        vec q_data_init = q_data;
        q_data_init(robot.J_HPY-1) = 0;
        q_data_init(robot.J_LHP-1) = -pi/7 - 0.1;
        q_data_init(robot.J_LKP-1) = pi/3.5;
        q_data_init(robot.J_LAP-1) = -pi/7;
        q_data_init(robot.J_RHP-1) = -pi/7 - 0.1;
        q_data_init(robot.J_RKP-1) = pi/3.5;
        q_data_init(robot.J_RAP-1) = -pi/7;

        //        q_data_init(robot.J_LEP-1) = -2.5*pi/4;
        //        q_data_init(robot.J_REP-1) = -2.5*pi/4;

        q_data_init(robot.J_LSR-1) = deg2rad(10);
        q_data_init(robot.J_RSR-1) = -deg2rad(10);

        vec q_init = q_data_init;
        vec q_tmp = q_data_init;


        double Kp = 100;
        double tsamp = 0.01;
        double err_thresh = 0.001;

        int ContactState = robot.LF;

        // CoM of the init pose
        mat CoM_init = CoM(robot, q_init, ContactState);

        //// generate init pose
        // from zero to init pose
        mat qdata = CosTraj_mat(30, zeros(robot.NJ,1), q_data_init);
        //cout << qdata << endl;

        // interpolation
        mat qdata1;

        // send init motion to Hubo-Ach
        qdata1 = spline_interp_mat(qdata, speed_motion);
        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);

        // Variable Declarations
        double steplength, steplength_y, stepheight, steplength_v, rotation_z;
        int Nframes_step, Nframes_com, F_swF, N_col, Nframes, cnt;
        mat T_W_ref, T_W_init, x_W_ref, T_swF_init, T_swF_ref, x_swF_ref;
        mat T_LH_ref, T_LH_init, x_LH_ref, T_RH_ref, T_RH_init, x_RH_ref, x_com_ref, com_data, com_tmp;
        vec ang_data, q_waist;
        double margin_x, margin_y, stepcollision, norm_error;
        cube Tdata_swF_tmp, Tdata_swF;
        mat x_W, x_swF, x_RH, x_LH, x_com, err_x_W, err_x_swF, err_x_RH, err_x_LH, err_x_com;
        mat J_W, J_swF, J_LH, J_RH, J_com, J_com_legs, Jacob, qd_tmp, error;
        mat x_com_des;

        int Loop = 1;
        double STEP_LENGTH_X, STEP_LENGTH_Y, STEP_HEIGHT, STEP_LENGTH_Z, ROTATION_Z;
        STEP_HEIGHT = 40;
        ROTATION_Z = 0;

        // Loop
        while(Loop){

            // initialize qdata
            qdata = q_tmp.t();

            cout << endl << endl << " Init Position Correction " << endl;
            cout << "Enter Step_length (x) [mm]: ";
            cin >> STEP_LENGTH_X;
            cout << "Enter Step_length (y) [mm]: ";
            cin >> STEP_LENGTH_Y;
            cout << "Enter Step_length (z) [mm]: ";
            cin >> STEP_LENGTH_Z;

            //// static backward staircase climbing motion generation
            // first step - left foot support
            steplength = 0.001*STEP_LENGTH_X;
            steplength_y = 0.001*STEP_LENGTH_Y;
            stepheight = 0.001*STEP_HEIGHT;
            steplength_v = 0.001*STEP_LENGTH_Z;
            rotation_z = 0.001*ROTATION_Z;

            Nframes_step = 40;
            Nframes_com = 50;

            if (STEP_LENGTH_Y <= 0){
                // contact state and swing foot
                ContactState = robot.LF;
                F_swF = robot.F_RF;
            }
            else{
                // contact state and swing foot
                ContactState = robot.RF;
                F_swF = robot.F_LF;
            }

            /// current pose of the robot
            // waist pose
            T_W_ref = FK_tree(robot, q_tmp, robot.F_HPY, ContactState);
            T_W_init = T_W_ref;
            T_W_ref(0,3) = T_W_ref(0,3);
            T_W_ref(1,3) = T_W_ref(1,3);
            T_W_ref(2,3) = T_W_ref(2,3);
            x_W_ref = tr2diff(T_W_ref);

            // swing foot pose
            T_swF_ref = FK_tree(robot, q_tmp, F_swF, ContactState);
            T_swF_init = T_swF_ref;
            T_swF_ref(0,3) = T_swF_ref(0,3);
            T_swF_ref(1,3) = T_swF_ref(1,3);
            T_swF_ref(2,3) = T_swF_ref(2,3);
            x_swF_ref = tr2diff(T_swF_ref);

            // left hand pose
            T_LH_ref = FK_tree(robot, q_tmp, robot.F_LH, ContactState);
            T_LH_init = T_LH_ref;
            T_LH_ref(0,3) = T_LH_ref(0,3);
            T_LH_ref(1,3) = T_LH_ref(1,3);
            T_LH_ref(2,3) = T_LH_ref(2,3);
            x_LH_ref = tr2diff(T_LH_ref);

            // right hand pose
            T_RH_ref = FK_tree(robot, q_tmp, robot.F_RH, ContactState);
            T_RH_init = T_RH_ref;
            T_RH_ref(0,3) = T_RH_ref(0,3);
            T_RH_ref(1,3) = T_RH_ref(1,3);
            T_RH_ref(2,3) = T_RH_ref(2,3);
            x_RH_ref = tr2diff(T_RH_ref);

            // CoM position
            x_com_ref = CoM(robot, q_tmp, ContactState);
            x_com_ref(0,0) = x_com_ref(0,0);
            x_com_ref(0,1) = x_com_ref(0,1);
            x_com_ref(0,2) = x_com_ref(0,2);

            // generate CoM trajectory
            margin_x = 0.001*50;
            margin_y = 0.001*0;

            com_data = generate_com_motion(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y);
            //com_data.print();

            // generate footstep angle data
            ang_data = join_cols(zeros<vec>(Nframes_com,1), CosTraj(Nframes_step, 0, deg2rad(rotation_z)));
            //ang_data.print();

            // generate swing foot stepping motion
            stepcollision = 0.001*0;
            N_col = 10;
            Tdata_swF_tmp = generate_stepping_motion(robot, q_tmp, ContactState, steplength, stepheight, Nframes_step, steplength_v, 1, stepcollision, steplength_y, N_col, rotation_z);

            //// pad trajectories
            // com trajectory
            com_tmp = com_data.row(com_data.n_rows-1);
            for (int i=0; i<Nframes_step; i++){
                com_data = join_cols(com_data, com_tmp);
            }
            //        com_data.print();

            Tdata_swF = zeros(4,4, Nframes_com + Nframes_step);

            // swing foot trajectory
            for (int i=0; i<Nframes_com; i++){
                Tdata_swF.slice(i) = T_swF_init;
            }

            for (int i=Nframes_com; i<Nframes_com + Nframes_step; i++){
                Tdata_swF.slice(i) = Tdata_swF_tmp.slice(i-Nframes_com);
            }

            Nframes = Nframes_com + Nframes_step;

            norm_error = 10000;
            cnt = 1;

            for (int I=0; I<Nframes; I++){
                x_com_ref = com_data.row(I).t();

                T_swF_ref = Tdata_swF.slice(I);
                x_swF_ref = tr2diff(T_swF_ref);

                norm_error = 10000;
                cnt = 1;

                while (norm_error > err_thresh){
                    x_W = tr2diff(FK_tree(robot,q_tmp,robot.F_HPY,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_W = x_W_ref - x_W;
                    err_x_W = err_x_W.rows(2,5);

                    x_swF = tr2diff(FK_tree(robot,q_tmp,F_swF,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_swF = x_swF_ref - x_swF;

                    //                x_RH = tr2diff(FK_tree(robot,q_tmp,robot.F_RH,ContactState));
                    //                err_x_RH = x_RH_ref - x_RH;

                    //                x_LH = tr2diff(FK_tree(robot,q_tmp,robot.F_LH,ContactState));
                    //                err_x_LH = x_LH_ref - x_LH;

                    x_com = CoM(robot, q_tmp, ContactState);
                    err_x_com = x_com_ref - x_com.t();

                    J_W = J_tree(robot, q_tmp, robot.F_HPY, ContactState);
                    J_W = J_W.rows(2,5);

                    J_swF = J_tree(robot, q_tmp, F_swF, ContactState);

                    //                J_RH = J_tree(robot, q_tmp, robot.F_RH, ContactState);
                    //                J_LH = J_tree(robot, q_tmp, robot.F_LH, ContactState);

                    J_com = J_com_tree(robot, q_tmp, ContactState);
                    J_com_legs = join_rows(join_rows(zeros(3,1), J_com.cols(robot.J_LHY-1,robot.J_RAR-1)), zeros(3,robot.NJ-13));

                    Jacob = join_cols(join_cols(J_com_legs.rows(0,1), J_swF), J_W);
                    error = join_cols(join_cols(err_x_com.rows(0,1), err_x_swF), err_x_W);

                    //cout << shape(Jacob) << endl;
                    //cout << shape(error) << endl;

                    qd_tmp = Kp*pinv(Jacob)*error;
                    q_tmp = q_tmp + qd_tmp*tsamp;
                    //q_tmp = joint_limit_check(robot, q_tmp);

                    cnt = cnt + 1;
                    norm_error = norm(error, 2);

                    //cout << I << " "<< cnt << " , error: " << norm_error << endl;
                    cout << I << " "<< cnt << endl;

                    // when no solution is found
                    if (cnt > 10){
                        cout << "solution is not found, try with different numbers" << endl;
                        exit(1);

                        // generate end pose
                        qdata = CosTraj_mat(60, q_tmp, zeros(robot.NJ,1));

                        // interpolation
                        qdata1 = spline_interp_mat(qdata, speed_motion);

                        // send the motion to Hubo-Ach
                        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);
                    }
                }

                qdata = join_cols(qdata,q_tmp.t());
            }

            //cout << shape(qdata) << endl;

            //// second step - right foot support
            //        steplength = -0.001*100;
            //        steplength_y = -0.001*50;
            //        stepheight = 0.001*75;
            steplength_v = 0;
            rotation_z = 0;

            Nframes_step = 40;
            Nframes_com = 70;

            if (STEP_LENGTH_Y <= 0){
                // contact state and swing foot
                ContactState = robot.RF;
                F_swF = robot.F_LF;
            }
            else{
                // contact state and swing foot
                ContactState = robot.LF;
                F_swF = robot.F_RF;
            }


            /// current pose of the robot
            // waist pose
            T_W_ref = FK_tree(robot, q_tmp, robot.F_HPY, ContactState);
            T_W_init = T_W_ref;
            T_W_ref(0,3) = T_W_ref(0,3);
            T_W_ref(1,3) = T_W_ref(1,3);
            T_W_ref(2,3) = T_W_ref(2,3);
            x_W_ref = tr2diff(T_W_ref);

            // swing foot pose
            T_swF_ref = FK_tree(robot, q_tmp, F_swF, ContactState);
            T_swF_init = T_swF_ref;
            T_swF_ref(0,3) = T_swF_ref(0,3);
            T_swF_ref(1,3) = T_swF_ref(1,3);
            T_swF_ref(2,3) = T_swF_ref(2,3);
            x_swF_ref = tr2diff(T_swF_ref);

            // left hand pose
            T_LH_ref = FK_tree(robot, q_tmp, robot.F_LH, ContactState);
            T_LH_init = T_LH_ref;
            T_LH_ref(0,3) = T_LH_ref(0,3);
            T_LH_ref(1,3) = T_LH_ref(1,3);
            T_LH_ref(2,3) = T_LH_ref(2,3);
            x_LH_ref = tr2diff(T_LH_ref);

            // right hand pose
            T_RH_ref = FK_tree(robot, q_tmp, robot.F_RH, ContactState);
            T_RH_init = T_RH_ref;
            T_RH_ref(0,3) = T_RH_ref(0,3);
            T_RH_ref(1,3) = T_RH_ref(1,3);
            T_RH_ref(2,3) = T_RH_ref(2,3);
            x_RH_ref = tr2diff(T_RH_ref);

            // CoM position
            x_com_ref = CoM(robot, q_tmp, ContactState);
            x_com_ref(0,0) = x_com_ref(0,0);
            x_com_ref(0,1) = x_com_ref(0,1);
            x_com_ref(0,2) = x_com_ref(0,2);

            // generate CoM trajectory
            //        margin_x = 0.001*0;
            //        margin_y = 0.001*0;

            com_data = generate_com_motion(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y);

            // generate footstep angle data
            ang_data = join_cols(zeros<vec>(Nframes_com,1), CosTraj(Nframes_step, 0, deg2rad(rotation_z)));
            //ang_data.print();

            // generate swing foot stepping motion
            stepcollision = 0.001*0;
            N_col = 10;
            Tdata_swF_tmp = generate_stepping_motion(robot, q_tmp, ContactState, steplength, stepheight, Nframes_step, steplength_v, 1, stepcollision, steplength_y, N_col, rotation_z);

            //// pad trajectories
            // com trajectory
            com_tmp = com_data.row(com_data.n_rows-1);
            for (int i=0; i<Nframes_step; i++){
                com_data = join_cols(com_data, com_tmp);
            }
            //        com_data.print();

            Tdata_swF = zeros(4,4, Nframes_com + Nframes_step);

            // swing foot trajectory
            for (int i=0; i<Nframes_com; i++){
                Tdata_swF.slice(i) = T_swF_init;
            }

            for (int i=Nframes_com; i<Nframes_com + Nframes_step; i++){
                Tdata_swF.slice(i) = Tdata_swF_tmp.slice(i-Nframes_com);
            }

            Nframes = Nframes_com + Nframes_step;

            for (int I=0; I<Nframes; I++){
                x_com_ref = com_data.row(I).t();

                T_swF_ref = Tdata_swF.slice(I);
                x_swF_ref = tr2diff(T_swF_ref);

                norm_error = 10000;
                cnt = 1;

                while (norm_error > err_thresh){
                    x_W = tr2diff(FK_tree(robot,q_tmp,robot.F_HPY,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_W = x_W_ref - x_W;
                    err_x_W = err_x_W.rows(2,5);

                    x_swF = tr2diff(FK_tree(robot,q_tmp,F_swF,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_swF = x_swF_ref - x_swF;

                    //                x_RH = tr2diff(FK_tree(robot,q_tmp,robot.F_RH,ContactState));
                    //                err_x_RH = x_RH_ref - x_RH;

                    //                x_LH = tr2diff(FK_tree(robot,q_tmp,robot.F_LH,ContactState));
                    //                err_x_LH = x_LH_ref - x_LH;

                    x_com = CoM(robot, q_tmp, ContactState);
                    err_x_com = x_com_ref - x_com.t();

                    J_W = J_tree(robot, q_tmp, robot.F_HPY, ContactState);
                    J_W = J_W.rows(2,5);

                    J_swF = J_tree(robot, q_tmp, F_swF, ContactState);

                    //                J_RH = J_tree(robot, q_tmp, robot.F_RH, ContactState);
                    //                J_LH = J_tree(robot, q_tmp, robot.F_LH, ContactState);

                    J_com = J_com_tree(robot, q_tmp, ContactState);
                    J_com_legs = join_rows(join_rows(zeros(3,1), J_com.cols(robot.J_LHY-1,robot.J_RAR-1)), zeros(3,robot.NJ-13));
                    //cout << shape(J_com_legs) << " " << shape(J_swF) << endl;

                    Jacob = join_cols(join_cols(J_com_legs.rows(0,1), J_swF), J_W);
                    error = join_cols(join_cols(err_x_com.rows(0,1), err_x_swF), err_x_W);

                    //cout << shape(Jacob) << endl;
                    //cout << shape(error) << endl;

                    qd_tmp = Kp*pinv(Jacob)*error;
                    q_tmp = q_tmp + qd_tmp*tsamp;
                    //q_tmp = joint_limit_check(robot, q_tmp);

                    cnt = cnt + 1;
                    norm_error = norm(error, 2);

                    //                    cout << I << " "<< cnt << " , error: " << norm_error << endl;
                    cout << I << " "<< cnt << endl;
                    // when no solution is found
                    if (cnt > 10){
                        cout << "solution is not found, try with different numbers" << endl;
                        exit(1);

                        // generate end pose
                        qdata = CosTraj_mat(60, q_tmp, zeros(robot.NJ,1));

                        // interpolation
                        qdata1 = spline_interp_mat(qdata, speed_motion);

                        // send the motion to Hubo-Ach
                        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);
                    }
                }

                qdata = join_cols(qdata,q_tmp.t());
            }

            //// third step - move CoM to the center

            Nframes_com = 40;

            if (STEP_LENGTH_Y <= 0){
                // contact state and swing foot
                ContactState = robot.RF;
                F_swF = robot.F_LF;
            }
            else{
                // contact state and swing foot
                ContactState = robot.LF;
                F_swF = robot.F_RF;
            }

            /// current pose of the robot
            // waist pose
            T_W_ref = FK_tree(robot, q_tmp, robot.F_HPY, ContactState);
            T_W_init = T_W_ref;
            T_W_ref(0,3) = T_W_ref(0,3);
            T_W_ref(1,3) = T_W_ref(1,3);
            T_W_ref(2,3) = T_W_ref(2,3);
            x_W_ref = tr2diff(T_W_ref);

            // swing foot pose
            T_swF_ref = FK_tree(robot, q_tmp, F_swF, ContactState);
            T_swF_init = T_swF_ref;
            T_swF_ref(0,3) = T_swF_ref(0,3);
            T_swF_ref(1,3) = T_swF_ref(1,3);
            T_swF_ref(2,3) = T_swF_ref(2,3);
            x_swF_ref = tr2diff(T_swF_ref);

            // left hand pose
            T_LH_ref = FK_tree(robot, q_tmp, robot.F_LH, ContactState);
            T_LH_init = T_LH_ref;
            T_LH_ref(0,3) = T_LH_ref(0,3);
            T_LH_ref(1,3) = T_LH_ref(1,3);
            T_LH_ref(2,3) = T_LH_ref(2,3);
            x_LH_ref = tr2diff(T_LH_ref);

            // right hand pose
            T_RH_ref = FK_tree(robot, q_tmp, robot.F_RH, ContactState);
            T_RH_init = T_RH_ref;
            T_RH_ref(0,3) = T_RH_ref(0,3);
            T_RH_ref(1,3) = T_RH_ref(1,3);
            T_RH_ref(2,3) = T_RH_ref(2,3);
            x_RH_ref = tr2diff(T_RH_ref);

            // CoM position
            x_com_ref = CoM(robot, q_tmp, ContactState);
            x_com_ref(0,0) = x_com_ref(0,0);
            x_com_ref(0,1) = x_com_ref(0,1);
            x_com_ref(0,2) = x_com_ref(0,2);

            // generate CoM trajectory
            //        margin_x = 0.001*0;
            //        margin_y = 0.001*30;

            x_com_des = CoM(robot, q_data_init, ContactState);
            com_data = com_motion_des(robot, q_tmp, ContactState, Nframes_com, x_com_des);

            // waist angle back to init
            q_waist = CosTraj(Nframes_com, q_tmp(robot.J_HPY-1,0),0);

            Nframes = Nframes_com;

            for (int I=0; I<Nframes; I++){
                x_com_ref = com_data.row(I).t();

                norm_error = 10000;
                cnt = 1;

                while (norm_error > err_thresh){
                    x_W = tr2diff(FK_tree(robot,q_tmp,robot.F_HPY,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_W = x_W_ref - x_W;
                    err_x_W = err_x_W.rows(2,5);

                    x_swF = tr2diff(FK_tree(robot,q_tmp,F_swF,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_swF = x_swF_ref - x_swF;

                    //                x_RH = tr2diff(FK_tree(robot,q_tmp,robot.F_RH,ContactState));
                    //                err_x_RH = x_RH_ref - x_RH;

                    //                x_LH = tr2diff(FK_tree(robot,q_tmp,robot.F_LH,ContactState));
                    //                err_x_LH = x_LH_ref - x_LH;

                    x_com = CoM(robot, q_tmp, ContactState);
                    err_x_com = x_com_ref - x_com.t();

                    J_W = J_tree(robot, q_tmp, robot.F_HPY, ContactState);
                    J_W = J_W.rows(2,5);

                    J_swF = J_tree(robot, q_tmp, F_swF, ContactState);

                    //                J_RH = J_tree(robot, q_tmp, robot.F_RH, ContactState);
                    //                J_LH = J_tree(robot, q_tmp, robot.F_LH, ContactState);

                    J_com = J_com_tree(robot, q_tmp, ContactState);
                    J_com_legs = join_rows(join_rows(zeros(3,1), J_com.cols(robot.J_LHY-1,robot.J_RAR-1)), zeros(3,robot.NJ-13));

                    Jacob = join_cols(join_cols(J_com_legs.rows(0,1), J_swF), J_W);
                    error = join_cols(join_cols(err_x_com.rows(0,1), err_x_swF), err_x_W);

                    //cout << shape(Jacob) << endl;
                    //cout << shape(error) << endl;

                    qd_tmp = Kp*pinv(Jacob)*error;
                    q_tmp = q_tmp + qd_tmp*tsamp;

                    q_tmp(robot.J_HPY-1,0) = q_waist(I);
                    //q_tmp = joint_limit_check(robot, q_tmp);

                    cnt = cnt + 1;
                    norm_error = norm(error, 2);

                    //                    cout << I << " "<< cnt << " , error: " << norm_error << endl;
                    cout << I << " "<< cnt << endl;
                    // when no solution is found
                    if (cnt > 10){
                        cout << "solution is not found, try with different numbers" << endl;
                        exit(1);

                        // generate end pose
                        qdata = CosTraj_mat(60, q_tmp, zeros(robot.NJ,1));

                        // interpolation
                        qdata1 = spline_interp_mat(qdata, speed_motion);

                        // send the motion to Hubo-Ach
                        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);
                    }
                }

                qdata = join_cols(qdata,q_tmp.t());
            }

            t = clock() - t;
            printf ("It took me %d clicks (%f seconds).\n",t,((float)t)/CLOCKS_PER_SEC);

            //            // generate end pose
            //            qdata = join_cols(qdata, CosTraj_mat(30, q_tmp, zeros(robot.NJ,1)));

            // interpolation
            qdata1 = spline_interp_mat(qdata, speed_motion);

            // send the motion to Hubo-Ach
            motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);

            cout << "do you want to continue? 1: Yes, 0: No: ";
            cin >> Loop;
        }


        // generate end pose
        qdata = CosTraj_mat(60, q_tmp, zeros(robot.NJ,1));

        // interpolation
        qdata1 = spline_interp_mat(qdata, speed_motion);

        // send the motion to Hubo-Ach
        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);


    }

    ////// InitOri
    /// 09-27-13 Init Foot Orientation Correction (DRC-Hubo)
    /// added margins and slower speed
    else if (select == 2){
        DRCHubo robot;
        robot_init(robot);

        vec q_data = zeros<vec>(robot.NJ,1);
        vec q_data_init = q_data;
        q_data_init(robot.J_HPY-1) = 0;
        q_data_init(robot.J_LHP-1) = -pi/7 - 0.1;
        q_data_init(robot.J_LKP-1) = pi/3.5;
        q_data_init(robot.J_LAP-1) = -pi/7;
        q_data_init(robot.J_RHP-1) = -pi/7 - 0.1;
        q_data_init(robot.J_RKP-1) = pi/3.5;
        q_data_init(robot.J_RAP-1) = -pi/7;

        //        q_data_init(robot.J_LEP-1) = -2.5*pi/4;
        //        q_data_init(robot.J_REP-1) = -2.5*pi/4;

        q_data_init(robot.J_LSR-1) = deg2rad(10);
        q_data_init(robot.J_RSR-1) = -deg2rad(10);

        vec q_init = q_data_init;
        vec q_tmp = q_data_init;


        double Kp = 100;
        double tsamp = 0.01;
        double err_thresh = 0.001;

        int ContactState = robot.LF;

        // CoM of the init pose
        mat CoM_init = CoM(robot, q_init, ContactState);

        //// generate init pose
        // from zero to init pose
        mat qdata = CosTraj_mat(30, zeros(robot.NJ,1), q_data_init);
        //cout << qdata << endl;

        // interpolation
        mat qdata1;

        // send init motion to Hubo-Ach
        qdata1 = spline_interp_mat(qdata, speed_motion);
        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);


        // Variable Declarations
        double steplength, steplength_y, stepheight, steplength_v, rotation_z;
        int Nframes_step, Nframes_com, F_swF, N_col, Nframes, cnt;
        mat T_W_ref, T_W_init, x_W_ref, T_swF_init, T_swF_ref, x_swF_ref;
        mat T_LH_ref, T_LH_init, x_LH_ref, T_RH_ref, T_RH_init, x_RH_ref, x_com_ref, com_data, com_tmp;
        vec ang_data, q_waist;
        double margin_x, margin_y, stepcollision, norm_error;
        cube Tdata_swF_tmp, Tdata_swF;
        mat x_W, x_swF, x_RH, x_LH, x_com, err_x_W, err_x_swF, err_x_RH, err_x_LH, err_x_com;
        mat J_W, J_swF, J_LH, J_RH, J_com, J_com_legs, Jacob, qd_tmp, error;
        mat x_com_des;

        int Loop = 1;
        double STEP_LENGTH_X, STEP_LENGTH_Y, STEP_HEIGHT, STEP_LENGTH_Z, ROTATION_Z, DISTANCE, DELTA_X, DELTA_Y;
        mat distance_tmp;
        STEP_HEIGHT = 40;
        //ROTATION_Z = 10;
        // distance between two feet

        distance_tmp = tr2transl(FK_tree(robot, zeros(robot.NJ,1), robot.F_LF, robot.RF));
        DISTANCE = distance_tmp(1,0);
        cout << "distance: " << DISTANCE << endl;
        //Sign = 1;

        // Loop
        //string input = "";
        while(Loop){

            // initialize qdata
            qdata = q_tmp.t();

            cout << endl << endl << " Init Orientation Correction " << endl;
            //cout << "Enter Step_length (x) [mm]: ";
            //cin >> STEP_LENGTH_X;
            //cout << "Enter Step_length (y) [mm]: ";
            //cin >> STEP_LENGTH_Y;
            //cout << "Enter Step_length (z) [mm]: ";
            //cin >> STEP_LENGTH_Z;
            cout << "Enter Rotation [deg]: ";
            cin >> ROTATION_Z;
//            scanf("\n%d",&ROTATION_Z);

//            cout << endl << "Rotation: " << ROTATION_Z << endl;
            STEP_LENGTH_X = 50;
            STEP_LENGTH_Y = -50;
            STEP_LENGTH_Z = 0;

            //// static backward staircase climbing motion generation
            // first step - left foot support


            steplength = 0.001*STEP_LENGTH_X;
            steplength_y = 0.001*STEP_LENGTH_Y;
            stepheight = 0.001*STEP_HEIGHT;
            steplength_v = 0.001*STEP_LENGTH_Z;
            rotation_z = deg2rad(ROTATION_Z);


            Nframes_step = 40;
            Nframes_com = 50;

            if (steplength_y <= 0){
                // contact state and swing foot
                ContactState = robot.LF;
                F_swF = robot.F_RF;
            }
            else{
                // contact state and swing foot
                ContactState = robot.RF;
                F_swF = robot.F_LF;
            }

            /// current pose of the robot
            // waist pose
            T_W_ref = FK_tree(robot, q_tmp, robot.F_HPY, ContactState);
            T_W_init = T_W_ref;
            T_W_ref(0,3) = T_W_ref(0,3);
            T_W_ref(1,3) = T_W_ref(1,3);
            T_W_ref(2,3) = T_W_ref(2,3);
            x_W_ref = tr2diff(T_W_ref);

            // swing foot pose
            T_swF_ref = FK_tree(robot, q_tmp, F_swF, ContactState);
            T_swF_init = T_swF_ref;
            T_swF_ref(0,3) = T_swF_ref(0,3);
            T_swF_ref(1,3) = T_swF_ref(1,3);
            T_swF_ref(2,3) = T_swF_ref(2,3);
            x_swF_ref = tr2diff(T_swF_ref);
            //            cout << "distance (y): " << T_swF_ref(1,3) << endl;

            // left hand pose
            T_LH_ref = FK_tree(robot, q_tmp, robot.F_LH, ContactState);
            T_LH_init = T_LH_ref;
            T_LH_ref(0,3) = T_LH_ref(0,3);
            T_LH_ref(1,3) = T_LH_ref(1,3);
            T_LH_ref(2,3) = T_LH_ref(2,3);
            x_LH_ref = tr2diff(T_LH_ref);

            // right hand pose
            T_RH_ref = FK_tree(robot, q_tmp, robot.F_RH, ContactState);
            T_RH_init = T_RH_ref;
            T_RH_ref(0,3) = T_RH_ref(0,3);
            T_RH_ref(1,3) = T_RH_ref(1,3);
            T_RH_ref(2,3) = T_RH_ref(2,3);
            x_RH_ref = tr2diff(T_RH_ref);

            // CoM position
            x_com_ref = CoM(robot, q_tmp, ContactState);
            x_com_ref(0,0) = x_com_ref(0,0);
            x_com_ref(0,1) = x_com_ref(0,1);
            x_com_ref(0,2) = x_com_ref(0,2);

            // generate CoM trajectory
            margin_x = 0.001*50;
            margin_y = 0.001*0;

            com_data = generate_com_motion(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y);
            //com_data.print();

            // generate footstep angle data
            ang_data = join_cols(zeros<vec>(Nframes_com,1), CosTraj(Nframes_step, 0, deg2rad(rotation_z)));
            //ang_data.print();

            // generate swing foot stepping motion
            stepcollision = 0.001*0;
            N_col = 10;
            Tdata_swF_tmp = generate_stepping_motion(robot, q_tmp, ContactState, steplength, stepheight, Nframes_step, steplength_v, 1, stepcollision, steplength_y, N_col, rotation_z);

            //// pad trajectories
            // com trajectory
            com_tmp = com_data.row(com_data.n_rows-1);
            for (int i=0; i<Nframes_step; i++){
                com_data = join_cols(com_data, com_tmp);
            }
            //        com_data.print();

            Tdata_swF = zeros(4,4, Nframes_com + Nframes_step);

            // swing foot trajectory
            for (int i=0; i<Nframes_com; i++){
                Tdata_swF.slice(i) = T_swF_init;
            }

            for (int i=Nframes_com; i<Nframes_com + Nframes_step; i++){
                Tdata_swF.slice(i) = Tdata_swF_tmp.slice(i-Nframes_com);
            }

            Nframes = Nframes_com + Nframes_step;

            norm_error = 10000;
            cnt = 1;

            for (int I=0; I<Nframes; I++){
                x_com_ref = com_data.row(I).t();

                T_swF_ref = Tdata_swF.slice(I);
                x_swF_ref = tr2diff(T_swF_ref);

                norm_error = 10000;
                cnt = 1;

                while (norm_error > err_thresh){
                    x_W = tr2diff(FK_tree(robot,q_tmp,robot.F_HPY,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_W = x_W_ref - x_W;
                    err_x_W = err_x_W.rows(2,5);

                    x_swF = tr2diff(FK_tree(robot,q_tmp,F_swF,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_swF = x_swF_ref - x_swF;

                    //                x_RH = tr2diff(FK_tree(robot,q_tmp,robot.F_RH,ContactState));
                    //                err_x_RH = x_RH_ref - x_RH;

                    //                x_LH = tr2diff(FK_tree(robot,q_tmp,robot.F_LH,ContactState));
                    //                err_x_LH = x_LH_ref - x_LH;

                    x_com = CoM(robot, q_tmp, ContactState);
                    err_x_com = x_com_ref - x_com.t();

                    J_W = J_tree(robot, q_tmp, robot.F_HPY, ContactState);
                    J_W = J_W.rows(2,5);

                    J_swF = J_tree(robot, q_tmp, F_swF, ContactState);

                    //                J_RH = J_tree(robot, q_tmp, robot.F_RH, ContactState);
                    //                J_LH = J_tree(robot, q_tmp, robot.F_LH, ContactState);

                    J_com = J_com_tree(robot, q_tmp, ContactState);
                    J_com_legs = join_rows(join_rows(zeros(3,1), J_com.cols(robot.J_LHY-1,robot.J_RAR-1)), zeros(3,robot.NJ-13));
                    J_W = join_rows(zeros(4,1),J_W.cols(1,robot.NJ-1)); // remove waist movement

                    Jacob = join_cols(join_cols(J_com_legs.rows(0,1), J_swF), J_W.rows(0,2));

                    // restrict the hip yaw rotation to only one leg
                    if (F_swF == robot.F_RF){
                        Jacob.col(robot.J_LHY-1) = zeros(Jacob.n_rows,1);
                    }
                    else{
                        Jacob.col(robot.J_RHY-1) = zeros(Jacob.n_rows,1);
                    }

                    error = join_cols(join_cols(err_x_com.rows(0,1), err_x_swF), err_x_W.rows(0,2));

                    //cout << shape(Jacob) << endl;
                    //cout << shape(error) << endl;

                    qd_tmp = Kp*pinv(Jacob)*error;
                    q_tmp = q_tmp + qd_tmp*tsamp;
                    //q_tmp = joint_limit_check(robot, q_tmp);

                    cnt = cnt + 1;
                    norm_error = norm(error, 2);

                    //cout << I << " "<< cnt << " , error: " << norm_error << endl;
                    cout << I << " "<< cnt << endl;

                    // when no solution is found
                    if (cnt > 10){
                        cout << "solution is not found, try with different numbers" << endl;
                        exit(1);

                        // generate end pose
                        qdata = CosTraj_mat(60, q_tmp, zeros(robot.NJ,1));

                        // interpolation
                        qdata1 = spline_interp_mat(qdata, speed_motion);

                        // send the motion to Hubo-Ach
                        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);
                    }
                }

                qdata = join_cols(qdata,q_tmp.t());
            }

            //cout << shape(qdata) << endl;

            //// second step - right foot support
            //                    stepheight = 0.001*75;

            //steplength_v = 0;
            //rotation_z = 0;

            Nframes_step = 40;
            Nframes_com = 70;

            if (ContactState == robot.RF){
                // contact state and swing foot
                ContactState = robot.LF;
                F_swF = robot.F_RF;
            }
            else{
                // contact state and swing foot
                ContactState = robot.RF;
                F_swF = robot.F_LF;
            }

            //            steplength = steplength - Sign*DISTANCE*sin(rotation_z);
            //            steplength_y = steplength_y - Sign*DISTANCE*(1-cos(rotation_z));

            /// current pose of the robot
            // waist pose
            T_W_ref = FK_tree(robot, q_tmp, robot.F_HPY, ContactState);
            T_W_init = T_W_ref;
            T_W_ref(0,3) = T_W_ref(0,3);
            T_W_ref(1,3) = T_W_ref(1,3);
            T_W_ref(2,3) = T_W_ref(2,3);
            x_W_ref = tr2diff(T_W_ref);

            // swing foot pose
            T_swF_ref = FK_tree(robot, q_tmp, F_swF, ContactState);
            T_swF_init = T_swF_ref;
            T_swF_ref(0,3) = T_swF_ref(0,3);
            T_swF_ref(1,3) = T_swF_ref(1,3);
            T_swF_ref(2,3) = T_swF_ref(2,3);
            x_swF_ref = tr2diff(T_swF_ref);

            // Calculate the travel distance to make the feet aligned
            steplength = -T_swF_ref(0,3);

            if (ContactState == robot.LF){
                steplength_y = -DISTANCE -T_swF_ref(1,3);
            }
            else{
                steplength_y = DISTANCE -T_swF_ref(1,3);
            }

            // Store the differences
            DELTA_X = steplength - 0.001*STEP_LENGTH_X;
            DELTA_Y = steplength_y - 0.001*STEP_LENGTH_Y;

            // left hand pose
            T_LH_ref = FK_tree(robot, q_tmp, robot.F_LH, ContactState);
            T_LH_init = T_LH_ref;
            T_LH_ref(0,3) = T_LH_ref(0,3);
            T_LH_ref(1,3) = T_LH_ref(1,3);
            T_LH_ref(2,3) = T_LH_ref(2,3);
            x_LH_ref = tr2diff(T_LH_ref);

            // right hand pose
            T_RH_ref = FK_tree(robot, q_tmp, robot.F_RH, ContactState);
            T_RH_init = T_RH_ref;
            T_RH_ref(0,3) = T_RH_ref(0,3);
            T_RH_ref(1,3) = T_RH_ref(1,3);
            T_RH_ref(2,3) = T_RH_ref(2,3);
            x_RH_ref = tr2diff(T_RH_ref);

            // CoM position
            x_com_ref = CoM(robot, q_tmp, ContactState);
            x_com_ref(0,0) = x_com_ref(0,0);
            x_com_ref(0,1) = x_com_ref(0,1);
            x_com_ref(0,2) = x_com_ref(0,2);

            // generate CoM trajectory
            //        margin_x = 0.001*0;
            //        margin_y = 0.001*0;

            com_data = generate_com_motion(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y);

            // generate footstep angle data
            //ang_data = join_cols(zeros<vec>(Nframes_com,1), CosTraj(Nframes_step, 0, deg2rad(rotation_z)));
            //ang_data.print();

            // generate swing foot stepping motion
            stepcollision = 0.001*0;
            N_col = 10;
            Tdata_swF_tmp = generate_stepping_motion(robot, q_tmp, ContactState, steplength, stepheight, Nframes_step, steplength_v, 1, stepcollision, steplength_y, N_col, rotation_z);

            //// pad trajectories
            // com trajectory
            com_tmp = com_data.row(com_data.n_rows-1);
            for (int i=0; i<Nframes_step; i++){
                com_data = join_cols(com_data, com_tmp);
            }
            //        com_data.print();

            Tdata_swF = zeros(4,4, Nframes_com + Nframes_step);

            // swing foot trajectory
            for (int i=0; i<Nframes_com; i++){
                Tdata_swF.slice(i) = T_swF_init;
            }

            for (int i=Nframes_com; i<Nframes_com + Nframes_step; i++){
                Tdata_swF.slice(i) = Tdata_swF_tmp.slice(i-Nframes_com);
            }

            Nframes = Nframes_com + Nframes_step;

            for (int I=0; I<Nframes; I++){
                x_com_ref = com_data.row(I).t();

                T_swF_ref = Tdata_swF.slice(I);
                x_swF_ref = tr2diff(T_swF_ref);

                norm_error = 10000;
                cnt = 1;

                while (norm_error > err_thresh){
                    x_W = tr2diff(FK_tree(robot,q_tmp,robot.F_HPY,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_W = x_W_ref - x_W;
                    err_x_W = err_x_W.rows(2,5);

                    x_swF = tr2diff(FK_tree(robot,q_tmp,F_swF,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_swF = x_swF_ref - x_swF;

                    //                x_RH = tr2diff(FK_tree(robot,q_tmp,robot.F_RH,ContactState));
                    //                err_x_RH = x_RH_ref - x_RH;

                    //                x_LH = tr2diff(FK_tree(robot,q_tmp,robot.F_LH,ContactState));
                    //                err_x_LH = x_LH_ref - x_LH;

                    x_com = CoM(robot, q_tmp, ContactState);
                    err_x_com = x_com_ref - x_com.t();

                    J_W = J_tree(robot, q_tmp, robot.F_HPY, ContactState);
                    J_W = J_W.rows(2,5);

                    J_swF = J_tree(robot, q_tmp, F_swF, ContactState);

                    //                J_RH = J_tree(robot, q_tmp, robot.F_RH, ContactState);
                    //                J_LH = J_tree(robot, q_tmp, robot.F_LH, ContactState);

                    J_com = J_com_tree(robot, q_tmp, ContactState);
                    J_com_legs = join_rows(join_rows(zeros(3,1), J_com.cols(robot.J_LHY-1,robot.J_RAR-1)), zeros(3,robot.NJ-13));
                    J_W = join_rows(zeros(4,1),J_W.cols(1,robot.NJ-1)); // remove waist movement

                    Jacob = join_cols(join_cols(J_com_legs.rows(0,1), J_swF), J_W.rows(0,2));

                    // restrict the hip yaw rotation to only one leg
                    if (F_swF == robot.F_RF){
                        Jacob.col(robot.J_RHY-1) = zeros(Jacob.n_rows,1);
                    }
                    else{
                        Jacob.col(robot.J_LHY-1) = zeros(Jacob.n_rows,1);
                    }

                    error = join_cols(join_cols(err_x_com.rows(0,1), err_x_swF), err_x_W.rows(0,2));

                    //cout << shape(Jacob) << endl;
                    //cout << shape(error) << endl;

                    qd_tmp = Kp*pinv(Jacob)*error;
                    q_tmp = q_tmp + qd_tmp*tsamp;
                    //q_tmp = joint_limit_check(robot, q_tmp);

                    cnt = cnt + 1;
                    norm_error = norm(error, 2);

                    //                    cout << I << " "<< cnt << " , error: " << norm_error << endl;
                    cout << I << " "<< cnt << endl;
                    // when no solution is found
                    if (cnt > 10){
                        cout << "solution is not found, try with different numbers" << endl;
                        exit(1);

                        // generate end pose
                        qdata = CosTraj_mat(60, q_tmp, zeros(robot.NJ,1));

                        // interpolation
                        qdata1 = spline_interp_mat(qdata, speed_motion);

                        // send the motion to Hubo-Ach
                        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);
                    }
                }

                qdata = join_cols(qdata,q_tmp.t());
            }

            //// third step - move CoM to the center

            Nframes_com = 40;

            if (STEP_LENGTH_Y <= 0){
                // contact state and swing foot
                ContactState = robot.RF;
                F_swF = robot.F_LF;
            }
            else{
                // contact state and swing foot
                ContactState = robot.LF;
                F_swF = robot.F_RF;
            }

            /// current pose of the robot
            // waist pose
            T_W_ref = FK_tree(robot, q_tmp, robot.F_HPY, ContactState);
            T_W_init = T_W_ref;
            T_W_ref(0,3) = T_W_ref(0,3);
            T_W_ref(1,3) = T_W_ref(1,3);
            T_W_ref(2,3) = T_W_ref(2,3);
            x_W_ref = tr2diff(T_W_ref);

            // swing foot pose
            T_swF_ref = FK_tree(robot, q_tmp, F_swF, ContactState);
            T_swF_init = T_swF_ref;
            T_swF_ref(0,3) = T_swF_ref(0,3);
            T_swF_ref(1,3) = T_swF_ref(1,3);
            T_swF_ref(2,3) = T_swF_ref(2,3);
            x_swF_ref = tr2diff(T_swF_ref);

            // left hand pose
            T_LH_ref = FK_tree(robot, q_tmp, robot.F_LH, ContactState);
            T_LH_init = T_LH_ref;
            T_LH_ref(0,3) = T_LH_ref(0,3);
            T_LH_ref(1,3) = T_LH_ref(1,3);
            T_LH_ref(2,3) = T_LH_ref(2,3);
            x_LH_ref = tr2diff(T_LH_ref);

            // right hand pose
            T_RH_ref = FK_tree(robot, q_tmp, robot.F_RH, ContactState);
            T_RH_init = T_RH_ref;
            T_RH_ref(0,3) = T_RH_ref(0,3);
            T_RH_ref(1,3) = T_RH_ref(1,3);
            T_RH_ref(2,3) = T_RH_ref(2,3);
            x_RH_ref = tr2diff(T_RH_ref);

            // CoM position
            x_com_ref = CoM(robot, q_tmp, ContactState);
            x_com_ref(0,0) = x_com_ref(0,0);
            x_com_ref(0,1) = x_com_ref(0,1);
            x_com_ref(0,2) = x_com_ref(0,2);

            // generate CoM trajectory
            //        margin_x = 0.001*0;
            //        margin_y = 0.001*30;

            x_com_des = CoM(robot, q_data_init, ContactState);
            com_data = com_motion_des(robot, q_tmp, ContactState, Nframes_com, x_com_des);

            // waist angle back to init
            q_waist = CosTraj(Nframes_com, q_tmp(robot.J_HPY-1,0),0);

            Nframes = Nframes_com;

            for (int I=0; I<Nframes; I++){
                x_com_ref = com_data.row(I).t();

                norm_error = 10000;
                cnt = 1;

                while (norm_error > err_thresh){
                    x_W = tr2diff(FK_tree(robot,q_tmp,robot.F_HPY,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_W = x_W_ref - x_W;
                    err_x_W = err_x_W.rows(2,5);

                    x_swF = tr2diff(FK_tree(robot,q_tmp,F_swF,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_swF = x_swF_ref - x_swF;

                    //                x_RH = tr2diff(FK_tree(robot,q_tmp,robot.F_RH,ContactState));
                    //                err_x_RH = x_RH_ref - x_RH;

                    //                x_LH = tr2diff(FK_tree(robot,q_tmp,robot.F_LH,ContactState));
                    //                err_x_LH = x_LH_ref - x_LH;

                    x_com = CoM(robot, q_tmp, ContactState);
                    err_x_com = x_com_ref - x_com.t();

                    J_W = J_tree(robot, q_tmp, robot.F_HPY, ContactState);
                    J_W = J_W.rows(2,5);

                    J_swF = J_tree(robot, q_tmp, F_swF, ContactState);

                    //                J_RH = J_tree(robot, q_tmp, robot.F_RH, ContactState);
                    //                J_LH = J_tree(robot, q_tmp, robot.F_LH, ContactState);

                    J_com = J_com_tree(robot, q_tmp, ContactState);
                    J_com_legs = join_rows(join_rows(zeros(3,1), J_com.cols(robot.J_LHY-1,robot.J_RAR-1)), zeros(3,robot.NJ-13));
                    J_W = join_rows(zeros(4,1),J_W.cols(1,robot.NJ-1)); // remove waist movement

                    Jacob = join_cols(join_cols(J_com_legs.rows(0,1), J_swF), J_W.rows(0,2));
                    error = join_cols(join_cols(err_x_com.rows(0,1), err_x_swF), err_x_W.rows(0,2));

                    //cout << shape(Jacob) << endl;
                    //cout << shape(error) << endl;

                    qd_tmp = Kp*pinv(Jacob)*error;
                    q_tmp = q_tmp + qd_tmp*tsamp;

                    q_tmp(robot.J_HPY-1,0) = q_waist(I);
                    //q_tmp = joint_limit_check(robot, q_tmp);

                    cnt = cnt + 1;
                    norm_error = norm(error, 2);

                    //                    cout << I << " "<< cnt << " , error: " << norm_error << endl;
                    cout << I << " "<< cnt << endl;
                    // when no solution is found
                    if (cnt > 10){
                        cout << "solution is not found, try with different numbers" << endl;
                        exit(1);

                        // generate end pose
                        qdata = CosTraj_mat(60, q_tmp, zeros(robot.NJ,1));

                        // interpolation
                        qdata1 = spline_interp_mat(qdata, speed_motion);

                        // send the motion to Hubo-Ach
                        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);
                    }
                }

                qdata = join_cols(qdata,q_tmp.t());
            }

            t = clock() - t;
            printf ("It took me %d clicks (%f seconds).\n",t,((float)t)/CLOCKS_PER_SEC);

            //            // generate end pose
            //            qdata = join_cols(qdata, CosTraj_mat(30, q_tmp, zeros(robot.NJ,1)));

            // interpolation
            //qdata1 = spline_interp_mat(qdata, 5);

            // send the motion to Hubo-Ach
            //motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);

            ///////////////////////////////////////////////////////////////////////
            /// Compensation (position by change of orientation)
            ///////////////////////////////////////////////////////////////////////

            //// static backward staircase climbing motion generation
            // first step - left foot support

            //STEP_LENGTH_X = 50;
            //STEP_LENGTH_Y = -50;
            //            steplength = -0.001*STEP_LENGTH_X + DISTANCE/2*sin(rotation_z);
            //            steplength_y = -0.001*STEP_LENGTH_Y + DISTANCE/2*(1 - cos(rotation_z));
            ////            steplength =  DISTANCE/2*sin(rotation_z);
            //            steplength_y =  DISTANCE/2*(1 - cos(rotation_z));
            steplength = - DELTA_X/2 - 0.001*STEP_LENGTH_X;
            steplength_y = - DELTA_Y/2 - 0.001*STEP_LENGTH_Y;
            rotation_z = 0;

            //stepheight = 0.001*STEP_HEIGHT;
            //steplength_v = 0.001*STEP_LENGTH_Z;
            //rotation_z = deg2rad(ROTATION_Z);


            Nframes_step = 40;
            Nframes_com = 50;

            if (steplength_y <= 0){
                // contact state and swing foot
                ContactState = robot.LF;
                F_swF = robot.F_RF;
            }
            else{
                // contact state and swing foot
                ContactState = robot.RF;
                F_swF = robot.F_LF;
            }

            /// current pose of the robot
            // waist pose
            T_W_ref = FK_tree(robot, q_tmp, robot.F_HPY, ContactState);
            T_W_init = T_W_ref;
            T_W_ref(0,3) = T_W_ref(0,3);
            T_W_ref(1,3) = T_W_ref(1,3);
            T_W_ref(2,3) = T_W_ref(2,3);
            x_W_ref = tr2diff(T_W_ref);

            // swing foot pose
            T_swF_ref = FK_tree(robot, q_tmp, F_swF, ContactState);
            T_swF_init = T_swF_ref;
            T_swF_ref(0,3) = T_swF_ref(0,3);
            T_swF_ref(1,3) = T_swF_ref(1,3);
            T_swF_ref(2,3) = T_swF_ref(2,3);
            x_swF_ref = tr2diff(T_swF_ref);
            cout << "distance (y): " << T_swF_ref.col(3) << endl;
            //cin >> DISTANCE;

            // left hand pose
            T_LH_ref = FK_tree(robot, q_tmp, robot.F_LH, ContactState);
            T_LH_init = T_LH_ref;
            T_LH_ref(0,3) = T_LH_ref(0,3);
            T_LH_ref(1,3) = T_LH_ref(1,3);
            T_LH_ref(2,3) = T_LH_ref(2,3);
            x_LH_ref = tr2diff(T_LH_ref);

            // right hand pose
            T_RH_ref = FK_tree(robot, q_tmp, robot.F_RH, ContactState);
            T_RH_init = T_RH_ref;
            T_RH_ref(0,3) = T_RH_ref(0,3);
            T_RH_ref(1,3) = T_RH_ref(1,3);
            T_RH_ref(2,3) = T_RH_ref(2,3);
            x_RH_ref = tr2diff(T_RH_ref);

            // CoM position
            x_com_ref = CoM(robot, q_tmp, ContactState);
            x_com_ref(0,0) = x_com_ref(0,0);
            x_com_ref(0,1) = x_com_ref(0,1);
            x_com_ref(0,2) = x_com_ref(0,2);

            // generate CoM trajectory
            //            margin_x = 0.001*0;
            //            margin_y = 0.001*0;

            com_data = generate_com_motion(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y);
            //com_data.print();

            // generate footstep angle data
            ang_data = join_cols(zeros<vec>(Nframes_com,1), CosTraj(Nframes_step, 0, deg2rad(rotation_z)));
            //ang_data.print();

            // generate swing foot stepping motion
            stepcollision = 0.001*0;
            N_col = 10;
            Tdata_swF_tmp = generate_stepping_motion(robot, q_tmp, ContactState, steplength, stepheight, Nframes_step, steplength_v, 1, stepcollision, steplength_y, N_col, 0);

            //// pad trajectories
            // com trajectory
            com_tmp = com_data.row(com_data.n_rows-1);
            for (int i=0; i<Nframes_step; i++){
                com_data = join_cols(com_data, com_tmp);
            }
            //        com_data.print();

            Tdata_swF = zeros(4,4, Nframes_com + Nframes_step);

            // swing foot trajectory
            for (int i=0; i<Nframes_com; i++){
                Tdata_swF.slice(i) = T_swF_init;
            }

            for (int i=Nframes_com; i<Nframes_com + Nframes_step; i++){
                Tdata_swF.slice(i) = Tdata_swF_tmp.slice(i-Nframes_com);
            }

            Nframes = Nframes_com + Nframes_step;

            norm_error = 10000;
            cnt = 1;

            for (int I=0; I<Nframes; I++){
                x_com_ref = com_data.row(I).t();

                T_swF_ref = Tdata_swF.slice(I);
                x_swF_ref = tr2diff(T_swF_ref);

                norm_error = 10000;
                cnt = 1;

                while (norm_error > err_thresh){
                    x_W = tr2diff(FK_tree(robot,q_tmp,robot.F_HPY,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_W = x_W_ref - x_W;
                    err_x_W = err_x_W.rows(2,5);

                    x_swF = tr2diff(FK_tree(robot,q_tmp,F_swF,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_swF = x_swF_ref - x_swF;

                    //                x_RH = tr2diff(FK_tree(robot,q_tmp,robot.F_RH,ContactState));
                    //                err_x_RH = x_RH_ref - x_RH;

                    //                x_LH = tr2diff(FK_tree(robot,q_tmp,robot.F_LH,ContactState));
                    //                err_x_LH = x_LH_ref - x_LH;

                    x_com = CoM(robot, q_tmp, ContactState);
                    err_x_com = x_com_ref - x_com.t();

                    J_W = J_tree(robot, q_tmp, robot.F_HPY, ContactState);
                    J_W = J_W.rows(2,5);

                    J_swF = J_tree(robot, q_tmp, F_swF, ContactState);

                    //                J_RH = J_tree(robot, q_tmp, robot.F_RH, ContactState);
                    //                J_LH = J_tree(robot, q_tmp, robot.F_LH, ContactState);

                    J_com = J_com_tree(robot, q_tmp, ContactState);
                    J_com_legs = join_rows(join_rows(zeros(3,1), J_com.cols(robot.J_LHY-1,robot.J_RAR-1)), zeros(3,robot.NJ-13));
                    J_W = join_rows(zeros(4,1),J_W.cols(1,robot.NJ-1)); // remove waist movement

                    Jacob = join_cols(join_cols(J_com_legs.rows(0,1), J_swF), J_W.rows(0,2));
                    error = join_cols(join_cols(err_x_com.rows(0,1), err_x_swF), err_x_W.rows(0,2));

                    //cout << shape(Jacob) << endl;
                    //cout << shape(error) << endl;

                    qd_tmp = Kp*pinv(Jacob)*error;
                    q_tmp = q_tmp + qd_tmp*tsamp;
                    //q_tmp = joint_limit_check(robot, q_tmp);

                    cnt = cnt + 1;
                    norm_error = norm(error, 2);

                    //cout << I << " "<< cnt << " , error: " << norm_error << endl;
                    cout << I << " "<< cnt << endl;

                    // when no solution is found
                    if (cnt > 10){
                        cout << "solution is not found, try with different numbers" << endl;
                        exit(1);

                        // generate end pose
                        qdata = CosTraj_mat(60, q_tmp, zeros(robot.NJ,1));

                        // interpolation
                        qdata1 = spline_interp_mat(qdata, speed_motion);

                        // send the motion to Hubo-Ach
                        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);
                    }
                }

                qdata = join_cols(qdata,q_tmp.t());
            }

            //cout << shape(qdata) << endl;

            //// second step - right foot support
            //                    stepheight = 0.001*75;

            //steplength_v = 0;
            //rotation_z = 0;

            Nframes_step = 40;
            Nframes_com = 70;

            //            if (STEP_LENGTH_Y <= 0){
            //                // contact state and swing foot
            //                ContactState = robot.RF;
            //                F_swF = robot.F_LF;
            //                Sign = 1;
            //            }
            //            else{
            //                // contact state and swing foot
            //                ContactState = robot.LF;
            //                F_swF = robot.F_RF;
            //                Sign = -1;
            //            }
            if (ContactState == robot.LF){
                ContactState = robot.RF;
                F_swF = robot.F_LF;
            }
            else{
                ContactState = robot.LF;
                F_swF = robot.F_RF;
            }

            //steplength = steplength - Sign*DISTANCE*sin(rotation_z);
            //steplength_y = steplength_y - Sign*DISTANCE*(1-cos(rotation_z));


            /// current pose of the robot
            // waist pose
            T_W_ref = FK_tree(robot, q_tmp, robot.F_HPY, ContactState);
            T_W_init = T_W_ref;
            T_W_ref(0,3) = T_W_ref(0,3);
            T_W_ref(1,3) = T_W_ref(1,3);
            T_W_ref(2,3) = T_W_ref(2,3);
            x_W_ref = tr2diff(T_W_ref);

            // swing foot pose
            T_swF_ref = FK_tree(robot, q_tmp, F_swF, ContactState);
            T_swF_init = T_swF_ref;
            T_swF_ref(0,3) = T_swF_ref(0,3);
            T_swF_ref(1,3) = T_swF_ref(1,3);
            T_swF_ref(2,3) = T_swF_ref(2,3);
            x_swF_ref = tr2diff(T_swF_ref);

            // left hand pose
            T_LH_ref = FK_tree(robot, q_tmp, robot.F_LH, ContactState);
            T_LH_init = T_LH_ref;
            T_LH_ref(0,3) = T_LH_ref(0,3);
            T_LH_ref(1,3) = T_LH_ref(1,3);
            T_LH_ref(2,3) = T_LH_ref(2,3);
            x_LH_ref = tr2diff(T_LH_ref);

            // right hand pose
            T_RH_ref = FK_tree(robot, q_tmp, robot.F_RH, ContactState);
            T_RH_init = T_RH_ref;
            T_RH_ref(0,3) = T_RH_ref(0,3);
            T_RH_ref(1,3) = T_RH_ref(1,3);
            T_RH_ref(2,3) = T_RH_ref(2,3);
            x_RH_ref = tr2diff(T_RH_ref);

            // CoM position
            x_com_ref = CoM(robot, q_tmp, ContactState);
            x_com_ref(0,0) = x_com_ref(0,0);
            x_com_ref(0,1) = x_com_ref(0,1);
            x_com_ref(0,2) = x_com_ref(0,2);

            // generate CoM trajectory
            //        margin_x = 0.001*0;
            //        margin_y = 0.001*0;

            com_data = generate_com_motion(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y);

            // generate footstep angle data
            //ang_data = join_cols(zeros<vec>(Nframes_com,1), CosTraj(Nframes_step, 0, deg2rad(rotation_z)));
            //ang_data.print();

            // generate swing foot stepping motion
            stepcollision = 0.001*0;
            N_col = 10;
            Tdata_swF_tmp = generate_stepping_motion(robot, q_tmp, ContactState, steplength, stepheight, Nframes_step, steplength_v, 1, stepcollision, steplength_y, N_col, 0);

            //// pad trajectories
            // com trajectory
            com_tmp = com_data.row(com_data.n_rows-1);
            for (int i=0; i<Nframes_step; i++){
                com_data = join_cols(com_data, com_tmp);
            }
            //        com_data.print();

            Tdata_swF = zeros(4,4, Nframes_com + Nframes_step);

            // swing foot trajectory
            for (int i=0; i<Nframes_com; i++){
                Tdata_swF.slice(i) = T_swF_init;
            }

            for (int i=Nframes_com; i<Nframes_com + Nframes_step; i++){
                Tdata_swF.slice(i) = Tdata_swF_tmp.slice(i-Nframes_com);
            }

            Nframes = Nframes_com + Nframes_step;

            for (int I=0; I<Nframes; I++){
                x_com_ref = com_data.row(I).t();

                T_swF_ref = Tdata_swF.slice(I);
                x_swF_ref = tr2diff(T_swF_ref);

                norm_error = 10000;
                cnt = 1;

                while (norm_error > err_thresh){
                    x_W = tr2diff(FK_tree(robot,q_tmp,robot.F_HPY,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_W = x_W_ref - x_W;
                    err_x_W = err_x_W.rows(2,5);

                    x_swF = tr2diff(FK_tree(robot,q_tmp,F_swF,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_swF = x_swF_ref - x_swF;

                    //                x_RH = tr2diff(FK_tree(robot,q_tmp,robot.F_RH,ContactState));
                    //                err_x_RH = x_RH_ref - x_RH;

                    //                x_LH = tr2diff(FK_tree(robot,q_tmp,robot.F_LH,ContactState));
                    //                err_x_LH = x_LH_ref - x_LH;

                    x_com = CoM(robot, q_tmp, ContactState);
                    err_x_com = x_com_ref - x_com.t();

                    J_W = J_tree(robot, q_tmp, robot.F_HPY, ContactState);
                    J_W = J_W.rows(2,5);

                    J_swF = J_tree(robot, q_tmp, F_swF, ContactState);

                    //                J_RH = J_tree(robot, q_tmp, robot.F_RH, ContactState);
                    //                J_LH = J_tree(robot, q_tmp, robot.F_LH, ContactState);

                    J_com = J_com_tree(robot, q_tmp, ContactState);
                    J_com_legs = join_rows(join_rows(zeros(3,1), J_com.cols(robot.J_LHY-1,robot.J_RAR-1)), zeros(3,robot.NJ-13));
                    J_W = join_rows(zeros(4,1),J_W.cols(1,robot.NJ-1)); // remove waist movement

                    Jacob = join_cols(join_cols(J_com_legs.rows(0,1), J_swF), J_W.rows(0,2));
                    error = join_cols(join_cols(err_x_com.rows(0,1), err_x_swF), err_x_W.rows(0,2));

                    //cout << shape(Jacob) << endl;
                    //cout << shape(error) << endl;

                    qd_tmp = Kp*pinv(Jacob)*error;
                    q_tmp = q_tmp + qd_tmp*tsamp;
                    //q_tmp = joint_limit_check(robot, q_tmp);

                    cnt = cnt + 1;
                    norm_error = norm(error, 2);

                    //                    cout << I << " "<< cnt << " , error: " << norm_error << endl;
                    cout << I << " "<< cnt << endl;
                    // when no solution is found
                    if (cnt > 10){
                        cout << "solution is not found, try with different numbers" << endl;
                        exit(1);

                        // generate end pose
                        qdata = CosTraj_mat(60, q_tmp, zeros(robot.NJ,1));

                        // interpolation
                        qdata1 = spline_interp_mat(qdata, speed_motion);

                        // send the motion to Hubo-Ach
                        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);
                    }
                }

                qdata = join_cols(qdata,q_tmp.t());
            }

            //// third step - move CoM to the center

            Nframes_com = 40;

            if (STEP_LENGTH_Y <= 0){
                // contact state and swing foot
                ContactState = robot.RF;
                F_swF = robot.F_LF;
            }
            else{
                // contact state and swing foot
                ContactState = robot.LF;
                F_swF = robot.F_RF;
            }

            /// current pose of the robot
            // waist pose
            T_W_ref = FK_tree(robot, q_tmp, robot.F_HPY, ContactState);
            T_W_init = T_W_ref;
            T_W_ref(0,3) = T_W_ref(0,3);
            T_W_ref(1,3) = T_W_ref(1,3);
            T_W_ref(2,3) = T_W_ref(2,3);
            x_W_ref = tr2diff(T_W_ref);

            // swing foot pose
            T_swF_ref = FK_tree(robot, q_tmp, F_swF, ContactState);
            T_swF_init = T_swF_ref;
            T_swF_ref(0,3) = T_swF_ref(0,3);
            T_swF_ref(1,3) = T_swF_ref(1,3);
            T_swF_ref(2,3) = T_swF_ref(2,3);
            x_swF_ref = tr2diff(T_swF_ref);

            // left hand pose
            T_LH_ref = FK_tree(robot, q_tmp, robot.F_LH, ContactState);
            T_LH_init = T_LH_ref;
            T_LH_ref(0,3) = T_LH_ref(0,3);
            T_LH_ref(1,3) = T_LH_ref(1,3);
            T_LH_ref(2,3) = T_LH_ref(2,3);
            x_LH_ref = tr2diff(T_LH_ref);

            // right hand pose
            T_RH_ref = FK_tree(robot, q_tmp, robot.F_RH, ContactState);
            T_RH_init = T_RH_ref;
            T_RH_ref(0,3) = T_RH_ref(0,3);
            T_RH_ref(1,3) = T_RH_ref(1,3);
            T_RH_ref(2,3) = T_RH_ref(2,3);
            x_RH_ref = tr2diff(T_RH_ref);

            // CoM position
            x_com_ref = CoM(robot, q_tmp, ContactState);
            x_com_ref(0,0) = x_com_ref(0,0);
            x_com_ref(0,1) = x_com_ref(0,1);
            x_com_ref(0,2) = x_com_ref(0,2);

            // generate CoM trajectory
            //        margin_x = 0.001*0;
            //        margin_y = 0.001*30;

            x_com_des = CoM(robot, q_data_init, ContactState);
            com_data = com_motion_des(robot, q_tmp, ContactState, Nframes_com, x_com_des);

            // waist angle back to init
            q_waist = CosTraj(Nframes_com, q_tmp(robot.J_HPY-1,0),0);

            Nframes = Nframes_com;

            for (int I=0; I<Nframes; I++){
                x_com_ref = com_data.row(I).t();

                norm_error = 10000;
                cnt = 1;

                while (norm_error > err_thresh){
                    x_W = tr2diff(FK_tree(robot,q_tmp,robot.F_HPY,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_W = x_W_ref - x_W;
                    err_x_W = err_x_W.rows(2,5);

                    x_swF = tr2diff(FK_tree(robot,q_tmp,F_swF,ContactState));
                    //cout << "x_RH" << x_RH << endl;
                    err_x_swF = x_swF_ref - x_swF;

                    //                x_RH = tr2diff(FK_tree(robot,q_tmp,robot.F_RH,ContactState));
                    //                err_x_RH = x_RH_ref - x_RH;

                    //                x_LH = tr2diff(FK_tree(robot,q_tmp,robot.F_LH,ContactState));
                    //                err_x_LH = x_LH_ref - x_LH;

                    x_com = CoM(robot, q_tmp, ContactState);
                    err_x_com = x_com_ref - x_com.t();

                    J_W = J_tree(robot, q_tmp, robot.F_HPY, ContactState);
                    J_W = J_W.rows(2,5);

                    J_swF = J_tree(robot, q_tmp, F_swF, ContactState);

                    //                J_RH = J_tree(robot, q_tmp, robot.F_RH, ContactState);
                    //                J_LH = J_tree(robot, q_tmp, robot.F_LH, ContactState);

                    J_com = J_com_tree(robot, q_tmp, ContactState);
                    J_com_legs = join_rows(join_rows(zeros(3,1), J_com.cols(robot.J_LHY-1,robot.J_RAR-1)), zeros(3,robot.NJ-13));
                    J_W = join_rows(zeros(4,1),J_W.cols(1,robot.NJ-1)); // remove waist movement

                    Jacob = join_cols(join_cols(J_com_legs.rows(0,1), J_swF), J_W.rows(0,2));
                    error = join_cols(join_cols(err_x_com.rows(0,1), err_x_swF), err_x_W.rows(0,2));

                    //cout << shape(Jacob) << endl;
                    //cout << shape(error) << endl;

                    qd_tmp = Kp*pinv(Jacob)*error;
                    q_tmp = q_tmp + qd_tmp*tsamp;

                    q_tmp(robot.J_HPY-1,0) = q_waist(I);
                    //q_tmp = joint_limit_check(robot, q_tmp);

                    cnt = cnt + 1;
                    norm_error = norm(error, 2);

                    //                    cout << I << " "<< cnt << " , error: " << norm_error << endl;
                    cout << I << " "<< cnt << endl;
                    // when no solution is found
                    if (cnt > 10){
                        cout << "solution is not found, try with different numbers" << endl;
                        exit(1);

                        // generate end pose
                        qdata = CosTraj_mat(60, q_tmp, zeros(robot.NJ,1));

                        // interpolation
                        qdata1 = spline_interp_mat(qdata, speed_motion);

                        // send the motion to Hubo-Ach
                        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);
                    }
                }

                qdata = join_cols(qdata,q_tmp.t());
            }

            t = clock() - t;
            printf ("It took me %d clicks (%f seconds).\n",t,((float)t)/CLOCKS_PER_SEC);

            //            // generate end pose
            //            qdata = join_cols(qdata, CosTraj_mat(30, q_tmp, zeros(robot.NJ,1)));


            //////////////////////////////////////////////////////////////////////////////
            /// End of Compensation
            //////////////////////////////////////////////////////////////////////////////

            //// Interpolation + send the motion
            // interpolation
            qdata1 = spline_interp_mat(qdata, speed_motion);

            // send the motion to Hubo-Ach
            motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);

            // Loop question
            cout << "do you want to continue? 1: Yes, 0: No: ";
            cin >> Loop;
        }


        // generate end pose
        qdata = CosTraj_mat(60, q_tmp, zeros(robot.NJ,1));

        // interpolation
        qdata1 = spline_interp_mat(qdata, speed_motion);

        // send the motion to Hubo-Ach
        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);


    }

    ////// Grasp
    /// 09-27-13 Hand Position/Orientation Modification (DRC-Hubo)
    // interactive - both arms
    // orientation constraint
    // read from current configuration
    else if (select == 3){
        DRCHubo robot;
        robot_init(robot);

        // Read Current Hubo Configuration
        vec q_tmp = ReadAchJoint(robot);
        cout << "current configuration: " << endl;
        cout << q_tmp.t() << endl << endl;
        vec q_init = q_tmp;

        mat qdata, qdata1;

        int cnt_loop = 1;
        mat q_sol_data = q_init.t();

        // Variables Declaration
        double Kp = 100;
        double tsamp = 0.01;
        double err_thresh = 0.001;

        int ContactState;
        double x_input_R = 0.0, y_input_R = 0.0, z_input_R = 0.0;
        double x_input_L = 0.0, y_input_L = 0.0, z_input_L = 0.0;
        double X_offset, Y_offset, Z_offset;
        int axis_R = 1, axis_L = 0, rotation_L = 0, rotation_R = 0;
        double ang_R = 0, ang_L = 0;
        mat axis_rotation_R, axis_rotation_L;

        mat T_RH_ref, T_RH_init, T_LH_ref, T_LH_init, R_RH_ref, R_LH_ref;
        mat q_tmp_init;

        double norm_error = 10000;
        int cnt = 1, flag_sol = 1, index_tmp;

        mat x_RH, x_LH, x_RH_ref, x_LH_ref, err_x_RH, err_x_LH, J_RH, J_LH, Jacob, error, qd_tmp, q_sol;

        //bool compliance = false;

        // End -- Variables Declaration

        int Loop = 1;
        while (Loop == 1){
            ContactState = robot.BODY;

            // read from current configuration
            q_tmp = ReadAchJoint(robot);

            cout << endl << endl << "Grasping position/orientation correction" << endl;

            // Get Position Offset
            cout << endl << "LEFT HAND position: " << endl;
            cout << "Enter x displacement [mm]: ";
            cin >> x_input_L;
            cout << "Enter y displacement [mm]: ";
            cin >> y_input_L;
            cout << "Enter z displacement [mm]: ";
            cin >> z_input_L;
            // Get Rotation Offset
            cout << "Do you want to change rotation? (Yes: 1, No: 0): ";
            cin >> rotation_L;
            if (rotation_L == 1){
                cout << "Enter rotation axis (x:1,y:2,z:3): ";
                cin >> axis_L;
                cout << "Enter rotation amount (deg): ";
                cin >> ang_L;
            }
            else{
                axis_L = 1;
                ang_L = 0;
            }

            X_offset = 0.001*x_input_L;
            Y_offset = 0.001*y_input_L;
            Z_offset = 0.001*z_input_L;

            if (axis_L == 1){
                axis_rotation_L << 1 << 0 << 0 << endr;
            }
            else if(axis_L == 2){
                axis_rotation_L << 0 << 1 << 0 << endr;
            }
            else if(axis_L == 3){
                axis_rotation_L << 0 << 0 << 1 << endr;
            }
            else{
                axis_rotation_L << 1 << 0 << 0 << endr;
            }

            // left hand pose
            T_LH_ref = FK_tree(robot, q_tmp, robot.F_LH, ContactState);
            T_LH_init = T_LH_ref;
            R_LH_ref = t2r(T_LH_ref)*angvec2r(deg2rad(ang_L), axis_rotation_L);
            T_LH_ref(0,3) = T_LH_ref(0,3) + X_offset;
            T_LH_ref(1,3) = T_LH_ref(1,3) + Y_offset;
            T_LH_ref(2,3) = T_LH_ref(2,3) + Z_offset;
            T_LH_ref(span(0,2),span(0,2)) = R_LH_ref;
            x_LH_ref = tr2diff(T_LH_ref);
            //            x_LH_ref = x_LH_ref.rows(0,2);

            // Get Position Offset
            cout << endl << "RIGHT HAND position: " << endl;
            cout << "Enter x displacement [mm]: ";
            cin >> x_input_R;
            cout << "Enter y displacement [mm]: ";
            cin >> y_input_R;
            cout << "Enter z displacement [mm]: ";
            cin >> z_input_R;
            // Get Rotation Offset
            cout << "Do you want to change rotation? (Yes: 1, No: 0): ";
            cin >> rotation_R;
            if (rotation_R == 1){
                cout << "Enter rotation axis (x:1,y:2,z:3): ";
                cin >> axis_R;
                cout << "Enter rotation amount (deg): ";
                cin >> ang_R;
            }
            else{
                axis_R = 1;
                ang_R = 0;
            }

            X_offset = 0.001*x_input_R;
            Y_offset = 0.001*y_input_R;
            Z_offset = 0.001*z_input_R;

            if (axis_R == 1){
                axis_rotation_R << 1 << 0 << 0 << endr;
            }
            else if(axis_R == 2){
                axis_rotation_R << 0 << 1 << 0 << endr;
            }
            else if(axis_R == 3){
                axis_rotation_R << 0 << 0 << 1 << endr;
            }
            else{
                axis_rotation_R << 1 << 0 << 0 << endr;
            }

            // right hand pose
            T_RH_ref = FK_tree(robot, q_tmp, robot.F_RH, ContactState);
            T_RH_init = T_RH_ref;
            R_RH_ref = t2r(T_RH_ref)*angvec2r(deg2rad(ang_R), axis_rotation_R);
            T_RH_ref(0,3) = T_RH_ref(0,3) + X_offset;
            T_RH_ref(1,3) = T_RH_ref(1,3) + Y_offset;
            T_RH_ref(2,3) = T_RH_ref(2,3) + Z_offset;
            T_RH_ref(span(0,2),span(0,2)) = R_RH_ref;
            x_RH_ref = tr2diff(T_RH_ref);
            //            x_RH_ref = x_RH_ref.rows(0,2);


            q_tmp_init = q_tmp;
            norm_error = 10000;
            cnt = 1;
            while (norm_error > err_thresh){
                x_RH = tr2diff(FK_tree(robot,q_tmp,robot.F_RH,ContactState));
                //                x_RH = x_RH.rows(0,2);
                err_x_RH = x_RH_ref - x_RH;
                //                err_x_RH = err_x_RH.rows(0,2);

                x_LH = tr2diff(FK_tree(robot,q_tmp,robot.F_LH,ContactState));
                //                x_LH = x_LH.rows(0,2);
                err_x_LH = x_LH_ref - x_LH;
                //                err_x_LH = err_x_LH.rows(0,2);

                J_RH = J_tree(robot, q_tmp, robot.F_RH, ContactState);
                //                J_RH = J_RH.rows(0,2);
                J_LH = J_tree(robot, q_tmp, robot.F_LH, ContactState);
                //                J_LH = J_LH.rows(0,2);

                Jacob = join_cols(J_RH, J_LH);
                error = join_cols(err_x_RH, err_x_LH);

                qd_tmp = Kp*pinv(Jacob)*error;
                q_tmp = q_tmp + qd_tmp*tsamp;
                q_tmp = joint_limit_check(robot, q_tmp);

                cnt = cnt + 1;
                norm_error = norm(error, 2);

                //                cout << cnt << " , error: " << norm_error << endl;
                cout << cnt << endl;

                // when no solution is found
                if (cnt > 50){
                    norm_error = 0.001;
                    q_tmp = q_tmp_init;
                    cout << "solution is not found, try with different numbers" << endl;
                    flag_sol = 0;
                }
                else{
                    flag_sol = 1;
                }

            }

            // if the sol is not found
            // go to previous mmotion2ach(robot, qdata1, "motion.txt", 1);otions
            if (flag_sol == 0){
                cout << "Currently we have " << cnt_loop-1 << " steps have been made" << endl;
//                cout << shape(q_sol_data) << endl;
                cout << "how many steps back?: ";
                cin >> index_tmp;

                q_tmp = q_sol_data.row(cnt_loop-index_tmp).t();
            }
            else{
                // stores the configurations
                q_sol = q_tmp;
                q_sol_data = join_cols(q_sol_data, q_sol.t());
                cnt_loop++;
            }

            // generate intermediate motion
            qdata = CosTraj_mat(30, q_tmp_init, q_tmp);

            // Convert to Robot offset
            qdata = Model2Robot(robot, qdata);

            // interpolation
            qdata1 = spline_interp_mat(qdata, speed_motion);

            // send the motion to Hubo-Ach
            motion2ach(robot, qdata1, "motion_test.txt", 1, MODE, compliance, ALL_JOINTS, FINGER_NEUTRAL);

            cout << endl << "Do you want to continue? 1: Yes, 0: No: ";
            cin >> Loop;
        }

        //        // Convert to Robot offset
        //        q_tmp = Model2Robot(robot, q_tmp.t()).t();

        //        // generate end pose
        //        qdata = CosTraj_mat(100, q_tmp, zeros(robot.NJ,1));

        //        // interpolation
        //        qdata1 = spline_interp_mat(qdata, speed_motion);

        //        // send the motion to Hubo-Ach
        //        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE);
        cout << "END of Grasping Position Correction " << endl << endl;

    }

    /// 09-27-13 Go to Init Pose (DRC-Hubo)
    /// close the fingers and move the neck/waist to the init position
    else if (select == 100){
        DRCHubo robot;
        robot_init(robot);

        vec q_data = ReadAchJoint(robot);
        cout << "current configuration: " << endl;
        cout << q_data.t() << endl << endl;

        // from zero to init pose
        mat qdata = CosTraj_mat(100, q_data, zeros(robot.NJ,1));
        //cout << qdata << endl;

        // interpolation
        mat qdata1;

        // send init motion to Hubo-Ach
        qdata1 = spline_interp_mat(qdata, speed_motion);
//        bool compliance = false;

        motion2ach(robot, qdata1, "motion_test.txt", 1, MODE, compliance, ALL_JOINTS, FINGER_CLOSE);
    }

    /* Additional Features */

    // read the current encoder reading and the reference for one joint (DRC-Hubo)
    // 10-04-13
    else if (select == 101){

        DRCHubo robot;
        robot_init(robot);

        // get encoder readings
        vec encoderValues = getEncoderValues_new();

        // get Ref readings
        vec RefValues = getRefValues_new();

        // display the current encoder reading for selected joint
        cout << "current encoder reading: " << encoderValues(INDEX_JOINT) << endl;
        // display current reference for selected joint
        cout << "current reference: " << RefValues(INDEX_JOINT) << endl << endl;

    }

    // move one joint from current position to the desired position (DRC-Hubo)
    // 10-04-13
    else if (select == 102){

        // get encoder readings
        vec encoderValues = getEncoderValues_new();

        // display the current encoder reading for selected joint
        double pos_cur = encoderValues(INDEX_JOINT);
        cout << "current encoder reading: " << pos_cur << endl;

        int N = 50;

//        // trajectory that maintains the current configuration for all the joints
        mat achdata = ones(N,1)*encoderValues.t();
        vec angdata = CosTraj(N, pos_cur, JOINT_ANGLE);
        achdata.col(INDEX_JOINT) = angdata;

        bool compliance = false;

        // interpolation
        mat achdata1;

        // send init motion to Hubo-Ach
        achdata1 = spline_interp_mat(achdata, speed_motion);

        data2ach(achdata1, "motion_test.txt", 1, compliance, MODE);
    }

    // move one joint from current position to the reference position (DRC-Hubo)
    // 10-09-13
    else if (select == 103){

        // get encoder readings
        vec encoderValues = getEncoderValues_new();

        // get ref readings
        vec RefValues = getRefValues_new();

        int N = 50;

        mat achdata;

        if (INDEX_JOINT < 100){
            double pos_cur = encoderValues(INDEX_JOINT);
            double pos_ref = RefValues(INDEX_JOINT);

            // display the current encoder reading for selected joint
            cout << "current encoder reading: " << pos_cur << endl;
            cout << "current reference: " << pos_ref << endl;

            // trajectory that maintains the current configuration for all the joints
            achdata = ones(N,1)*encoderValues.t();
            vec angdata = CosTraj(N, pos_cur, pos_ref);
            achdata.col(INDEX_JOINT) = angdata;
        }
        // ALL JOINTS
        else if (INDEX_JOINT ==  100)
        {
            // interpolate from the current encoder to reference
            achdata = CosTraj_mat(N, encoderValues, RefValues);
        }

        bool compliance = false;

        // interpolation
        mat achdata1;

        // send init motion to Hubo-Ach
        achdata1 = spline_interp_mat(achdata, speed_motion);

        data2ach(achdata1, "motion_test.txt", 1, compliance, MODE);
    }

    // move one joint from current position to the desired position in the motion file
    // 10-10-13
    else if (select == 104){

        // get encoder readings
        vec encoderValues = getEncoderValues_new();


        int N = 100;

        mat achdata;

        // interpolate from the current encoder to reference
        achdata = CosTraj_mat(N, encoderValues, JOINT_DATA);

        bool compliance = false;

        // interpolation
        mat achdata1;

        // send init motion to Hubo-Ach
        achdata1 = spline_interp_mat(achdata, speed_motion);

        data2ach(achdata1, "motion_test.txt", 1, compliance, MODE);
    }

    // open/close both fingers
//    else if (select == 103){

//    }


    return 0;
}


// Tutorial for Robocpp (showing the basic functions)
void Tutorial(void){
    vec q = zeros<vec>(10,1);

    mat A = randu<mat>(4,4);

    //DRCHubo robot;
    Huboplus robot;
    //    vec data1 = CosTraj(20,1,10);
    //    vec data2 = CycloidTraj(20,0,20,10);

    cout << "--------------------------------------" << endl;
    cout << "|  basic functionalities of Robocpp  |" << endl;
    cout << "--------------------------------------" << endl << endl;

    int cnt = 0;

    cnt += 1;
    cout << cnt << ". raised cosine function" << endl <<endl;
    cout << CosTraj(20,1,10) << endl<<endl;

    cnt += 1;
    cout << cnt << ". Cycloid-like function" << endl <<endl;
    cout << CycloidTraj(20,0,20,10) << endl<<endl;

    cnt += 1;
    cout << cnt << ". get the shape of the matrix" << endl <<endl;
    cout << shape(A) << endl<<endl;

    cnt += 1;
    cout << cnt << ". check if 4x4 matrix" << endl<<endl;
    cout << ishomog(A) <<endl<<endl;

    cnt += 1;
    cout << cnt << ". unit vector computation" << endl<<endl;
    vec v;
    v << 1 << 2 << 3 << endr;
    cout << unit(v) <<endl<<endl;

    cnt += 1;
    cout << cnt << ". rotation about x-axis" << endl<<endl;
    cout << rotx(pi/8) <<endl<<endl;

    cnt += 1;
    cout << cnt << ". rotation about y-axis" << endl<<endl;
    cout << roty(pi/6) <<endl<<endl;

    cnt += 1;
    cout << cnt << ". rotation about z-axis" << endl<<endl;
    cout << rotz(pi/12) <<endl<<endl;

    cnt += 1;
    cout << cnt << ". convert R to TR" << endl<<endl;
    cout << r2t(rotz(pi/12)) <<endl<<endl;

    cnt += 1;
    cout << cnt << ". convert TR to R" << endl<<endl;
    cout << t2r(A) <<endl<<endl;

    cnt += 1;
    cout << cnt << ". rotation about x axis (TR)" << endl<<endl;
    cout << trotx(pi/9) <<endl<<endl;

    cnt += 1;
    cout << cnt << ". rotation about an arbitrary axis" << endl<<endl;
    mat axis;
    axis << 0 << 0 << 1 <<endr;
    cout << angvec2r(pi/2,axis) <<endl<<endl;

    cnt += 1;
    cout << cnt << ". translation decomposition" << endl<<endl;
    cout << tr2transl(trotx(pi/9)) <<endl<<endl;

    cnt += 1;
    cout << cnt << ". translation ==> 4x4 HTM" << endl<<endl;
    cout << transl2tr(1,3,4) <<endl<<endl;

    cnt += 1;
    cout << cnt << ". HTM ==> 6x1 diff vector" << endl<<endl;
    cout << tr2diff(transl2tr(1,3,4)) <<endl<<endl;

    cout << cnt << "======= Robot Information =====" << endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: NB" << endl<<endl;
    cout << robot.NB <<endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: grav" << endl<<endl;
    cout << robot.grav <<endl<<endl;

    cout << cnt << "======= Body Information =====" << endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: body_com" << endl<<endl;
    cout << robot.body_com <<endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: body_mass" << endl<<endl;
    cout << robot.body_mass <<endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: body_inertia" << endl<<endl;
    cout << robot.body_I <<endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: body_parent" << endl<<endl;
    cout << robot.body_parent <<endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: body_no_child_frames" << endl<<endl;
    cout << robot.body_no_child_frames <<endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: body_child_frames" << endl<<endl;
    cout << robot.body_child_frame <<endl<<endl;

    cout << cnt << "======= Joint Information =====" << endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: joint_axis" << endl<<endl;
    cout << robot.joint_axis <<endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: joint_translation" << endl<<endl;
    cout << robot.joint_translation <<endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: joint_rotation" << endl<<endl;
    cout << robot.joint_rotation <<endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: joint_q_min" << endl<<endl;
    cout << robot.joint_q_min <<endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: joint_q_max" << endl<<endl;
    cout << robot.joint_q_max <<endl<<endl;

    cout << cnt << "======= Frame Information =====" << endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: frame_axis" << endl<<endl;
    cout << robot.frame_axis <<endl<<endl;

    cnt += 1;
    cout << cnt << ". class info: frame_translation" << endl<<endl;
    cout << robot.frame_translation <<endl<<endl;

    /// initialize robot
    // get frame list
    // get body list
    robot_init(robot);

    // check get_frame_list
    cnt += 1;
    cout << cnt << ". get list of frames" << endl<<endl;
    cout << robot.frame_list <<endl<<endl;

    // check find index
    vec list;
    list << 0 << 0 << 1 <<endr;

    cnt += 1;
    cout << cnt << ". get index of the wanted value" << endl<<endl;
    cout << find_index(list,1) <<endl<<endl;

    //    // check get body_bf, body_cf
    //    cnt += 1;
    //    cout << cnt << ". get body_bf" << endl<<endl;
    //    cout << robot.body_bf <<endl<<endl;

    //    cnt += 1;
    //    cout << cnt << ". get body_cf" << endl<<endl;
    //    cout << robot.body_cf <<endl<<endl;

    //    // check get body_list
    //    cnt += 1;
    //    cout << cnt << ". get body_list" << endl<<endl;
    //    cout << robot.body_list <<endl<<endl;

    //    // check generated body path
    //    cnt += 1;
    //    cout << cnt << ". get body_path" << endl<<endl;
    //    cout << robot.body_path <<endl<<endl;

    //    // check generated body path for LF
    //    cnt += 1;
    //    cout << cnt << ". get body_path_LF" << endl<<endl;
    //    cout << robot.body_path_LF <<endl<<endl;

    //    // check generated // Jacobian
    //    cnt += 1;
    //    cout << cnt << ". get body_path_RF" << endl<<endl;
    //    cout << robot.body_path_RF <<endl<<endl;

    // check generated body path for LF (filtered)
    cnt += 1;
    cout << cnt << ". get body_path_LF_filtered" << endl<<endl;
    cout << robot.body_path_LF <<endl<<endl;

    // check generated body path for RF (filtered)
    cnt += 1;
    cout << cnt << ". get body_path_RF_filtered" << endl<<endl;
    cout << robot.body_path_RF <<endl<<endl;

    // HT
    cnt += 1;
    vec q_data = zeros<vec>(robot.NJ,1);
    cout << cnt << ". homogeneous transformation" << endl<<endl;
    //cout << trotz(0) <<endl<<endl;
    cout << HT_tree(robot, q_data, 1) <<endl<<endl;

    // FK
    cnt += 1;
    //vec q_data = zeros<vec>(robot.NJ,1);
    cout << cnt << ". forward kinematics" << endl<<endl;
    cout << FK_tree(robot, q_data, 14) <<endl<<endl;

    // FK - REF:LF
    cnt += 1;
    q_data =  pi/8*ones<vec>(robot.NJ,1);
    cout << cnt << ". forward kinematics (REF:LF)" << endl<<endl;
    cout << FK_tree(robot, q_data, robot.F_RH) <<endl<<endl;

    // Jacobian
    cnt += 1;
    q_data = pi/8*ones<vec>(robot.NJ,1);
    cout << cnt << ". Jacobian" << endl<<endl;
    cout << J_tree(robot, q_data, robot.F_RH, 3).row(5) <<endl<<endl;

    //    // Jacobian
    //    cnt += 1;
    //    q_data = zeros<vec>(robot.NJ,1);
    //    cout << cnt << ". Jacobian (REF:EEF)" << endl<<endl;
    //    cout << J_tree(robot, q_data, robot.F_RF, 1) <<endl<<endl;

    // CoM_robot
    cnt += 1;
    q_data = zeros<vec>(robot.NJ,1);
    cout << cnt << ". CoM" << endl<<endl;
    cout << CoM(robot, q_data, robot.RF) <<endl<<endl;

    // CoM Jacobian
    cnt += 1;
    q_data = zeros<vec>(robot.NJ,1);
    cout << cnt << ". CoM Jacobian" << endl<<endl;
    cout << J_com_tree(robot, q_data) <<endl<<endl;

    // check issubset function
    cnt += 1;
    mat x, y;
    x << 0 << 1 << 4 << 3 << 2 << endr;
    y << 1 << 2 << 3 << 0 << 3 << endr;
    cout << cnt << ". issubset?" << endl<<endl;
    cout << issubset(y, x) <<endl<<endl;

    // CoM Jacobian (REF:LF)
    cnt += 1;
    q_data = zeros<vec>(robot.NJ,1);
    cout << cnt << ". CoM Jacobian (REF:LF)" << endl<<endl;
    cout << J_com_tree(robot, q_data, robot.LF) <<endl<<endl;

    ///////////////// MISC.
    cnt += 1;
    mat qi = zeros<vec>(robot.NJ,1);
    mat qf = ones<vec>(robot.NJ,1);
    cout << cnt << ". raised cosine function (vec)" << endl <<endl;
    cout << CosTraj_mat(10,qi,qf) << endl<<endl;

    // save to a text file
    cnt += 1;
    cout << "data is saved to a text file" << endl << endl;
    CosTraj_mat(10,qi,qf).save("data.txt", raw_ascii);

    // save into a hubo-ach format file
    cnt += 1;
    cout << "data is saved to a hubo-ach format" << endl << endl;
    motion2ach(robot,qf.t(),"motion.txt");
}
