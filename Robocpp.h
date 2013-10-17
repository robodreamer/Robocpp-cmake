/*
 * Robocpp.h
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

#ifndef ROBOCPP_H
#define ROBOCPP_H

#include <iostream>
#include <sstream>
#include <string>
#include <cstdio>
#include <armadillo>
#include <ctime>
#include <math.h>
#include <iomanip>
//#include "DRCHubo.h"
#include "Huboplus.h"

#define pi 3.14159

using namespace std;
using namespace arma;

// Raised Cosine Interpolation
// written on 08-03-13
vec CosTraj(double N, double ri, double rf);

// Raised Cosine Interpolation - vector
// written on 08-27-13
mat CosTraj_mat(double N, vec qi, vec qf);

// conversion from degree to radian
double deg2rad(double angle);

// conversion from radian to degree
double rad2deg(double angle);

// check the size of the matrix
mat shape(mat data);

// True if TR is a 4x4 homogeneous transform matrix
bool ishomog(mat TR);

// True if R is a 3x3 rotation matrix
bool isrot(mat R);

// Return a unit vector
vec unit(vec v);

// Rotation about X-axis
mat rotx(double theta);

// Rotation about Y-axis
mat roty(double theta);

// Rotation about Z-axis
mat rotz(double theta);

// Convert 4x4 TR Matrix to 3x3 R matrix
mat t2r(mat TR);

// Convert 3x3 Rotation Matrix to 4x4 TR matrix
mat r2t(mat R);

// Rotation about X-axis
mat trotx(double theta);

// Rotation about Y-axis
mat troty(double theta);

// Rotation about Z-axis
mat trotz(double theta);

// Rotation about arbitrary axis
mat angvec2r(double theta, mat v);

// Rotation about arbitrary axis
mat angvec2tr(double theta, mat v);

// decompose translational homogeneous transformations
mat tr2transl(mat TR);

// Create translational homogeneous transformations
mat transl2tr(double x, double y, double z);

// convert a HTM to differential representation
vec tr2diff(mat TR);

// Cyloid-like Function
// given initial pos, maximum height, final height, generate a cycloid trajectory
// written on 08-03-13
// r1: stepheight
// r2: steplength_vertical
vec CycloidTraj(float N, float ri, float r1, float r2);

// spline interpolation
// written on 08-30-13
vec spline_interp(vec data, double N);

// spline interpolation
// written on 08-30-13
mat spline_interp_mat(mat data, double N);

// return index of the list that matches the value
int find_index(vec list, int value);

// return the first index of the list that matches the value
int find_index_mat(mat list, int value);

// check if x is subset of y
bool issubset(mat y, mat x);

// get submatrix in reversed column order
mat get_submat(mat input, int ind_start, int ind_end);

// convert a string number to double number
double str2double(string s);

// string to int conversion
double str2int(string s);

// find a specific joint data from a specific file and a specific line number
double FindJointAngFile(char* filename, int desired_line_no, int JOINT_INDEX);

// find joint data from a specific file and a specific line number
vec FindJointDataFile(char* filename, int desired_line_no);

// Cubic Spline Class
class Cubic{
public:

    //! Class constructor
    Cubic(int n, vec x, vec y);

    //! m_class destructor
    //~Cubic();

    //! Returns an interpolated value.
    double getValue(double x);

private:

    int m_n;

    //double *m_x, *m_y, *m_b, *m_c, *m_d;
    vec m_x;
    vec m_y;
    vec m_b;
    vec m_c;
    vec m_d;

};



#endif // ROBOCPP_H
