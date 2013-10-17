/*
 * Robocpp.cpp
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
//#include "DRCHubo.h"
//#include "Huboplus.h"

//namespace Robocpp {

//const double pi = 3.14159;

// Raised Cosine Interpolation
// written on 08-03-13
vec CosTraj(double N, double ri, double rf){
    vec data = linspace(0,N-1,N);
    vec f = 0.5 - 0.5*cos(data*pi/(N-1));
    vec Traj = ri*ones(N,1) + (rf-ri)*f;
    return Traj;
}

// Raised Cosine Interpolation - vector
// written on 08-27-13
mat CosTraj_mat(double N, vec qi, vec qf){
    mat Traj = zeros(N, qi.n_rows);

    if(qi.n_rows != qf.n_rows){
        cout << "the dimension is invalid" << endl;
    }
    else{
        for (int i=0; i<int(qi.n_rows); i++){
            Traj.col(i) = CosTraj(N, qi(i), qf(i));
        }
    }
    return Traj;
}

// conversion from degree to radian
double deg2rad(double angle){
    return angle*pi/180;
}

// conversion from radian to degree
double rad2deg(double angle){
    return angle*180/pi;
}


// check the size of the matrix
mat shape(mat data){
    mat size_of_data = zeros(1,2);
    size_of_data(0,0) = data.n_rows;
    size_of_data(0,1) = data.n_cols;
    return size_of_data;
}

// True if TR is a 4x4 homogeneous transform matrix
bool ishomog(mat TR){
    int n_rows = TR.n_rows;
    int n_cols = TR.n_cols;

    return (n_rows == 4) && (n_cols == 4);
}

// True if R is a 3x3 rotation matrix
bool isrot(mat R){
    int n_rows = R.n_rows;
    int n_cols = R.n_cols;

    return (n_rows == 3) && (n_cols == 3);
}

// Return a unit vector
vec unit(vec v){
    return v/norm(v, 2);
}

// Rotation about X-axis
mat rotx(double theta){
    double ct = cos(theta);
    double st = sin(theta);
    mat R;
    R << 1 << 0 << 0 << endr
      << 0 << ct << -st << endr
      << 0 << st << ct << endr;

    return R;
}

// Rotation about Y-axis
mat roty(double theta){
    double ct = cos(theta);
    double st = sin(theta);
    mat R;
    R << ct << 0 << st << endr
      << 0 << 1 << 0 << endr
      << -st << 0 << ct << endr;

    return R;
}

// Rotation about Z-axis
mat rotz(double theta){
    double ct = cos(theta);
    double st = sin(theta);
    mat R;
    R << ct << -st << 0 << endr
      << st << ct << 0 << endr
      << 0 << 0 << 1 << endr;

    return R;
}



// Convert 4x4 TR Matrix to 3x3 R matrix
mat t2r(mat TR){
    if (ishomog(TR) == false){
        cout << "input must be a homogeneous transform" <<endl;
        exit(-1);
    }
    return TR.submat(0,0,2,2);
}

// Convert 3x3 Rotation Matrix to 4x4 TR matrix
mat r2t(mat R){
    mat tmp;
    tmp << 0 << 0 << 0 << 1 <<endr;
    return join_cols(join_rows(R, zeros(3,1)),tmp);
}

// Rotation about X-axis
mat trotx(double theta){

    return r2t(rotx(theta));
}

// Rotation about Y-axis
mat troty(double theta){

    return r2t(roty(theta));
}

// Rotation about Z-axis
mat trotz(double theta){

    return r2t(rotz(theta));
}

// Rotation about arbitrary axis
mat angvec2r(double theta, mat v){
    double cth = cos(theta);
    double sth = sin(theta);
    double vth = 1-cth;
    double kx = v(0,0);
    double ky = v(0,1);
    double kz = v(0,2);
    mat R;
    R << kx*kx*vth+cth << ky*kx*vth-kz*sth << kz*kx*vth+ky*sth <<endr
      << kx*ky*vth+kz*sth <<   ky*ky*vth+cth <<   kz*ky*vth-kx*sth<<endr
      << kx*kz*vth-ky*sth <<  ky*kz*vth+kx*sth <<   kz*kz*vth+cth<<endr;

    return R;
}

// Rotation about arbitrary axis
mat angvec2tr(double theta, mat v){
    return r2t(angvec2r(theta, v));
}

// decompose translational homogeneous transformations
mat tr2transl(mat TR){
    if (ishomog(TR) == false){
        cout << "input must be a homogeneous transform" <<endl;
        exit(-1);
    }
    return TR(span(0,2),3);
}

// Create translational homogeneous transformations
mat transl2tr(double x, double y, double z){
    vec t;
    t << x << y << z << 1 << endr;
    return join_rows(join_cols(eye(3,3),zeros(1,3)),t);
}

// convert a HTM to differential representation
vec tr2diff(mat TR){
    if (ishomog(TR) == false){
        cout << "input must be a homogeneous transform" <<endl;
        exit(-1);
    }

    vec d;
    d << 0.5*(TR(2,1) - TR(1,2)) << 0.5*(TR(0,2) - TR(2,0))
      << 0.5*(TR(1,0) - TR(0,1)) << endr;
    return join_cols(TR(span(0,2),3),d);
}

// Cyloid-like Function
// given initial pos, maximum height, final height, generate a cycloid trajectory
// written on 08-03-13
// r1: stepheight
// r2: steplength_vertical
vec CycloidTraj(float N, float ri, float r1, float r2){
    vec Traj = zeros(N,1);
    float Nmid = N/2;
    float f;

    for (int i=0; i<Nmid; i++){
        f = r1/2*(1 - cos(i*2*pi/(N-1)));
        Traj[i] = ri + f;
    }

    for (int i=Nmid; i<N; i++){
        f = ((ri+r1)-r2)/2*(1 - cos(i*2*pi/(N-1)));
        Traj[i] = r2 + f;
    }
    return Traj;
}

// spline interpolation
// written on 08-30-13
vec spline_interp(vec data, double N){
    int rows = data.n_rows;
    vec t = linspace<vec>(0,rows-1,rows);
    vec tnew = linspace<vec>(0,rows-1,rows*N);

    Cubic f(rows,t,data);

    vec data_out = zeros<vec>(rows*N,1);
    for (int i=0; i<rows*N; i++){
        data_out(i) = f.getValue(tnew(i));
    }

    return data_out;
}

// spline interpolation
// written on 08-30-13
mat spline_interp_mat(mat data, double N){
    int rows = data.n_rows;
    int cols = data.n_cols;
    vec t = linspace<vec>(0,rows-1,rows);
    vec tnew = linspace<vec>(0,rows-1,rows*N);

    vec data_tmp;
    mat data_out = zeros(rows*N,cols);
    for(int col=0; col<cols; col++){
        data_tmp = data.col(col);
        Cubic f(rows,t,data_tmp);

        for (int row=0; row<rows*N; row++){
            data_out(row,col) = f.getValue(tnew(row));
        }
    }

    return data_out;
}

//}

//// Class Functions

//! Class constructor
Cubic::Cubic(int n, vec x, vec y)
{
    m_n = n;
    m_x = zeros<vec>(n + 1,1);
    m_y = zeros<vec>(n + 1,1);
    m_b = zeros<vec>(n + 1,1);
    m_c = zeros<vec>(n + 1,1);
    m_d = zeros<vec>(n + 1,1);

    // remember and shift base and shift base.
    for (int i = 1; i <= n; ++i) {
        m_x(i) = x(i - 1);
        m_y(i) = y(i - 1);
    }

    // linear interpolation when we have too little data (i.e. just two points!)
    if (n < 3) {
        m_b(2) = m_b(1) = (m_y(2) - m_y(1)) / (m_x(2) - m_x(1));
        m_c(1) = m_c(2) = m_d(1) = m_d(2) = 0.0;
        return;
    }

    m_d(1) = m_x(2) - m_x(1);
    m_b(1) = - m_d(1);
    m_c(2) = (m_y(2) - m_y(1)) / m_d(1);
    m_c(1) = 0.0;
    for (int i = 2; i < n; ++i) {
        m_d(i) = m_x(i + 1) - m_x(i);
        m_b(i) = 2.0 * (m_d(i - 1) + m_d(i));
        m_c(i + 1) = (m_y(i + 1) - m_y(i)) / m_d(i);
        m_c(i) = m_c(i + 1) - m_c(i);
    }

    m_b(n) = -m_d(n - 1);
    m_c(n) = 0.0;
    if (n != 3) {
        m_c(1) = m_c(3) / (m_x(4) - m_x(2)) - m_c(2) / (m_x(3) - m_x(1));
        m_c(n) = m_c(n - 1) / (m_x(n) - m_x(n - 2)) - m_c(n - 2) / (m_x(n - 1) - m_x(n - 3));
        m_c(1) *= m_d(1) * m_d(1) / (m_x(4) - m_x(1));
        m_c(n) *= - m_d(n - 1) * m_d(n - 1) / (m_x(n) - m_x(n - 3));
    }
    for (int i = 2; i < n; ++i) {
        double T = m_d(i - 1) / m_b(i - 1);
        m_b(i) -= T * m_d(i - 1);
        m_c(i) -= T * m_c(i - 1);
    }

    m_c(n) /= m_b(n);
    for  (int i = n - 1; i > 0; --i)
        m_c(i) = (m_c(i) - m_d(i) * m_c(i + 1)) / m_b(i);

    m_b(n) = (m_y(n) - m_y(n - 1)) / m_d(n - 1) + m_d(n - 1) * (m_c(n - 1) + 2.0 * m_c(n));

    for (int i = 1; i < n; ++i) {
        m_b(i) = (m_y(i + 1) - m_y(i)) / m_d(i) - m_d(i) * (m_c(i + 1) + 2.0 * m_c(i));
        m_d(i) = (m_c(i + 1) - m_c(i)) / m_d(i);
        m_c(i) *= 3.0;
    }
    m_c(n) *= 3.0;
    m_d(n) = m_d(n - 1);
}

//! m_class destructor
/*Cubic::~Cubic(){
    //  delete [] m_x;
    //  delete [] m_y;
    //  delete [] m_b;
    //  delete [] m_c;
    //  delete [] m_d;
}*/

//! Returns an interpolated value.
double Cubic::getValue(double x)
{
    if (x <= m_x(1))
        return (((m_y(2)-m_y(1))/(m_x(2)-m_x(1)))*(x-m_x(1))+m_y(1));
    if (x >= m_x(m_n))
        return (((m_y(m_n)-m_y(m_n-1))/(m_x(m_n)-m_x(m_n-1)))*(x-m_x(m_n-1))+m_y(m_n-1));

    if (x <= m_x(2))
    {
        double dx = x - m_x(1);
        return m_y(1) + dx * (m_b(1) + dx * (m_c(1) + dx * m_d(1)));
    }

    int i = 1, j = m_n + 1;
    do {
        int k = (i + j) / 2;
        (x < m_x(k)) ? j = k : i = k;
    } while (j > i + 1);

    double dx = x - m_x(i);
    return m_y(i) + dx * (m_b(i) + dx * (m_c(i) + dx * m_d(i)));
}

// return index of the list that matches the value
int find_index(vec list, int value){
    int index = -100;
    for (int i=0;i<int(list.n_rows);i++){
        if(value == list(i)){
            index = i;
        }
    }
    if (index==-100){
        //cout << "there is no such value in the list" <<endl;
    }
    return index;
}

// return the first index of the list that matches the value
int find_index_mat(mat list, int value){
    int ind_out = -100;
    for (int i=0;i<int(list.n_cols);i++){
        if(value == list(0,i)){
            ind_out = i;
            break;
        }
    }
    if (ind_out==-100){
        //cout << ind_out << " there is no such value in the list" <<endl;
    }
    return ind_out;
}

// check if x is subset of y
bool issubset(mat y, mat x){
    int val, index;
    for (int i=0; i<int(x.n_cols); i++){
        val = x(0,i);
        // for valid entry
        if (val != -1){
            // check if the index exist (!= -100)
            index = find_index_mat(y, val);
            if (index == -100){
                return false;
            }
        }
    }
    return true;
}

// get submatrix in reversed column order
mat get_submat(mat input, int ind_start, int ind_end){
    int tmp_ind = ind_start;
    mat tmp = -100*ones(1,100);
    int cnt = 0;
    while(tmp_ind >= ind_end){
        tmp(0,cnt) = input(0,tmp_ind);
        tmp_ind--;
        cnt++;
    }
    //cout<<cnt<<endl;
    return tmp(0,span(0,cnt-1));
}

// string to double conversion
double str2double(string s) {
     double d;
     stringstream ss(s); //turn the string into a stream
     ss >> d; //convert
     return d;
}

// string to int conversion
double str2int(string s) {
     int d;
     stringstream ss(s); //turn the string into a stream
     ss >> d; //convert
     return d;
}

// find a specific joint data from a specific file and a specific line number
double FindJointAngFile(char* filename, int desired_line_no, int JOINT_INDEX){
    string line;
    //int desired_line = 3;
    //ifstream inputfile("../data.txt");
    ifstream inputfile(filename);
    if (!inputfile){
        cout<<"failed to open the file!"<<endl;
        exit(1);
    }

    if (desired_line_no <= 0){
        cout <<"invalid line number!" << endl;
        exit(1);
    }

    for(int i = 0; i < desired_line_no ; ++i){
       getline(inputfile, line);
    }

    // diplay specific line
    //cout << desired_line_no <<": " <<endl;
    //cout << line << endl;

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

    //int JOINT_INDEX = 1;
    //cout << "data that we want: "<< temp_data(JOINT_INDEX) << endl;

    return temp_data(JOINT_INDEX);
}

// find joint data from a specific file and a specific line number
vec FindJointDataFile(char* filename, int desired_line_no){
    string line;

//    cout <<

    //int desired_line = 3;
    //ifstream inputfile("../data.txt");
    ifstream inputfile(filename);
    if (!inputfile){
        cout<<"failed to open the file!"<<endl;
        exit(1);
    }

    if (desired_line_no <= 0){
        cout <<"invalid line number!" << endl;
        exit(1);
    }

    for(int i = 0; i < desired_line_no ; ++i){
       getline(inputfile, line);
    }

    // diplay specific line
//    cout << desired_line_no <<": " <<endl;
//    cout << line << endl;

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

    //int JOINT_INDEX = 1;
    //cout << "data that we want: "<< temp_data(JOINT_INDEX) << endl;

    return temp_data;
}
