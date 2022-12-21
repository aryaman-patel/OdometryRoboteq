#ifndef ODOM_H_
#define ODOM_H_

#include <iostream>
#include <chrono>
#include <ctime> 
#include <sstream>
#include <cstring>
#include <fstream>
#include <string.h>
#include <math.h>
#include <cmath>
#include "RoboteqDevice.h"
#include "Constants.h"
#include "ErrorCodes.h"


class Positn_Correction //: public AGV_Functionality
{
    public:
        Positn_Correction();
        void dist_corr(double distance, int dir);
        void angle_corr(double angle, int dir);
        

    private:

        RoboteqDevice device;
        bool motor_command(int throttle, int steering);
        std::string port = "/dev/ttyACM0";
        double encoder_min;
        double encoder_max;

        double encoder_low_wrap;
        double encoder_high_wrap;
        int enc_l = 0;
        int enc_r = 0;

        double prev_lencoder;
        double prev_rencoder;
        double lmult;
        double rmult;


        double left;
        double right;
    	int enc_left ;
        double enc_right;
        double ticks_meter;
        double base_width;
        double dx;
        double dr;
        double x_final,y_final, theta_final;
        std::chrono::system_clock::time_point end;

        
        void get_wheel_enc();
        void delay(int millisecond);
        void write_log(std::string logName, double t_elapsed, int enc_l, int enc_r, double right, double left, int throttle, int steering, double X_final, double Y_final, double Distance = 0.0);




};

extern "C"
#endif
{

    Positn_Correction* Odom_constructor() {return new Positn_Correction();}
    void Dist_corr(Positn_Correction* pc, double dist, int dir) {return pc->dist_corr(dist,dir);}
    void Angle_corr(Positn_Correction* pc, double ang, int dir) {return pc->angle_corr(ang, dir);}
}

// Run the following in the terminal - 
// 'g++ -c odometry.cpp -fPIC -o odometry.o'
// 'g++ -c RoboteqDevice.cpp -fPIC -o RoboteqDevice.o'
// 'g++ odometry.o RoboteqDevice.o -shared -o libOdometry.so -Wall'
// OR build the cmake files.