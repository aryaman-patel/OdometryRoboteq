// Project: Odometry
// Created: 15th October 2021
#include "odom.h"
#define PI 3.14159265

Positn_Correction::Positn_Correction()
{
    // Create connection with Roboteq device.   
    //string port = "/dev/ttyACM0"; // For Linux
    // Constructor
	prev_lencoder = 0;
	prev_rencoder = 0;
	lmult = 0;
	rmult = 0;
	left = 0;
	right = 0;
	encoder_min =  -2147483648; //-65536; // to be checked where encode value changes to opposite side
	encoder_max =  2147483648; //65536;
	ticks_meter = 75021;  
	base_width = 0.472; // track width
	encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min ;
	encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min ;

	enc_left = 0; // check why 10
	enc_right = 0;


	dx = 0;
	dr = 0;
    end = std::chrono::system_clock::now(); 
 
	x_final = 0;y_final=0;theta_final=0;
    device.Connect(port);                  // Connect with motor controller.
    int conn_status = device.Connect(port);
    if(conn_status != RQ_SUCCESS){
        cout<<"\nError Connecting to device: "<<conn_status<<"."<<endl;
        cout<<"\nError Connecting to Roboteq device: "<<conn_status<<"."<<endl;
        cout<<"\nPlease Check Following Conditions!!"<<endl;
        cout<<"\n\t1. Check Serial Cable."<<endl;
        cout<<"\n\t2. Check MCB status."<<endl;
        cout<<"\n\t3. Check Toggle Switch."<<endl;
    }


}

void Positn_Correction::get_wheel_enc()
{
    // Get the encoder values using the Roboteq API.
    delay(10);
    // Get the encoder values using the Roboteq API.
    if (device.GetValue(_CSS,1,enc_l) ==  RQ_SUCCESS)
    {
        if((enc_l < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))
        {
            
            lmult = lmult + 1;
        }
        

        if((enc_l > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap))

        {
            
            lmult = lmult - 1;
        }

        left = 1.0 * (enc_l + lmult * (encoder_max - encoder_min )); // left, left ticks, enc are same.. it  is just that 
        //left is after wrapping. 

        prev_lencoder = enc_l;
    }
    else
    {
        std::cout << "Error getting left encoder values!!" << std::endl;
    }
    delay(10);
    if (device.GetValue(_CSS,2,enc_r) == RQ_SUCCESS)
    {
        if((enc_r < encoder_low_wrap) && (prev_rencoder > encoder_high_wrap))
        {
            
            rmult = rmult + 1;
        }
        

        if((enc_r > encoder_high_wrap) && (prev_rencoder < encoder_low_wrap))

        {
            
            rmult = rmult - 1;
        }

        right = 1.0 * (enc_r + rmult * (encoder_max - encoder_min ));

        prev_rencoder = enc_r;	
    }
    else
    {
        std::cout << "Error getting right encoder values!!" << std::endl;
    }

}

void Positn_Correction::dist_corr(double distance, int dir)
{
    // Code to make the robot move in a straight line for a given distance.
    auto start = std::chrono::system_clock::now();
    double elapsed;
    double dist_l = 0.0, dist_r = 0.0 , dist_c = 0.0, th = 0.0 , x = 0.0 , y = 0.0;
    bool record_logs = false;
    int throttle = dir*200;
    int steering = 0;
    x_final = 0.0;
    y_final = 0.0;
    theta_final = 0.0;
    double x_prev = 0.00;
    double y_prev = 0.00;
    double path_len = 0.00;
    string logName;
    if(record_logs){
        //Create Log for Current Time
        time_t time_now;
        struct tm * timeinfo;
        char buffer[80];
        time (&time_now);
        timeinfo = localtime(&time_now);
        strftime(buffer,sizeof(buffer),"%d-%m-%Y-%H:%M:%S",timeinfo);
        logName = buffer;
        logName = "../logs/"+logName+".csv";

        std::fstream log;
        log.open(logName,ios::out);
        log<<"Date Time"<<", "<<"Time Elapsed"<<", "<<"Encoder Left"<<", "<<"Encoder Right"<<", "<<"Right enc"<<", "<<"Left enc"<<", "<<"Throttle"<<", "<<"Steering"<< ", " << "X_final"<< ", " << "Y_final" << "," << "Distance" <<"\n";
        log.close();
    }
    while (true)
    {
        motor_command(throttle,steering);
        end = std::chrono::system_clock::now(); 
        get_wheel_enc();
        std::chrono::duration<double> elapsed_time = end - start;
        elapsed = elapsed_time.count();   // Calculate the elapsed time in double
        //std::cout << "Elapsed Time  --> " << elapsed << std::endl;

        if(enc_left == 0)
        {
            dist_l = 0;
            dist_r = 0;
        }
        else
        {
            dist_l = (left - enc_left)/(ticks_meter);
            //std::cout << "Left Wheel dist --------->" <<dist_l << std::endl;
            dist_r = (right - enc_right)/(ticks_meter);
            //std::cout << "Right Wheel dist --------->" <<dist_r << std::endl;
        }
        enc_left = left;
        enc_right = right;
        dist_c = (dist_l + dist_r)/2.0;    // cal distance covered by center

        th = (dist_r - dist_l)/base_width;

        dx = dist_c/elapsed;
        dr = th/elapsed;
        if ( dist_c != 0){
            x = cos(th)*dist_c; // increament in x and not the absolute position.. this is delta
            y = -sin(th)*dist_c; 
            // calculate the final position of the robot
            x_final = x_final + ( cos( theta_final ) * x - sin( theta_final ) * y ); // with total previous angle
            y_final = y_final + ( sin( theta_final ) * x + cos( theta_final ) * y );
		}
        if( th != 0)
            theta_final = theta_final + th; // theta final is the total angle
        
        path_len = std::sqrt(pow(x_final-x_prev, 2) + pow(y_final-y_prev, 2)) + path_len;
        x_prev = x_final;
        y_prev = y_final;
        // Set the values.
        if(record_logs){
            write_log(logName,elapsed, enc_l, enc_r, right, left, throttle, steering, x_final, y_final, path_len);
        }
        // std::cout << "x : " << x_final << std::endl;
        // std::cout << "y : " << y_final << std::endl;
        // std::cout << "theta : " << theta_final*180/PI << std::endl; 
        // std::cout << "Distance : " << path_len << std::endl;    
        if(abs(x_final) >= distance)
        {
            break;
        }

          
    }
    motor_command(0,0);
}

void Positn_Correction::angle_corr(double angle, int dir)
{
    // Code to make the robot turn by a certain angle.
    // angle input is in degrees
    // dir is 1 for clockwise and -1 for anticlockwise
    auto start = std::chrono::system_clock::now();
    double elapsed;
    double dist_l = 0.0, dist_r = 0.0 , dist_c = 0.0, th = 0.0 , x = 0.0 , y = 0.0;
    theta_final = 0.0;
    bool record_logs = false;
    int throttle = 0;
    int steering = dir*30;

    string logName;
    if(record_logs){
        //Create Log for Current Time
        time_t time_now;
        struct tm * timeinfo;
        char buffer[80];
        time (&time_now);
        timeinfo = localtime(&time_now);
        strftime(buffer,sizeof(buffer),"%d-%m-%Y-%H:%M:%S",timeinfo);
        logName = buffer;
        logName = "../logs/"+logName+".csv";

        std::fstream log;
        log.open(logName,ios::out);
        log<<"Date Time"<<", "<<"Time Elapsed"<<", "<<"Encoder Left"<<", "<<"Encoder Right"<<", "<<"Right enc"<<", "<<"Left enc"<<", "<<"Throttle"<<", "<<"Steering"<< ", " << "X_final"<< ", " << "Y_final" << "," << "Distance" <<"\n";
        log.close();
    }
    while (true)
    {

        motor_command(throttle,steering);
        end = std::chrono::system_clock::now(); 
        get_wheel_enc();
        std::chrono::duration<double> elapsed_time = end - start;
        elapsed = elapsed_time.count();   // Calculate the elapsed time in double
        std::cout << "Elapsed Time  --> " << elapsed << std::endl;

        if(enc_left == 0)
        {
            dist_l = 0;
            dist_r = 0;
        }
        else
        {
            dist_l = (left - enc_left)/(ticks_meter);
            std::cout << "Left Wheel dist --------->" <<dist_l << std::endl;
            dist_r = (right - enc_right)/(ticks_meter);
            std::cout << "Right Wheel dist --------->" <<dist_r << std::endl;
        }
        enc_left = left;
        enc_right = right;
        dist_c = (dist_l + dist_r)/2.0;    // cal distance covered by center

        th = (dist_r - dist_l)/base_width;

        dx = dist_c/elapsed;
        dr = th/elapsed;
        if ( dist_c != 0){
            x = cos(th)*dist_c; // increament in x and not the absolute position.. this is delta
            y = -sin(th)*dist_c; 
            // calculate the final position of the robot
            x_final = x_final + ( cos( theta_final ) * x - sin( theta_final ) * y ); // with total previous angle
            y_final = y_final + ( sin( theta_final ) * x + cos( theta_final ) * y );
		}
        if( th != 0)
            theta_final = theta_final + th; // theta final is the total angle

        // Set the values.
        if(record_logs){
            write_log(logName,elapsed, enc_l, enc_r, right, left, throttle, steering, x_final, y_final);
        }
        std::cout << "x : " << x_final << std::endl;
        std::cout << "y : " << y_final << std::endl;
        std::cout << "theta : " << theta_final*180/PI << std::endl;    
        if(abs(theta_final*180/PI) >= angle*0.98)
        {
            break;
        }

          
    }
    motor_command(0,0);
}

void Positn_Correction::delay(int millisecond)
{
    long pause;
    clock_t now,then;

    pause = millisecond*(CLOCKS_PER_SEC/1000);
    now = then = clock();
    while((now-then) < pause)
    {
        now = clock();
    }
}

//Function to Give Motor Command
bool Positn_Correction::motor_command(int throttle, int steering){
    int conn_status;
    int t = 10;
    if((conn_status = device.SetCommand(_GO, 1, throttle)) != RQ_SUCCESS){
        //agv_status.agv_error_code = "AFC016";
        std::cout<<"Failed in Motor Throttle --> "<<conn_status<<std::endl;
        return false;
    }
    //std::cout<<"Throttle Value --> " <<throttle <<std::endl;
    sleepms(t);
    int steer_dir = 1;		
    if((conn_status = device.SetCommand(_GO, 2, steer_dir*steering)) != RQ_SUCCESS){
        //agv_status.agv_error_code = "AFC016";
        std::cout<<"Failed in Motor Steering --> "<<conn_status<<std::endl;
        return false;
    }
    //std::cout<<"Steering Value --> "<< steer_dir*steering << std::endl;
    sleepms(t);
    return true;
}

void Positn_Correction::write_log(std::string logName, double t_elapsed, int enc_l, int enc_r, double right, double left, int throttle, int steering, double X_final, double Y_final, double Distance)
{
    // Write to a log file
    std::fstream log;
    log.open(logName,ios::app);
    time_t time_now;
    struct tm * timeinfo;
    char date_time[80];
    time (&time_now);
    timeinfo = localtime(&time_now);
    strftime(date_time,sizeof(date_time),"%d-%m-%Y-%H:%M:%S",timeinfo);

    log<<date_time<<", "<<t_elapsed<<", "<<enc_l<<", "<<enc_r<<", "<<right<<", "<<left<<", "<<throttle<<", "<<steering<< ", " << X_final << ", " << Y_final << ", " << Distance <<"\n";
    log.close();
}