/*
  Includes all contants and global variables in the program
*/

#include "constants_and_globals.h"


///////////////////////////
//                       //
// admittance parameters //
//                       // 
///////////////////////////
float EFE_AC_MASS = 0.25;
float EFE_AC_DAMPER = 3;
// Admittance control (Right Elbow Joint)
// Admittance control (Right Elbow Joint)
float Ts_MR = (control_loop_time/1000)/1000;
float Ka_MR = EFE_AC_MASS; // 0.038 //M
float Ko_MR = EFE_AC_DAMPER;     //D
float a0_MR =  Ts_MR / (2 *  Ka_MR +  Ko_MR *  Ts_MR);
float a1_MR =  Ts_MR / (2 *  Ka_MR +  Ko_MR *  Ts_MR);
float a2_MR = 0;
float b1_MR = (-2 *  Ka_MR +  Ko_MR *  Ts_MR) / (2 *  Ka_MR +  Ko_MR *  Ts_MR);
float b2_MR = 0;
float in_MR = 0.0;
float in_old_MR = 0.0;
float in_oldold_MR = 0.0;
float out_MR = 0.0;
float out_old_MR = 0.0;
float out_oldold_MR = 0.0;

// Admittance control (Left Elbow Joint)
float Ts_ML = (control_loop_time/1000)/1000;
float Ka_ML = EFE_AC_MASS; // 0.038 //M
float Ko_ML = EFE_AC_DAMPER;     //D
float a0_ML =  Ts_ML / (2 *  Ka_ML +  Ko_ML *  Ts_ML);
float a1_ML =  Ts_ML / (2 *  Ka_ML +  Ko_ML *  Ts_ML);
float a2_ML = 0;
float b1_ML = (-2 *  Ka_ML +  Ko_ML *  Ts_ML) / (2 *  Ka_ML +  Ko_ML *  Ts_ML);
float b2_ML = 0;
float in_ML = 0.0;
float in_old_ML = 0.0;
float in_oldold_ML = 0.0;
float out_ML = 0.0;
float out_old_ML = 0.0;
float out_oldold_ML = 0.0;


///////////////////////////
//                       //
//   Control paramters   //
//                       // 
///////////////////////////



float torque_constant  = 0.111;
float gear_ratio       = 100;
float radps_2_rpm        = 9.5493;    
int   motors_enable_flag = 0; 
float motor_actuation_loop = 0;
float motor_actuation_time = 50*1000; // 50 msec



float conditional_velocity_limit_rpm = 3000; //rpm on motor side
float conditional_velocity_limit_rps = (conditional_velocity_limit_rpm/gear_ratio)/radps_2_rpm; //rpm on motor side
float maximum_velocity = 4500; //rpm on motor side
float minimum_velocity = -4500; //rpm on motor side
float maximum_velocity_in = 4.7124; //rpm on link side
float minimum_velocity_in = -4.7124; //rpm on link side

float m_velocity_in = (maximum_velocity_in-minimum_velocity_in)/(3724 - 0);//using 12 bit resolution
float c_velocity_in = maximum_velocity_in - m_velocity_in*3724;

float m_velocity_out = (4096*0.9 - 4096*0.1)/(maximum_velocity-0);//using 12 bit resolution
float c_velocity_out = 4096*0.9 - m_velocity_out*maximum_velocity;


float maximum_current  = 3.5;  // amperes
float minimum_current  = -3.5;  // amperes

float m_current_in = (maximum_current-minimum_current)/(3724 - 0);//using 12 bit resolution
float c_current_in = maximum_current - m_current_in*3724;

float m_current_out = (4096*0.9 - 4096*0.1)/(maximum_current-0);//using 12 bit resolution
float c_current_out = 4096*0.9 - m_current_out*maximum_current;



///////////////////////////
//     Sit to Stand      //
//      Trajectory       //
//      parameters       // 
///////////////////////////
float final_position_MR_trajectory = 0;
float final_position_ML_trajectory = 0;
float initial_position_MR_trajectory = 0;
float initial_position_ML_trajectory = 0;
float compeletion_time_trajectory  = 0.5;
float compeletion_time_trajectory_f = 2;
float starting_time_trajectory = 0;
float current_time_trajectory  = 0;

///////////////////////////
//                       //
//    PID structure      //
//                       // 
///////////////////////////

struct PID_gains {
  float p_gain;
  float d_gain;
  float i_gain;
  float pre_error;
  float pre_i_val;
  float output;
};

///////////////////////////
//                       //
//     AF structure      //
//                       // 
///////////////////////////

struct AF_gains {
  float inertia_val;
  float damping_val;
  float stiffness_val;
};

///////////////////////////
//                       //
//     LP filter      //
//                       // 
///////////////////////////

struct LP_vals {
  float old_vel_MR;
  float old_cur_MR;
  float old_vel_ML;
  float old_cur_ML;
  float ts;
  float wc;
};

///////////////////////////
//                       //
//   control strategy    //
//                       // 
///////////////////////////
char control_strategy = 'A';
// A admittance
// T trajectory control
