/*
  Includes all contants and global variables in the program
*/
#ifndef constants_and_globals_h
#define constants_and_globals_h
// communication through serial port
char serial_receive = 'n';//
char send_data_status = 'n';
int exo_row_index = 0;
float data_frequency = 50; //Hz
float sampling_time = 20; // micro seconds
// Serial Communication
char received_command = 'n';//
char send_data = 'n';//
int send_loop = 0;

// time related
unsigned long current_time = 0;
unsigned long previous_current_time_saving = 0;
unsigned long previous_current_time_sending = 0;
unsigned long previous_current_time_control = 0.f;
float loop_time = 0;
float control_loop_time = 2 * 1000;

float exo_data_matrix[100] = {};//
float exo_data[20] = {};
//////////////////////////////////////
///// Motor control and feedback /////
//////////////////////////////////////
//const int control_status_ledPin =  13;
char control_status = 'D';//
long loop_count = 1;

// position encoder
int MR_PE_A =  23; //M2 -> MR
int MR_PE_B =  22;
int ML_PE_A =  19; //M1 -> ML
int ML_PE_B =  21;

// velocity and current feedback
const int MR_vel =  A4; //M2 -> MR
const int MR_curr =  A7;
const int ML_vel =  A2; //M1 -> ML 26;//
const int ML_curr =  A3; // 25;//

// PWM pins
const int MR_pwm =  33; //M2 -> MR
const int MR_pwm_channel = 0;
const int ML_pwm =  4; //M1 -> ML
const int ML_pwm_channel = 1;

// enable and direction feedback
const int MR_en =  15; //M2 -> MR
const int MR_dir =  14;
const int ML_en =  5; //M1 -> ML
const int ML_dir =  18;

// EXO data
float old_Exo_raw_data[6] = {};//
float Exo_raw_data[6] = {};//
float old_Exo_filter_data[6] = {};//
float Exo_filter_data[6] = {};//
//////////////////////////////////////
//////////////////////////////////////
//////////////////////////////////////


float old_LC_raw_data[6] = {};//
float LC_raw_data[6] = {};//
float old_LC_filter_data[6] = {};//
float LC_filter_data[6] = {};//
int LC_read_counter = 0;
//////////////////////////////////////
//////////////////////////////////////
//////////////////////////////////////

//////////////////////////////////////
////////// Digital filter ////////////
//////////////////////////////////////
// cuttoff frequency = 100hz;
// sampilng time = 1ms;
float fcut = 20;
float wcut = 2 * 3.14 * fcut;
float Tsamp = control_loop_time/1000;
float f_a = Tsamp * wcut;
float f_b = Tsamp * wcut;
float f_c = 2 + wcut * Tsamp;
float f_d = wcut * Tsamp - 2;
//////////////////////////////////////
//////////////////////////////////////
//////////////////////////////////////


float desired_position_MR = 0;
float desired_velocity_MR = 0;
float desired_current_MR = 0;
float desired_position_ML = 0;
float desired_velocity_ML = 0;
float desired_current_ML = 0;

float actual_position_MR = 0;
float actual_velocity_MR = 0;
float actual_current_MR = 0;
float actual_position_ML = 0;
float actual_velocity_ML = 0;
float actual_current_ML = 0;

float torque_x_MR = 0;
float torque_y_MR = 0;
float torque_z_MR = 0;
float torque_x_ML = 0;
float torque_y_ML = 0;
float torque_z_ML = 0;

float previous_position = 0;

float assistive_force = 0;

float max_assist_value = 8.1;

///////////////////////////////////////////////////
////////// communication with LC ESP32 ////////////
///////////////////////////////////////////////////

//const int interrupt_pin = 4;


////////////////////////////////////////////////////////////////////////////////////////
// alphabets assigned to particular command received from serial port 'serial_receive'//
////////////////////////////////////////////////////////////////////////////////////////
#endif
// C -> connect teensy and enable control
// D -> disconnect teensy and disable control
// c -> initiate calibration
// S -> send data
