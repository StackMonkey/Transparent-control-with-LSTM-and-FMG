  
//===================================================================================
// INCLUDES
//===================================================================================

//#include <Encoder.h>
// esp32 board manager must be 2.0.16
#include <ESP32Encoder.h> // Must be version 0.7.0
#include "constants_and_globals.h"
#include "control_parameters.h"
#include "InverseDynamics.hh"
#include "HardwareSerial.h"


ESP32Encoder encoderMR;
ESP32Encoder encoderML;
float reference_signals = 0.0;

// timer and flag for example, not needed for encoders
unsigned long encoder2lastToggled;
bool encoder2Paused = false;

HardwareSerial MySerial(1);
InverseDynamics right;
//InverseDynamics left;
//===================================================================================
// TYPECASTS
//===================================================================================

//////////////////////////////////////////////////////////
//////////////// Initial setup commands //////////////////
//////////////////////////////////////////////////////////
float torqueInfo = 0.f;
float dynamictorque = 0.f;
float payloadMass = 1.1;
float scalingUpper = 7.87; //0.0009913;//7.87
float scalingLower = -7.87;//-0.00082609;//-7.87
PID_gains MR_pos_gains = { 0.1, 0.001, 0, 0, 0, 0};
PID_gains ML_pos_gains = { 0.1, 0.003, 0, 0, 0, 0};
PID_gains MR_vel_gains = { 1, 0, 0, 0, 0, 0};
PID_gains ML_vel_gains = { 1, 0, 0, 0, 0, 0};

LP_vals lp_vals = {0, 0, 0, 0, 0.002, 190};

AF_gains MR_AF_gains = { 0.15, 0.0, 0.0};//{ 0.15, 4, 0.001};
AF_gains ML_AF_gains = { 0.15, 0.0, 0.0};
//Encoder knobRight(MR_PE_A, MR_PE_B);

float assist_val_right = 0;
float assist_val_left = 0;
float last_detected = millis();
float difference_val = 0;

#define NUM_data 20
byte buff_data[NUM_data];
int data_send_loop = 0;

void setup() {
  //analogWriteResolution(12);  // set the analog output resolution to 12 bit
  analogReadResolution(12);   // set the analog input resolution to 12 bit
  //SerialBT.begin("Motor_Control"); //Bluetooth device name
  Serial.begin(250000); //Bluetooth device name
  MySerial.begin(250000, SERIAL_8N1, 16, 17);



  // Enable the weak pull down resistors
  ESP32Encoder::useInternalWeakPullResistors = UP;

  // set starting count value
  encoderMR.setCount(0);
  encoderML.setCount(0);
  right.LSTMsetup();
  //left.LSTMsetup();
  // clear the encoder's raw count and set the tracked count to zero
  encoderMR.clearCount();
  encoderML.clearCount();

  // Attache pins for use as encoder pins
  encoderMR.attachHalfQuad(MR_PE_A, MR_PE_B);
  // Attache pins for use as encoder pins
  encoderML.attachHalfQuad(ML_PE_A, ML_PE_B);

  // set the lastToggle
  encoder2lastToggled = millis();

  pinmode_slection();

  ///////
  char c = 'C';
  char C = 'b';
  while (c != 'C')
  {
    //c = SerialBT.read();
    if (Serial.available()){
      c = Serial.read();
    }
  }
  //SerialBT.print('M');  // sending charachter to PS
  //SerialBT.print('>');  // sending charachter to PS
  Serial.print('M');  // sending charachter to PS
  Serial.print('>');  // sending charachter to
  c = 'B';
  while (c != 'B')
  {
    if (Serial.available()){
      c = Serial.read();
    }
    
  }
  MySerial.println('A');  // sending charachter to PS

  // set starting count value
  encoderMR.setCount(0);
  encoderML.setCount(0);

  // clear the encoder's raw count and set the tracked count to zero
  encoderMR.clearCount();
  encoderML.clearCount();

  previous_current_time_control = (float)micros();
  //attachInterrupt(readLC(LC_raw_data), 100000);
  // Setup interrupt
  //Timer1.attachInterrupt(LC_raw_data).setPeriod(100*1000).start(); // time is set in micro seconds
  //esp_wifi_stop();
  for (int i = 0; i < 20; i++)
  {
    exo_data[i] = i;
  }
}

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////           /////////////////////////
////////////////////// Main loop /////////////////////////
//////////////////////           /////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

void loop()
{
  /*if (SerialBT.available())
    {
    received_command = SerialBT.read();
    }*/
  if (Serial.available())
  {
    received_command = Serial.read();
  }
  if (received_command == 'G')
  {
    read_string();
  }
  exo_data[9] = (int)(assist_val_right * 100 / 13.5);
  exo_data[19] = (int)(assist_val_left * 100 / 13.5);


  exo_data[3] = max_assist_value;
  exo_data[13] = max_assist_value;

  //  readExo(Exo_raw_data); ///// Reading EXO parameters
  //  ///////////////////////////////////////////////
  //  ///////////////////////////////////////////////
  //  ////  reading LC data after every 100 msec ////
  //  ///////////////////////////////////////////////
  //  ///////////////////////////////////////////////

  readExo(Exo_filter_data); ///// Reading EXO parameters

  Exo_filter_data[1] = (lp_vals.ts * lp_vals.wc * Exo_filter_data[1] + lp_vals.old_vel_MR) / (1 + lp_vals.ts * lp_vals.wc);
  right.acceleration = (Exo_filter_data[1]-lp_vals.old_vel_MR)/lp_vals.ts;
  right.acceleration = (lp_vals.ts * lp_vals.wc * right.acceleration + right.prevAcceleration) / (1 + lp_vals.ts * lp_vals.wc);
  lp_vals.old_vel_MR = Exo_filter_data[1];
  right.prevAcceleration = right.acceleration;
  Exo_filter_data[2] = (lp_vals.ts * lp_vals.wc * Exo_filter_data[2] + lp_vals.old_cur_MR) / (1 + lp_vals.ts * lp_vals.wc);
  lp_vals.old_cur_MR = Exo_filter_data[2];

  Exo_filter_data[4] = (lp_vals.ts * lp_vals.wc * Exo_filter_data[4] + lp_vals.old_vel_ML) / (1 + lp_vals.ts * lp_vals.wc);
 // left.acceleration = (Exo_filter_data[4]-lp_vals.old_vel_ML)/lp_vals.ts;
  //left.acceleration = (lp_vals.ts * lp_vals.wc * left.acceleration + left.prevAcceleration) / (1 + lp_vals.ts * lp_vals.wc);
  lp_vals.old_vel_ML = Exo_filter_data[4];
  //left.prevAcceleration = left.acceleration;
  Exo_filter_data[5] = (lp_vals.ts * lp_vals.wc * Exo_filter_data[5] + lp_vals.old_cur_ML) / (1 + lp_vals.ts * lp_vals.wc);
  lp_vals.old_cur_ML = Exo_filter_data[5];


  //actual_position
  exo_data[0] = (int)Exo_filter_data[0];
  exo_data[10] = (int)Exo_filter_data[3];
  //actual_velocity
  exo_data[1] = (int)((Exo_filter_data[1] + 5) * 25);
  exo_data[11] = (int)((Exo_filter_data[4] + 5) * 25);
  //actual_current
  exo_data[2] = (int)((Exo_filter_data[2] + 5) * 25);
  exo_data[12] = (int)((Exo_filter_data[5] + 5) * 25);

  ///////////////////////////////////////////////
  ///////////////////////////////////////////////
  ////  reading LC data after every 100 msec ////
  ///////////////////////////////////////////////
  ///////////////////////////////////////////////
 if (LC_read_counter >= 100 / (control_loop_time / 1000))
  {
    readLC(LC_filter_data);
    exo_data[6] = (LC_filter_data[0] + 20) * 6;
    exo_data[7] = (LC_filter_data[1] + 20) * 6;
    exo_data[8] = (LC_filter_data[2] + 20) * 6;

    exo_data[16] = (LC_filter_data[3] + 20) * 6;
    exo_data[17] = (LC_filter_data[4] + 20) * 6;
    exo_data[18] = (LC_filter_data[5] + 20) * 6;
    LC_read_counter = 0;
  }
  LC_read_counter = LC_read_counter + 1;
  //load cell data


  ///////////////////////////////////////////////
  ///                                         ///
  ///         admittance filter               ///
  ///                                         ///
  ///////////////////////////////////////////////
  if (control_strategy == 'T')
  {
    MR_AF_gains.inertia_val = 0.2; // 0.038 //M
    MR_AF_gains.damping_val = 4;     //D
    ML_AF_gains.inertia_val = 0.2; // 0.038 //M
    ML_AF_gains.damping_val = 4;     //D
    current_time_trajectory = millis();

    if (((current_time_trajectory - starting_time_trajectory) / 1000) <= compeletion_time_trajectory)
    {
      desired_position_MR = trajectory_generator (initial_position_MR_trajectory, final_position_MR_trajectory, (current_time_trajectory - starting_time_trajectory) / 1000, compeletion_time_trajectory);
      desired_position_ML = trajectory_generator (initial_position_ML_trajectory, final_position_ML_trajectory, (current_time_trajectory - starting_time_trajectory) / 1000, compeletion_time_trajectory);
      difference_val = (LC_filter_data[0] - desired_position_MR);
      desired_velocity_MR = admittance_filter_MR(difference_val);
      desired_velocity_ML = admittance_filter_ML(LC_filter_data[3] - desired_position_ML);
    }
    else
    {
      desired_position_MR = final_position_MR_trajectory;
      desired_position_ML = final_position_ML_trajectory;
      difference_val = (LC_filter_data[0] - desired_position_MR);
      desired_velocity_MR = admittance_filter_MR(difference_val);
      desired_velocity_ML = admittance_filter_ML(LC_filter_data[3] - desired_position_ML);
    }
  }
  if (control_strategy == 'A')
  {
    MR_AF_gains.inertia_val = 0.14 + right.mass2*sq(right.lp); // 0.038 //M
    MR_AF_gains.damping_val = 0.01;     //D
    //ML_AF_gains.inertia_val = 0.14 + left.mass2*sq(left.lp); // 0.038 //M
    ML_AF_gains.damping_val = 0.01;     //D'
    if(payloadMass > 0.0){
      MR_AF_gains.inertia_val = 0.1 + right.mass2*sq(right.lp); // 0.038 //M
      MR_AF_gains.damping_val = 0.01;     //D
      //ML_AF_gains.inertia_val = 0.1 + left.mass2*sq(left.lp); // 0.038 //M
      ML_AF_gains.damping_val = 0.01;     //D'
    }
    float rtorque = right.Torque(Exo_filter_data[0]*3.142/180.0,Exo_filter_data[1],payloadMass,0.002);
    dynamictorque = rtorque;
    //float ltorque = left.Torque(Exo_filter_data[3]*3.142/180.0,Exo_filter_data[4],payloadMass,0.002);
    float scaledrtorque = right.Scaling_transformation(right.diffTorque(rtorque),-1,1,scalingLower,scalingUpper);
    //float scaledltorque = left.Scaling_transformation(left.diffTorque(ltorque),-1,1,scalingLower,scalingUpper);
    float rtorquePre = right.Inverse_Scaling_transformation(right.LSTMpredict(scaledrtorque),-1,1,scalingLower,scalingUpper);
    //float ltorquePre = left.Inverse_Scaling_transformation(left.LSTMpredict(scaledltorque),-1,1,scalingLower,scalingUpper);
    float invrtorquePre = right.invDiffTorque(rtorquePre);
    //float invltorquePre = left.invDiffTorque(ltorquePre); 
    desired_velocity_MR = admittance_filter_MR(invrtorquePre);
    //desired_velocity_ML = admittance_filter_ML(invltorquePre);
    desired_velocity_ML = 0;
    torqueInfo = invrtorquePre;
  }

  //if (SerialBT.available())
  //{
  //  serial_receive = SerialBT.read();
  //}

  ///////////////////////////////
  ////                      /////
  /////   Control enable    /////
  ////      command         /////
  ///////////////////////////////
  if (received_command == 'C')
  {
    //digitalWrite(control_status_ledPin, HIGH);
    control_status = 'C';
    received_command = 'n';
  }
  ///////////////////////////////
  ////                      /////
  /////   Control disable   /////
  ////      command         /////
  ///////////////////////////////
  if (received_command == 'D')
  {
    control_status = 'D';
    received_command = 'n';
    LC_read_counter = 0;
  }
  ///////////////////////////////
  ///////////////////////////////
  ///////////////////////////////

  ///////////////////////////////
  ////                      /////
  /////   MOVE CW and ACW   /////
  ////                      /////
  ///////////////////////////////
  if (received_command == 'E')
  {
    control_status = 'E';
    received_command = 'n';
  }
  if (received_command == 'F')
  {
    control_status = 'F';
    received_command = 'n';
  }
  ///////////////////////////////
  ///////////////////////////////
  ///////////////////////////////

  ////////////////////////////////////
  /////                           ////
  /////    Start sending data     ////
  /////                           ////
  ////////////////////////////////////
  if (received_command == 'S')
  {
    send_data = 'S';  // command to disconnect the sensor
    received_command = 'n';
  }
  ///////////////////////////////
  ///////////////////////////////
  ///////////////////////////////

  ////////////////////////////////////
  /////                           ////
  /////  Terminate sending data   ////
  /////                           ////
  ////////////////////////////////////
  if (received_command == 'T')
  {
    send_data = 'T';  // command to disconnect the sensor
    received_command = 'n';
  }
  ///////////////////////////////
  ///////////////////////////////
  ///////////////////////////////

  ////////////////////////////////////
  /////                           ////
  /////      Control Strategy     ////
  /////                           ////
  ////////////////////////////////////
  if (received_command == 't')
  {
    if ((millis() - last_detected) > 2000)
    {
      last_detected = millis();
      if (Exo_filter_data[0] > 40 && Exo_filter_data[3] > 40)
      {
        control_strategy = 'T';
        control_status = 'C';
        initial_position_MR_trajectory = LC_filter_data[0];
        initial_position_ML_trajectory = LC_filter_data[3];
        final_position_MR_trajectory = max_assist_value;
        final_position_ML_trajectory = max_assist_value;
        starting_time_trajectory = millis();
      }
    }
    received_command = 'n';
  }
  if (received_command == 'A')
  {
    control_strategy = 'A';

    received_command = 'n';
  }
  ////////////////////////////////////
  /////                           ////
  /////  Terminate sending data   ////
  /////                           ////
  ////////////////////////////////////
  if (received_command == 'X')
  {
    data_frequency = 50;  // command to change data transmission frequency
    sampling_time = 20; // micro seconds
    previous_current_time_sending = 0;
    received_command = 'n';
  }
  if (received_command == 'Y')
  {
    data_frequency = 10;  // command to change data transmission frequency
    sampling_time = 100; // micro seconds
    previous_current_time_sending = 0;
    received_command = 'n';
  }
  ///////////////////////////////
  ///////////////////////////////
  ///////////////////////////////

  ///////////////////////////////
  ///////////////////////////////
  ///////////////////////////////

  ///////////////////////////////
  ////                      /////
  /////   MOVE ACW and CW  //////
  ////                      /////
  ///////////////////////////////
  if (control_status == 'E')
  {
    control_strategy = 'F';
    control_status = 'C';
    received_command = 'n';
    initial_position_MR_trajectory = Exo_filter_data[0];
    initial_position_ML_trajectory = Exo_filter_data[3];
    final_position_MR_trajectory = 10;
    final_position_ML_trajectory = 10;
    starting_time_trajectory = millis();
  }
  if (control_status == 'F')
  {
    control_strategy = 'F';
    control_status = 'C';
    received_command = 'n';
    initial_position_MR_trajectory = Exo_filter_data[0];
    initial_position_ML_trajectory = Exo_filter_data[3];
    final_position_MR_trajectory = 100;
    final_position_ML_trajectory = 100;
    starting_time_trajectory = millis();
  }

  //desired_velocity
  exo_data[4] = (int)((desired_velocity_MR + 5) * 25);
  exo_data[14] = (int)((desired_velocity_ML + 5) * 25);
  ///////////////////////////////
  ////                      /////
  /////   Control Enabled  //////
  ////                      /////
  ///////////////////////////////
  if (control_status == 'C')
  {
    if (motors_enable_flag == 0)
    {
      digitalWrite(MR_en, HIGH);
      digitalWrite(ML_en, HIGH);
      motors_enable_flag = 1;
    }
    /// motor actuation loop to run at 50 HZ
    if (motor_actuation_loop >= (motor_actuation_time / control_loop_time))
    {
      ////////////////////////
      //
      // Trajectory control using escon current control
      //
      ////////////////////////

      if (control_strategy == 'F')
      {
        current_time_trajectory = millis();

        if (((current_time_trajectory - starting_time_trajectory) / 1000) <= compeletion_time_trajectory_f)
        {
          desired_position_MR = trajectory_generator (initial_position_MR_trajectory, final_position_MR_trajectory, (current_time_trajectory - starting_time_trajectory) / 1000, compeletion_time_trajectory_f);
          PID_control((desired_position_MR - Exo_filter_data[0]), &MR_pos_gains);
          current_control(MR_pos_gains.output, MR_dir, MR_pwm_channel);
          desired_position_ML = trajectory_generator (initial_position_ML_trajectory, final_position_ML_trajectory, (current_time_trajectory - starting_time_trajectory) / 1000, compeletion_time_trajectory_f);
          PID_control((desired_position_ML - Exo_filter_data[3]), &ML_pos_gains);
          current_control(ML_pos_gains.output, ML_dir, ML_pwm_channel);
        }
        else
        {
          current_control((desired_position_MR - Exo_filter_data[0]) * 0, MR_dir, MR_pwm_channel);
          current_control((desired_position_ML - Exo_filter_data[3]) * 0, ML_dir, ML_pwm_channel);
          control_strategy = 'A';
        }
      }
      if (control_strategy == 'A' || control_strategy == 'T')
      {
        //Serial.println("control status C");
        if (desired_velocity_MR > 0)
        {
          if (desired_velocity_MR > conditional_velocity_limit_rps)
          {
            desired_velocity_MR = conditional_velocity_limit_rps;
          }
        }
        if (desired_velocity_MR < 0)
        {
          if (desired_velocity_MR < -1 * conditional_velocity_limit_rps)
          {
            desired_velocity_MR = -1 * conditional_velocity_limit_rps;
          }
        }
        desired_velocity_MR = safety_function(desired_velocity_MR, Exo_filter_data[1], Exo_filter_data[0]);
        Serial.print(torqueInfo);
        Serial.print(",");
        Serial.print(dynamictorque);
        Serial.print(",");
        Serial.print(LC_filter_data[0]);
        Serial.print(",");
        Serial.print(Exo_filter_data[0]);
        Serial.print(",");
        Serial.print(Exo_filter_data[1]);
        Serial.print(",");
        Serial.print(desired_velocity_MR);
        Serial.print(",");
        Serial.print(right.acceleration);
        Serial.println();
        PID_control((desired_velocity_MR - Exo_filter_data[1]), &MR_vel_gains);
        /*Serial.print(MR_vel_gains.output);
          Serial.print(',');*/
        current_control(MR_vel_gains.output, MR_dir, MR_pwm_channel);

        if (desired_velocity_ML > 0)
        {
          if (desired_velocity_ML > conditional_velocity_limit_rps)
          {
            desired_velocity_ML = conditional_velocity_limit_rps;
          }
        }
        if (desired_velocity_ML < 0)
        {
          if (desired_velocity_ML < -1 * conditional_velocity_limit_rps)
          {
            desired_velocity_ML = -1 * conditional_velocity_limit_rps;
          }
        }
        desired_velocity_ML = safety_function(desired_velocity_ML, Exo_filter_data[4], Exo_filter_data[3]);
        PID_control((desired_velocity_ML - Exo_filter_data[4]), &ML_vel_gains);

        current_control(ML_vel_gains.output, ML_dir, ML_pwm_channel);
      }
      motor_actuation_loop = 0;
    }
    motor_actuation_loop = motor_actuation_loop + 1;
  }
  ///////////////////////////////
  ///////////////////////////////
  ///////////////////////////////



  ///////////////////////////////
  ////                      /////
  /////  Control disabled  //////
  ////                      /////
  ///////////////////////////////
  if (control_status == 'D')
  {
    digitalWrite(MR_en, LOW);
    digitalWrite(ML_en, LOW);
    motors_enable_flag = 0;
    //motor_actuation_loop = 0;
    velocity_control(0, MR_dir, MR_pwm_channel);
    velocity_control(0, ML_dir, ML_pwm_channel);
    if (motor_actuation_loop >= (motor_actuation_time / control_loop_time)){
        Serial.print(torqueInfo);
        Serial.print(",");
        Serial.print(dynamictorque);
        Serial.print(",");
        Serial.print(LC_filter_data[0]);
        Serial.print(",");
        Serial.print(Exo_filter_data[0]);
        Serial.print(",");
        Serial.print(Exo_filter_data[1]);
        Serial.print(",");
        Serial.print(desired_velocity_MR);
        Serial.print(",");
        Serial.print(right.acceleration);
        Serial.println();
      motor_actuation_loop = 0;
    }
    motor_actuation_loop++;
  }
  ///////////////////////////////
  ///////////////////////////////
  ///////////////////////////////

  ////////////////////////////////////
  /////                           ////
  /////Loop time to save data in  ////
  // semg array after each 14msec //
  ////////////////////////////////////
  // if data frequency is 50
  for (int column_index = 0; column_index < 20; column_index++)
  {
    exo_data_matrix[column_index] = exo_data[column_index];
  }
  ////////////////////////////////////
  /////                           ////
  ////////////////////////////////////

  ////////////////////////////////////
  /////                           ////
  /////       Sending data        ////
  ////                            ////
  ////////////////////////////////////
  // if data frequency is 50

  if (send_data == 'S')
  {
    if (data_send_loop < 50)
    {
      current_time = millis();

      if (previous_current_time_sending > current_time)
      {
        previous_current_time_sending = 0;
      }

      if ((current_time - previous_current_time_sending) >= 16)
      {
        previous_current_time_sending = current_time;
        for (int ii = 0; ii < 20; ii++)
        {
          buff_data[ii] = (byte)(exo_data_matrix[ii]);
        }
        //SerialBT.write(buff_data, 20);
        Serial.write(buff_data, 20);
        data_send_loop = data_send_loop + 1;
      }
    }
    else
    {
      data_send_loop = 0;
      send_data = 'n';
    }
  }

  ////////////////////////////////////
  /////                           ////
  ////////////////////////////////////

  //////////////////////////////
  ////                     /////
  /// controlling loop time ////
  ////                     /////
  //////////////////////////////
  current_time = (float)micros();
  loop_time = current_time - previous_current_time_control;
  if (loop_time < control_loop_time)
  {
    delayMicroseconds(control_loop_time - loop_time);
  }
  previous_current_time_control = (float)micros();
  //////////////////////////////
  //////////////////////////////
  //////////////////////////////
}


//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
/////////////////////            /////////////////////////
///////////////////// ADDITIONAL /////////////////////////
///////////////////// FUNCTIONS  /////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////


////////////////////////////////////
////                           /////
////// pin mode selection //////////
////                           /////
////////////////////////////////////
void pinmode_slection(void)
{
  // MR //
  pinMode(MR_PE_A, INPUT);
  pinMode(MR_PE_B, INPUT);
  pinMode(MR_en, OUTPUT);
  pinMode(MR_dir, OUTPUT);
  ledcSetup(MR_pwm_channel, 5000, 12); // 5000 pwm frequency , 12 resolution
  ledcAttachPin(MR_pwm, MR_pwm_channel);
  // ML //
  pinMode(ML_PE_A, INPUT);
  pinMode(ML_PE_B, INPUT);
  pinMode(ML_en, OUTPUT);
  pinMode(ML_dir, OUTPUT);
  ledcSetup(ML_pwm_channel, 5000, 12); // 5000 pwm frequency , 12 resolution
  ledcAttachPin(ML_pwm, ML_pwm_channel);
}
////////////////////////////////////
////////////////////////////////////
////////////////////////////////////

///////////////////////////////////////
////                              /////
// reading raw data from exoskeleton //
////                              /////
///////////////////////////////////////
void readExo(float *Exo_readings)
{
  Exo_readings[0] = encoderMR.getCount();
  Exo_readings[0] = Exo_readings[0] * 360 / 2000;
  Exo_readings[1] = analogRead(MR_vel) * m_velocity_in + c_velocity_in + 0.65; // velocity
  Exo_readings[1] = -1 * Exo_readings[1];
  Exo_readings[2] = analogRead(MR_curr) * m_current_in + c_current_in; // current

  Exo_readings[3] = encoderML.getCount() * (-1);
  Exo_readings[3] = Exo_readings[3] * 360 / 2000;
  Exo_readings[4] = analogRead(ML_vel) * m_velocity_in + c_velocity_in + 0.65; // velocity
  Exo_readings[5] = analogRead(ML_curr) * m_current_in + c_current_in; // current

}
/////////////////////////////////////
/////////////////////////////////////
/////////////////////////////////////

////////////////////////////////////
////                           /////
////  Readings from Lad cell   /////
////                           /////
////////////////////////////////////

void readLC(float *LC)
{
  MySerial.print('1');
  delayMicroseconds(10);
  LC[0] = -1 * read_digit() * 0.27;
  delayMicroseconds(10);

  MySerial.print('2');
  delayMicroseconds(10);
  LC[1] = read_digit() * 0.27;
  delayMicroseconds(10);

  MySerial.print('3');
  delayMicroseconds(10);
  LC[2] = read_digit() * 0.27;
  delayMicroseconds(10);

  MySerial.print('4');
  delayMicroseconds(10);
  LC[3] = read_digit() * 0.27;
  delayMicroseconds(10);

  MySerial.print('5');
  delayMicroseconds(10);
  LC[4] = read_digit() * 0.27;
  delayMicroseconds(10);

  MySerial.print('6');
  delayMicroseconds(10);
  LC[5] = read_digit() * 0.27;
  delayMicroseconds(10);
  MySerial.print('t');
}

////////////////////////////////////
////////////////////////////////////
////////////////////////////////////

////////////////////////////////////
////                  /////
////   Read float     /////
////                  /////
////////////////////////////////////
float read_digit(void)
{

  String inString = "";
  char serial_data = 'n';//
  float val = 0;
  delayMicroseconds(10);
  int timeOut = 10; 
  int timeOutCount = millis();
  while (serial_data != 't')
  {
    if (MySerial.available())
    {
      serial_data = MySerial.read();
      if (serial_data != 't')
      {
        inString += serial_data;
      }
    }
  }
  val = inString.toInt() / 100;
  return val;
}

////////////////////////////////////
////                  /////
//// admittance filter /////
////                  /////
////////////////////////////////////


float admittance_filter_MR(float interaction_torque)
{
  Ka_MR = MR_AF_gains.inertia_val; // 0.038 //M
  Ko_MR = MR_AF_gains.damping_val;     //D
  //set input
  in_MR = interaction_torque;
  a0_MR = Ts_MR / (2 * Ka_MR + Ko_MR * Ts_MR);
  a1_MR = Ts_MR / (2 * Ka_MR + Ko_MR * Ts_MR);
  a2_MR = 0;
  b1_MR = (-2 * Ka_MR + Ko_MR * Ts_MR) / (2 * Ka_MR + Ko_MR * Ts_MR);
  b2_MR = 0;
  // filter
  out_MR = a0_MR * in_MR + a1_MR * in_old_MR + a2_MR * in_oldold_MR - b1_MR * out_old_MR - b2_MR * out_oldold_MR;
  // assign old values for next iteration
  in_oldold_MR = in_old_MR;
  in_old_MR = in_MR;
  out_oldold_MR = out_old_MR;
  out_old_MR = out_MR;
  return out_MR;
}

float admittance_filter_ML(float interaction_torque)
{

  Ka_ML = ML_AF_gains.inertia_val; // 0.038 //M
  Ko_ML = ML_AF_gains.damping_val;     //D
  //set input
  in_ML = interaction_torque;
  a0_ML = Ts_ML / (2 * Ka_ML + Ko_ML * Ts_ML);
  a1_ML = Ts_ML / (2 * Ka_ML + Ko_ML * Ts_ML);
  a2_ML = 0;
  b1_ML = (-2 * Ka_ML + Ko_ML * Ts_ML) / (2 * Ka_ML + Ko_ML * Ts_ML);
  b2_ML = 0;
  // filter
  out_ML = a0_ML * in_ML + a1_ML * in_old_ML + a2_ML * in_oldold_ML - b1_ML * out_old_ML - b2_ML * out_oldold_ML;
  // assign old values for next iteration
  in_oldold_ML = in_old_ML;
  in_old_ML = in_ML;
  out_oldold_ML = out_old_ML;
  out_old_ML = out_ML;
  return out_ML;
}


////////////////////////////////////
////                  /////
//// Velocity Control /////
////                  /////
////////////////////////////////////

void velocity_control(float reference_signal, int digital_pin, int analog_pin)
{

  reference_signal = reference_signal * gear_ratio; // converting velocity to motor side rad/sec
  reference_signal = reference_signal * radps_2_rpm; // converting rad/sec to rpm

  if (reference_signal > 0)
  {
    digitalWrite(digital_pin, HIGH);
  }
  if (reference_signal < 0)
  {
    digitalWrite(digital_pin, LOW);
    reference_signal = -1 * reference_signal;
  }
  if (reference_signal > conditional_velocity_limit_rpm)
  {
    reference_signal = conditional_velocity_limit_rpm;
  }
  reference_signal = m_velocity_out * reference_signal + c_velocity_out;
  ledcWrite(analog_pin, (int)reference_signal);
}

////////////////////////////////////
////                  /////
//// Current Control  /////
////                  /////
////////////////////////////////////
void current_control(float reference_signal, int digital_pin, int analog_pin)
{

  if (reference_signal > 0)
  {
    digitalWrite(digital_pin, HIGH);
  }
  if (reference_signal < 0)
  {
    digitalWrite(digital_pin, LOW);
    reference_signal = -1 * reference_signal;
  }
  if (reference_signal > maximum_current)
  {
    reference_signal = maximum_current;
  }
  reference_signal = m_current_out * reference_signal + c_current_out;
  ledcWrite(analog_pin, (int)reference_signal);
}

////////////////////////////////////
////                  /////
////  Torque Control  /////
////                  /////
////////////////////////////////////

void direct_torque_control(float reference_signal, int digital_pin, int analog_pin)
{
  if (reference_signal > 0)
  {
    digitalWrite(digital_pin, HIGH);
  }
  if (reference_signal < 0)
  {
    digitalWrite(digital_pin, LOW);
    reference_signal = -1 * reference_signal;
  }

  reference_signal = reference_signal / gear_ratio; // torque at motor side
  reference_signal = reference_signal / torque_constant; // conversion to current

  if (reference_signal > maximum_current)
  {
    reference_signal = maximum_current;
  }

  reference_signal = m_current_out * reference_signal + c_current_out;
  ledcWrite(analog_pin, (int)reference_signal);
}

////////////////////////////////////
////                   /////
////  safety function  /////
////                   /////
////////////////////////////////////

float safety_function(float reference_signal, float actual_velocity, float actual_position)
{
  if (actual_position < 10)
  {
    if (reference_signal < 0)
    {
      reference_signal = 0;
    }
  }
  if (actual_position > 100)
  {
    if (reference_signal > 0)
    {
      reference_signal = 0;
    }
  }
  return reference_signal;
}

////////////////////////////////////
////                                /////
////   Trajectory generation code   /////
////                                /////
////////////////////////////////////

float trajectory_generator (float initial_val, float final_val, float cur_time, float comp_time)
{
  float val = 0.0;
  float a = 10;
  float b = -15;
  float c = 6;

  val = a * pow(cur_time / comp_time, 3) + b * pow(cur_time / comp_time, 4) + c * pow(cur_time / comp_time, 5);
  val = initial_val + (final_val - initial_val) * val;
  return val;
}


////////////////////////////////////
////                                /////
////  position initialization code  /////
////                                /////
////////////////////////////////////

void position_initialization(int digital_pin, int analog_pin, float reference_signal)
{
  digitalWrite(digital_pin, LOW);
  reference_signal = m_current_out * reference_signal + c_current_out;
  ledcWrite(analog_pin, (int)reference_signal);
}

////////////////////////////////////
////                                /////
////         PID  control           /////
////                                /////
////////////////////////////////////

void PID_control(float error_value, PID_gains *pid_struc)
{
  float p_val = 0.0;
  float d_val = 0.0;
  float i_val = 0.0;
  p_val = pid_struc->p_gain * error_value;
  d_val = pid_struc->d_gain * (error_value - pid_struc->pre_error) / (motor_actuation_time / 1000000);
  i_val = pid_struc->i_gain * (motor_actuation_time / 1000000) * error_value + pid_struc->pre_i_val;
  pid_struc->output = p_val + d_val + i_val;
  pid_struc->pre_i_val = i_val;
  pid_struc->pre_error = error_value;
}

/////////////////////////////////////////
////                                /////
////         read  string           /////
////                                /////
/////////////////////////////////////////

void read_string()
{
  String stringone = "";
  String mc_sel = "";
  String gains_sel = "";
  String gains_val = "";
  int comma_index = 0;
  received_command = 'n';
  while (received_command != 'T')
  {
    /*if (SerialBT.available())
      {
      received_command = SerialBT.read();*/
    if (Serial.available())
    {
      received_command = Serial.read();
      if (received_command != 'T')
      {
        stringone += received_command;
      }
      else
      {
        mc_sel = stringone.substring(0, 3);
        gains_sel = stringone.substring(4);
        if (mc_sel == "rmp")
        {
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          MR_pos_gains.p_gain = gains_val.toFloat();
          gains_sel = gains_sel.substring(comma_index + 1);
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          MR_pos_gains.d_gain = gains_val.toFloat();
          gains_sel = gains_sel.substring(comma_index + 1);
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          MR_pos_gains.i_gain = gains_val.toFloat();
        }
        if (mc_sel == "lmp")
        {
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          ML_pos_gains.p_gain = gains_val.toFloat();
          gains_sel = gains_sel.substring(comma_index + 1);
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          ML_pos_gains.d_gain = gains_val.toFloat();
          gains_sel = gains_sel.substring(comma_index + 1);
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          ML_pos_gains.i_gain = gains_val.toFloat();
        }
        ////////////////
        if (mc_sel == "rmv")
        {
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          MR_vel_gains.p_gain = gains_val.toFloat();
          gains_sel = gains_sel.substring(comma_index + 1);
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          MR_vel_gains.d_gain = gains_val.toFloat();
          gains_sel = gains_sel.substring(comma_index + 1);
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          MR_vel_gains.i_gain = gains_val.toFloat();
        }
        if (mc_sel == "lmv")
        {
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          ML_vel_gains.p_gain = gains_val.toFloat();
          gains_sel = gains_sel.substring(comma_index + 1);
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          ML_vel_gains.d_gain = gains_val.toFloat();
          gains_sel = gains_sel.substring(comma_index + 1);
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          ML_vel_gains.i_gain = gains_val.toFloat();
        }
        if (mc_sel == "raf")
        {
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          MR_AF_gains.inertia_val = gains_val.toFloat();
          gains_sel = gains_sel.substring(comma_index + 1);
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          MR_AF_gains.damping_val = gains_val.toFloat();
          gains_sel = gains_sel.substring(comma_index + 1);
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          MR_AF_gains.stiffness_val = gains_val.toFloat();
        }

        if (mc_sel == "laf")
        {
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          ML_AF_gains.inertia_val = gains_val.toFloat();
          gains_sel = gains_sel.substring(comma_index + 1);
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          ML_AF_gains.damping_val = gains_val.toFloat();
          gains_sel = gains_sel.substring(comma_index + 1);
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          ML_AF_gains.stiffness_val = gains_val.toFloat();
        }
        if (mc_sel == "rav")
        {
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          assist_val_right = gains_val.toFloat() * 13.5;
        }
        if (mc_sel == "lav")
        {
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          assist_val_left = gains_val.toFloat() * 13.5;
        }
        if (mc_sel == "mav")
        {
          comma_index = gains_sel.indexOf(',', 0);
          gains_val = gains_sel.substring(0, comma_index);
          max_assist_value = gains_val.toFloat();
        }
      }
    }
  }
  received_command = 'n';
}

////////////////////////////////////
////                                /////
////  PID for velocity control      /////
////                                /////
////////////////////////////////////

/*float PID_velocity(float desired_value, float actual_value)
  {
  float control_input = 0.0;

  }*/


////////////////////////////////////
////                                /////
////       Data to transmitt        /////
////                                /////
////////////////////////////////////

// Right motor
// 0 act pos
// 1 act vel
// 2 act curr
// 3 ref pos
// 4 ref vel
// 5 ref curr
// 6 fx
// 7 fy
// 8 fx
// 9 something

// left motor
// 10 act pos
// 11 act vel
// 12 act curr
// 13 ref pos
// 14 ref vel
// 15 ref curr
// 16 fx
// 17 fy
// 18 fx
// 19 something
