// Create timer interrupt of 10msec for IMU reading

/* Firmware for ESP32 board
  - Initiate connection with Python Script (PS)
  - IF data_req is received start sending data continuously
  - If disconnect recv  - restart main loop

  - Connection close/re-open works properly
  - Data rate too slow with 126 hz on BT
*/

#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"

#include "commands.h"

#define DEFAULT_INPUT_VOLTAGE 1

// typedef float DataType;
typedef float DataType;


#define NUM_FSR 8
#define NUM_IMU 13

#define SERIAL_USB Serial
#define SERIAL_BT SerialBT

#define SERIAL SERIAL_BT
#define BAUDRATE 115200

DataType net_fsr = 0;

DataType data[NUM_FSR + NUM_IMU] = {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999};

byte buff[(NUM_FSR + NUM_IMU)*sizeof(DataType)] = {};
float stack_fsr[160] = {};
int stack_fsr_count = 0;

byte buff_data_FSR[NUM_FSR * 20];
byte buff_data_IMU_bd[NUM_IMU];
byte buff_data_IMU_ad[NUM_IMU];

//===================================================================================
// TYPECASTS
//===================================================================================

bool initBluetooth()
{
  if (!btStart()) {
   // Serial.println("Failed to initialize controller");
    return false;
  }

  if (esp_bluedroid_init() != ESP_OK) {
    //Serial.println("Failed to initialize bluedroid");
    return false;
  }

  if (esp_bluedroid_enable() != ESP_OK) {
    //Serial.println("Failed to enable bluedroid");
    return false;
  }

}


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define BUTTON_PIN_BITMASK 0x000000000 // 2^0 in hex
RTC_DATA_ATTR int bootCount = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
BluetoothSerial SERIAL_BT;


////////////////////////////
//                        //
//      time related      //
//        constants       //
////////////////////////////
float current_time = 0.0;
float imu_read_time = 10000; // is in micro seconds which is 10 milli second
float imu_start_timer = 0;
float imu_end_timer = 0;
float send_time = 500; // is in micro seconds which is 5 milli second
float timer_send = 0.0;
float sleep_time = 20000; // is in micro seconds which is 5 milli second
float pre_sleep_time = 0.0;
float charge_time = 1500; // is in micro seconds which is 2 second
float pre_charge_time = 0; //
float send_imu_counter = 0;

float transmit_time = 500; // is in micro seconds which is 2 second
float pre_transmit_time = 0; //

float look_for_off = 0;
////////////////////////////
//                        //
//  Serial Communication  //
//        constants       //
////////////////////////////
//char command_received = 'n';//
Command command_received = NONE;
// char send_data = 'n';//

bool send_data = false;
bool stop_data = false;
bool calibrate_amplifier = false;
bool calibration_status = true;


////////////////////////////
//                        //
//         I/O pins       //
//                        //
////////////////////////////
const int red_pin =  13;
const int grn_pin =  15;
const int dac_pin =  26;
int input_voltage = 10;
const int charge_id =  25;
int val = 0;

bool first_loop = true;

unsigned long timer = 0;
long imu_loopTime = 8000;   // microseconds

bool conn_established = false;

void setup()
{
  pinMode(charge_id, INPUT);
  pinMode(red_pin, OUTPUT);
  pinMode(grn_pin, OUTPUT);
  ////////////////////////////
  //                        //
  //    Initialize setup    //
  //        files           //
  ////////////////////////////

  Wire.begin(23, 22);
  SERIAL_USB.begin(BAUDRATE);

  initBluetooth();
  const uint8_t* point = esp_bt_dev_get_address();
  String bt_name = "BIOX_AAL_Band_";

  for (int i = 4; i < 6; i++) {
    char str[3];
    sprintf(str, "%02X", (int)point[i]);
    bt_name += str;
  }

  SERIAL_BT.begin(bt_name);

  analogReadResolution(9);

  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 0);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //SERIAL_USB.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  bno.setExtCrystalUse(true);

  dacWrite(dac_pin, input_voltage);

  //   ////////////////////////////
  //   //                        //
  //   //     finish setup       //
  //   //        files           //
  //   ////////////////////////////

  start_up();
  timer = micros();

}



void loop()
{











  send_data = true;
  






  if (send_data)
  {
    if (micros() - timer_send >= send_time)
    {

      timer_send = micros();
      readFSR();
      //stack fsr
      for (uint8_t i = 0; i < 8; i++)
      {
        stack_fsr[stack_fsr_count] = data[i];
        stack_fsr_count = stack_fsr_count + 1;
      }
      //sendData_old_FSR();
      send_imu_counter = send_imu_counter + 1;
      if (send_imu_counter == 20)
      {
        timer = micros();
        readIMU();
        //stack imu
        sendData_old_FSR();
        sendData();
        //delay(10);
        stack_fsr_count = 0;
        if (stop_data)
        {
          send_data = false;
//          SERIAL.print('T');
//          SERIAL.println('T');
          stop_data = false;
        }

        if (micros() - timer < imu_read_time)
        {
          delayMicroseconds(imu_read_time - (micros() - timer));
        }
        send_imu_counter = 0;
      }
      transmitt_indication();
    }

  }
  else
  {
    digitalWrite(grn_pin, HIGH);
    send_imu_counter = 0;
  }

}





////////**********//////////
////////////////////////////
//                        //
//      Additional        //
//      Functions         //
////////////////////////////
///////***********//////////


void readFSR()
{
  data[0] = (DataType)(analogRead(32));
  data[1] = (DataType)(analogRead(33));
  data[2] = (DataType)(analogRead(34));
  data[3] = (DataType)(analogRead(35));
  data[4] = (DataType)(analogRead(36));
  data[5] = (DataType)(analogRead(37));
  data[6] = (DataType)(analogRead(38));
  data[7] = (DataType)(analogRead(39));
}

void readIMU()
{
  unsigned long imu_timer = 0;
  imu_start_timer = micros();
  imu::Vector<3> read_data_gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  data[8] = read_data_gravity.x();
  data[9] = read_data_gravity.y();
  data[10] = read_data_gravity.z();

  imu::Vector<3> read_data_gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  data[11] = read_data_gyro.x();
  data[12] = read_data_gyro.y();
  data[13] = read_data_gyro.z();

 // imu::Vector<3> read_data_liacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
   imu::Vector<3> read_data_euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  data[14] = read_data_euler.x();
  data[15] = read_data_euler.y();
  data[16] = read_data_euler.z();

  imu::Quaternion read_data_quat = bno.getQuat();
  data[17] = read_data_quat.x();
  data[18] = read_data_quat.y();
  data[19] = read_data_quat.z();
  data[20] = read_data_quat.w();

  imu_end_timer = micros();
  if ((imu_end_timer - imu_start_timer) < imu_loopTime)
  {
    delayMicroseconds(imu_loopTime - (imu_end_timer - imu_start_timer));
  }
}

/*void sendData_old_FSR()
  {
  for (uint8_t i = 0; i < (NUM_FSR); i++)
  {
    int data_fsr_int = 0;
    data_fsr_int = (int)(data[i] / 2.1);
    byte* byteData = (byte*)(data + i);
    buff_data_FSR[i] = lowByte(data_fsr_int);
    if (buff_data_FSR[i] > 254)
    {
      buff_data_FSR[i] = 254;
    }
  }
  SERIAL.write(buff_data_FSR, NUM_FSR);
  }*/
void sendData_old_FSR()
{
  for (uint8_t i = 0; i < 160; i++)
  {
    int data_fsr_int = 0;
    data_fsr_int = (int)(stack_fsr[i] / 2.1);
    byte* byteData = (byte*)(stack_fsr + i);
    buff_data_FSR[i] = lowByte(data_fsr_int);
    if (buff_data_FSR[i] > 254)
    {
      buff_data_FSR[i] = 254;
    }
  }
  Serial.write(buff_data_FSR, 160);
}



void sendData()
{
  uint16_t count = 0;
  for (uint8_t i = 8; i < (NUM_FSR + NUM_IMU); i++)
  {
    byte* byteData = (byte*)(data + i);
    for (uint8_t j = 0; j < sizeof(DataType); j++)
    {
      buff[count + j] = byteData[j];
    }
    count += sizeof(DataType); // sizeof(float)
  }
  Serial.write(buff, (NUM_IMU)*sizeof(DataType));

  memset(buff, 0, sizeof(buff)); // Clear the buffer
}

////////////////////////////
//                        //
//   condition for shut   //
//          down          //
////////////////////////////
void shut_down_condition()
{
  readFSR();
  net_fsr = (data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7]) / (8);
  if (digitalRead(0) == 0)
  {
    shut_down();
    esp_deep_sleep_start();
  }

  /*if (net_fsr < (input_voltage * 0.85 * 2.1 + 15))
    {
    if (pre_sleep_time == 0)
    {
      pre_sleep_time = millis();
    }
    else
    {
      current_time = millis();
      if ((current_time - pre_sleep_time) > sleep_time)
      {
        shut_down();
        esp_deep_sleep_start();
      }
    }
    }
    else
    {
    pre_sleep_time = 0;
    }*/
}

////////////////////////////
//                        //
//     initiate shut      //
//   down and start up    //
////////////////////////////
void shut_down()
{
  for (int i = 0; i <= 5; i++) {
    digitalWrite(red_pin, LOW);
    delay(500);
    digitalWrite(red_pin, HIGH);
    delay(500);
  }
}

void start_up()
{
  for (int i = 0; i <= 1; i++) {
    digitalWrite(red_pin, LOW);
    digitalWrite(grn_pin, LOW);
    delay(1500);
    digitalWrite(red_pin, HIGH);
    digitalWrite(grn_pin, HIGH);
    delay(500);
  }
}

////////////////////////////
//                        //
//       initiate         //
//     calibration        //
////////////////////////////



////////////////////////////
//                        //
//        charge          //
//      indication        //
////////////////////////////
void charge_indication()
{
  current_time = millis();
  if ((current_time - pre_charge_time) > charge_time)
  {
    digitalWrite(red_pin, LOW);
    delay(500);
    digitalWrite(red_pin, HIGH);
    pre_charge_time = current_time;
  }
}

////////////////////////////
//                        //
//    data transmitt      //
//      indication        //
////////////////////////////
void transmitt_indication()
{
  current_time = millis();
  if ((current_time - pre_transmit_time) > transmit_time)
  {
    digitalWrite(grn_pin, !digitalRead(grn_pin));
    pre_transmit_time = current_time;
  }
}







//// functionalities to add in the code
//1- use charge pin to indicate if it is charging--
//2- Command to start sarting and stoping data transmission--
//3- Command to increase or decrease input voltage--
//4- sleep mode when FSR sensors are silent for more then 10 seconds--.


// BATTERY NOT Attached - code stuck in charging indication  - RED LED BLNK
