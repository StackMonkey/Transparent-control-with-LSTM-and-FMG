/*
 * Bluetooth Classic Example
 * Scan for devices - asynchronously, print device as soon as found
 * query devices for SPP - SDP profile
 * connect to first device offering a SPP connection
 *
 * Example python server:
 * source: https://gist.github.com/ukBaz/217875c83c2535d22a16ba38fc8f2a91
 *
 * Tested with Raspberry Pi onboard Wifi/BT, USB BT 4.0 dongles, USB BT 1.1 dongles,
 * 202202: does NOT work with USB BT 2.0 dongles when esp32 arduino lib is compiled with SSP support!
 *         see https://github.com/espressif/esp-idf/issues/8394
 *
 * use ESP_SPP_SEC_ENCRYPT|ESP_SPP_SEC_AUTHENTICATE in connect() if remote side requests 'RequireAuthentication': dbus.Boolean(True),
 * use ESP_SPP_SEC_NONE or ESP_SPP_SEC_ENCRYPT|ESP_SPP_SEC_AUTHENTICATE in connect() if remote side has Authentication: False
 */

#include <map>
#include <BluetoothSerial.h>
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"

char command = 'T';
bool initBluetooth(const char *deviceName) {
	if (!btStart()) {
		Serial.println("Failed to initialize controller");
		return false;
	}

	if (esp_bluedroid_init() != ESP_OK) {
		Serial.println("Failed to initialize bluedroid");
		return false;
	}

	if (esp_bluedroid_enable() != ESP_OK) {
		Serial.println("Failed to enable bluedroid");
		return false;
	}

	esp_bt_dev_set_device_name(deviceName);

	esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
}
BluetoothSerial SerialBT;

#define BT_DISCOVER_TIME 10000
esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;  // or ESP_SPP_SEC_ENCRYPT|ESP_SPP_SEC_AUTHENTICATE to request pincode confirmation
esp_spp_role_t role = ESP_SPP_ROLE_SLAVE;   // or ESP_SPP_ROLE_MASTER

// std::map<BTAddress, BTAdvertisedDeviceSet> btDeviceList;

void setup() {
  Serial.begin(250000);
  SerialBT.begin("BluetoothTest"); //Bluetooth device name
  Serial.begin(250000); //Bluetooth device name
  char c = 'B';
  while (c != 'k')
  {
    if (SerialBT.available()){
      c = SerialBT.read();
    }
  }
  SerialBT.print('M');  // sending charachter to PS
}

float rightTorquePre = 0.f;
float leftTorquePre = 0.f;
long currentTime = 0;
long lastTime = 0;
float torque = 1.1;
byte sendData[2] = {};
void loop() {
  currentTime = millis();
  if(SerialBT.available()){
    command == SerialBT.read();
  }
  if(command == 'G'){
    read_string();
    Serial.println("new predictions:");
    Serial.println(rightTorquePre);
    Serial.println(leftTorquePre);
  }
  if (currentTime>lastTime+50)
  {
    torque = random(1000)/1000.0;
    sendData[0] = (byte)torque;
    sendData[1] = (byte)torque;
    SerialBT.print(torque, 6);
    SerialBT.print("\n");
    lastTime = currentTime;
  }
}
void read_string() {
	String stringone = "";
	String mc_sel = "";
	String gains_sel = "";
	String gains_val = "";
	int comma_index = 0;
	command = 'n';
	while (command != 'T') {
		if (SerialBT.available()) {
			command = SerialBT.read();
			if (command != 'T') {
				stringone += command;
			} else {
				mc_sel = stringone.substring(0, 3);
				gains_sel = stringone.substring(4);
				if (mc_sel == "rmp") {
					comma_index = gains_sel.indexOf(',', 0);
					gains_val = gains_sel.substring(0, comma_index);
					rightTorquePre = gains_val.toFloat();
          gains_sel = gains_sel.substring(comma_index + 1);
					comma_index = gains_sel.indexOf(',', 0);
					gains_val = gains_sel.substring(0, comma_index);
					leftTorquePre = gains_val.toFloat();
				}
			}
		}
	}
	command = 'n';
}