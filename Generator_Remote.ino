#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h> 
#include "secrets.h"
#include "LedControl.h"
#include <OneButton.h>
#include <uTimerLib.h>

// Seven Segment Displays (SSD's)
LedControl lc=LedControl(12,11,10,2);

// WiFi Config
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int status  = WL_IDLE_STATUS;
WiFiClient wifiClient;

// MQTT Config
const char* cerbogx = "172.16.10.20";
PubSubClient mqtt(wifiClient);



// Globals
bool remote = true;
const char* ON   = "1";
const char* OFF  = "0";
bool gen_enable   = 0;
bool gen_start    = 0;
bool gen_running  = 0;
unsigned long timer = 0;
int enable_start_offset = 1000;
unsigned long duration = 4000 + enable_start_offset;
const int gen_enable_pin = 9;
const int gen_start_pin = 8;
const int btn_start_pin = 2;
const int btn_stop_pin = 3;
const int led_enable = 5;
const int led_start = 4;
const int led_soc = 6;

// Buttons
OneButton btn_start = OneButton(
  btn_start_pin,    // Pin
  true, // Active LOW
  true  // Enable internal Pull-up
);
int btn_enable_pressed_ms = 0;

OneButton btn_stop = OneButton(
  btn_stop_pin,    // Pin
  true, // Active LOW
  true  // Enable internal Pull-up
);

// NonBlocking Voltage/SOC switch Variables
int soc_state = LOW;
unsigned long soc_prev_millis = 0;
const long soc_interval = 3000;
float soc_voltage_value = 0.00;

/* SETUP
 * ========================================================================= */
void setup() {
  Serial.begin(115200);
  //while (!Serial) {}; delay(1000);
  if(Serial.available() > 0) { delay(1000); }

  // WiFi Setup
  while (status != WL_CONNECTED) {
    Serial.print("Connecting to SSID: "); Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(1000);
  }
  printWifiStatus();

  // MQTT Setup
  mqtt.setServer(cerbogx, 1883);
  mqtt.setCallback(mqttCallback);

  // SSD Setup
  /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  lc.shutdown(0,false);
  lc.shutdown(1, false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,4);
  lc.setIntensity(1,4);

  /* and clear the display */
  lc.clearDisplay(0);
  lc.clearDisplay(1);

  // Button Setup
  btn_start.attachLongPressStop(handleBtnStart, &btn_start);
  btn_stop.attachLongPressStop(handleBtnStop, &btn_stop);
  //btn_enable.attachLongPressStop(btnEnableStop, &btn_enable);
  //btn_start.attachLongPressStop(btnStartStop, &btn_start);

  //btn_start.attachLongPressStop(handleStart);

  // Relay Pins
  pinMode(gen_enable_pin, OUTPUT);
  pinMode(gen_start_pin, OUTPUT);

  // LED Pins
  pinMode(led_enable, OUTPUT);
  pinMode(led_start, OUTPUT);
  pinMode(led_soc, OUTPUT);

  // Manually set LOW on boot
  digitalWrite(gen_start_pin, LOW);
  digitalWrite(gen_enable_pin, LOW);

  // Publish FALSE to turn off at boot.
  delay(500);
  mqttPublish("N/c0619ab48d75/bsr/generator/enable", false);
  mqttPublish("N/c0619ab48d75/bsr/generator/start", false);
}

/* LOOP
 * ========================================================================= */
void loop() {
  if (!mqtt.connected()) {
    reconnect();
  }
  mqtt.loop();
  btn_start.tick();
  btn_stop.tick();

  // Handle Generator start timer
  if(timer != 0 && (millis() - timer) >= duration)  {
    Serial.print("Timer Expired: "); Serial.print(timer);
    Serial.print(" millis: "); Serial.print(millis());
    Serial.print(" t+d: "); Serial.println(timer+duration);
    digitalWrite(gen_start_pin, LOW);
    mqttPublish("N/c0619ab48d75/bsr/generator/start", false);
    timer = 0;
  }

  // Voltage / SOC Swap
  unsigned long soc_current_millis = millis();
  if(soc_current_millis - soc_prev_millis >= soc_interval) {
    soc_prev_millis = soc_current_millis;
  
    if (soc_state == LOW) {
      soc_state = HIGH;
      clearSSD(2);
      float soc = calculateSoC(soc_voltage_value);
      ssdPrintFloat(2, soc);
      
    } else {
      soc_state = LOW;
      clearSSD(2);
      ssdPrintFloat(2, soc_voltage_value);
    }
    digitalWrite(led_soc, soc_state);
  }
  

}

void handleBtnStart(void *oneButton) {
  if (!gen_enable) {
    Serial.println("Enabling generator");
    mqttPublish("N/c0619ab48d75/bsr/generator/enable", true);
  } else if (gen_enable) {
    Serial.println("Starting Generator");
    mqttPublish("N/c0619ab48d75/bsr/generator/start", true);
    timer = millis();
  }
  //Serial.println("Start Button Pushed");
  //mqttPublish("N/c0619ab48d75/bsr/generator/enable", true);
  //runStart();
  // delay(enable_start_offset);
  // mqttPublish("N/c0619ab48d75/bsr/generator/start", true);
  // timer = millis();
}

void runStart() {
  delay(enable_start_offset);
  mqttPublish("N/c0619ab48d75/bsr/generator/start", true);
  timer = millis();
}

void handleBtnStop(void *oneButton) {
  Serial.println("Stop Button Pushed");
  mqttPublish("N/c0619ab48d75/bsr/generator/enable", false);
  mqttPublish("N/c0619ab48d75/bsr/generator/start", false);
}

// void btnEnableStop(void *oneButton) {
//   //Serial.print(((OneButton *)oneButton)->getPressedMs());
//   Serial.println("Enable Pushed");
//   //mqtt.publish("N/c0619ab48d75/bsr/generator/enable", ON);
//   mqttPublish("N/c0619ab48d75/bsr/generator/enable", !gen_enable);
// }

// void btnStartStop(void *oneButton) {
//   Serial.println("Start Pushed");
//   mqttPublish("N/c0619ab48d75/bsr/generator/start", !gen_start);
// }


//void btnEnableDuringPress(void *oneButton) {
//  Serial.print(((OneButton *)oneButton)->getPressedMs());
//  Serial.println("\t - DuringLongPress()");
//  return;
//}

void mqttPublish(char* topic, bool val) {
  Serial.print("Publishing Data: "); Serial.print(topic); Serial.print(": "); Serial.println(val);
  StaticJsonDocument<200> doc;
  doc["value"] = val;
  char buffer[256];
  size_t n = serializeJson(doc, buffer);
  mqtt.publish(topic, buffer, n);
}

/* mqttCallback()
 * Fires when any subscribed topics are published from the server. 
 * Does a string compare on the topic and then processes each topic.
 =========================================================================== */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  //Serial.print("Message Received: "); Serial.println(topic);
  double payload_data = processPayload(payload, length);
  //Serial.print("Payload Data: ");
  //Serial.println(payload_data);


  // Process PV Yield and send to function
  if (strcmp(topic, "N/c0619ab48d75/solarcharger/0/Yield/Power") == 0) {
    //payload_data = int(634.00);   // TEST DATA

    //Serial.print("PV Yield: ");
    //Serial.println(payload_data);
    clearSSD(1);
    ssdPrintInt(1, int(payload_data));
    if (int(payload_data) < 1 ) { clearSSD(1); }  // Clear the SSD if value is zero.
  }

  // Process Generator Yield and send to function
  if(strcmp(topic, "N/c0619ab48d75/vebus/276/Ac/ActiveIn/P") == 0) {
    //payload_data = int(2122.23);

    //Serial.print("Generator Yield: ");
    //Serial.println(payload_data);
    clearSSD(0);
    ssdPrintInt(0, int(payload_data));
    if (int(payload_data) < 1 )  { clearSSD(0); }  // Clear the SSD if value is zero.
  }

  // Process System Voltage and send to function
  if(strcmp(topic, "N/c0619ab48d75/vebus/276/Dc/0/Voltage") == 0) {
    soc_voltage_value = payload_data;
  }


  // Generator Enable Change
  if(strcmp(topic, "N/c0619ab48d75/bsr/generator/enable") == 0) {
    bool tmp_val = boolIt(payload_data);
    Serial.print("generator/enable: "); Serial.println(tmp_val);
    gen_enable = tmp_val;

    if(gen_enable) {  // Generator is enabled
      Serial.println("Enabling Generator");
      digitalWrite(gen_enable_pin, HIGH);
      digitalWrite(led_enable, HIGH);
    } else if(!gen_enable) {
      Serial.println("Diabling Generator");
      digitalWrite(gen_enable_pin, LOW);
      digitalWrite(led_enable, LOW);
    }

  }

  // // Generator Start Change
  if(strcmp(topic, "N/c0619ab48d75/bsr/generator/start") == 0) {
    bool tmp_val = boolIt(payload_data);
    Serial.print("generator/start: "); Serial.println(tmp_val);
    gen_start = tmp_val;

    if(gen_start) {
      Serial.println("Starting Generator");
      digitalWrite(gen_start_pin, HIGH);
      digitalWrite(led_start, HIGH);
    } else if (!gen_start) {
      digitalWrite(led_start, LOW);
      digitalWrite(gen_start_pin, LOW);
    }
    // if(!gen_start) {
    //   Serial.println("Disable Generator START");
    //   digitalWrite(8, LOW);
    // } else if (gen_start) {
    //   Serial.println("Enable Generator START");
    //   digitalWrite(8, HIGH);
    //   TimerLib.setInterval_s(endStartRelay, 5);
    // }
  }

  // Generator Running Change
  if(strcmp(topic, "N/c0619ab48d75/bsr/generator/running") == 0) {
    bool tmp_val = boolIt(payload_data);
    Serial.print("generator/running: ");

    Serial.println(tmp_val);
    gen_running = tmp_val;
  }


}

bool boolIt(double val) {
  if(val == 1.00) { return true;  } else
  if(val == 0.00) { return false; }
}

/* processPayload()
 * Takes the json formatted byte array and extracts the double
 * from the "value" field.
 * ========================================================================= */
double processPayload(byte* payload, unsigned int length) {
  //Serial.println("processPayload() START");

  // Convert to char
  char data_string[length+1];
  memcpy(data_string, payload, length);
  data_string[length] = '\0';

  // Parse JSON
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, data_string);
  if (error) {
    return -1.00;
  }
  double value = doc["value"];

  return value;
}

/* reconnect()
 * Handles connecting to MQTT and reconnect if connection is lost
 * ========================================================================= */
void reconnect() {
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqtt.connect(clientId.c_str())) {

      Serial.println("connected");

      // Subscriptions
      mqtt.subscribe("N/c0619ab48d75/solarcharger/0/Yield/Power");    // PV Total Yield
      mqtt.subscribe("N/c0619ab48d75/vebus/276/Ac/ActiveIn/P");       // Generator Total Yield
      mqtt.subscribe("N/c0619ab48d75/vebus/276/Dc/0/Voltage");        // DC System Voltage
      mqtt.subscribe("N/c0619ab48d75/vebus/276/Soc");                 // State of Charge
      mqtt.subscribe("N/c0619ab48d75/bsr/generator/enable");          // Generator Enable
      mqtt.subscribe("N/c0619ab48d75/bsr/generator/start");           // Generator Start
      mqtt.subscribe("N/c0619ab48d75/bsr/generator/running");         // Generator Running
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/* printWifiStatus()
 * Shows wifi status on boot via serial terminal
 =========================================================================== */
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  delay(2000);
}

/* calculateSoC()
 * Very simple linear interpolation of battery voltage -> SOC
 * There are better ways to do this, but this is *close enough* for now
 =========================================================================== */
float calculateSoC(float voltage) {
    // LiFePO4 battery voltage characteristics
    float fullyChargedVoltage = 54.4; // Replace with your battery's fully charged voltage
    float nominalVoltage = 51.2; // Replace with your battery's nominal voltage
    float dischargedVoltage = 40.0; // Replace with your battery's discharged voltage

    // Calculate SoC based on voltage levels
    if (voltage >= fullyChargedVoltage) {
        return 100.0;
    } else if (voltage <= dischargedVoltage) {
        return 0.0;
    } else {
        // Linear interpolation between discharged and fully charged
        float voltageRange = fullyChargedVoltage - dischargedVoltage;
        float socRange = 100.0;
        float soc = socRange * (voltage - dischargedVoltage) / voltageRange;
        return soc;
    }
}

// SSD Functions

void clearSSD(int disp) {
  int addr;
  int d0, d1, d2, d3;

  // Set correct address and columns based on display number (disp)
  if(disp == 0) {
    addr = 0;
    d0 = 0; d1 = 1; d2 = 2, d3 = 3;
  } else if (disp == 1) {
    addr = 0;
    d0 = 4; d1 = 5; d2 = 6; d3 = 7;
  } else if (disp == 2) {
    addr = 1;
    d0 = 0; d1 = 1; d2 = 2, d3 = 3;
  }

  lc.setChar(addr, d0, '<SPACE>', false);
  lc.setChar(addr, d1, '<SPACE>', false);
  lc.setChar(addr, d2, '<SPACE>', false);
  lc.setChar(addr, d3, '<SPACE>', false);
}

/* ssdPrintFloat(int disp, double val)
 * disp: Display number. 0, 1 or 2
 * val: input as double. 
 * val will truncate to 2 digits and round up
 =========================================================================== */
void ssdPrintFloat(int disp, double val) {
  int addr;
  int d0, d1, d2, d3;
  int ones, tens, hundreds, thousands;
  int i_part, d_part;
  int i_part_len;
  //val = round(val);

  // Set correct address and columns based on display number (disp)
  if(disp == 0) {
    addr = 0;
    d0 = 0; d1 = 1; d2 = 2, d3 = 3;
  } else if (disp == 1) {
    addr = 0;
    d0 = 4; d1 = 5; d2 = 6; d3 = 7;
  } else if (disp == 2) {
    addr = 1;
    d0 = 0; d1 = 1; d2 = 2, d3 = 3;
  }

  i_part = (int)(val);
  i_part_len = countDigits(i_part);
  d_part = 100 * (val - i_part + 0.005);
  //Serial.print("Integer Part: "); Serial.print(i_part); Serial.print(" Len: "); Serial.println(i_part_len);
  //Serial.print("Decimal Part: "); Serial.println(d_part);

  // If Integer part is 3, we can assume the value is 100 %
  if(i_part_len >= 3) {
    lc.setDigit(addr, d0, 1, false);
    lc.setDigit(addr, d1, 0, false);
    lc.setDigit(addr, d2, 0, true);
    lc.setDigit(addr, d3, 0, false);
  } else { // True decimal. Split integer part and decimal part. 
    int i_o = (i_part / 10) % 10;
    int i_t = (i_part / 1) % 10;

    int d_o = (d_part / 10) % 10;
    int d_t = (d_part / 1) % 10;

    // Display ones and tens
    lc.setDigit(addr, d0, i_o, false);
    lc.setDigit(addr, d1, i_t, true);   // turn on decimal point

    // Display decimal places
    lc.setDigit(addr, d2, d_o, false);
    lc.setDigit(addr, d3, d_t, false);
  }


}

/* ssdPrintInt(int disp, int num)
 * disp: Display number. 0, 1 or 2
 * num: input as integer. 
 =========================================================================== */
void ssdPrintInt(int disp, int num) {
  int addr;
  int d0, d1, d2, d3;
  int ones, tens, hundreds, thousands;

  // Set correct address and columns based on display number (disp)
  if(disp == 0) {
    addr = 0;
    d0 = 0; d1 = 1; d2 = 2, d3 = 3;
  } else if (disp == 1) {
    addr = 0;
    d0 = 4; d1 = 5; d2 = 6; d3 = 7;
  } else if (disp == 2) {
    addr = 1;
    d0 = 0; d1 = 1; d2 = 2, d3 = 3;
  }

  thousands = (num / 1000) % 10; //Serial.print("Thousadnds: "); Serial.println(thousands);
  hundreds  = (num / 100) % 10;  //Serial.print("Hundreds: "); Serial.println(hundreds);
  tens      = (num / 10) % 10;   //Serial.print("Tens: "); Serial.println(tens);
  ones      = (num / 1) % 10;    //Serial.print("Ones: "); Serial.println(ones);

  // Display SSD and get rid of leading zeros
  if(num >= 1000) {
    lc.setDigit(addr, d0, thousands, false);
    lc.setDigit(addr, d1, hundreds, false);
    lc.setDigit(addr, d2, tens, false);
  } else if (num >= 100) {
    lc.setDigit(addr, d1, hundreds, false);
    lc.setDigit(addr, d2, tens, false);
  } else if (num >= 10) {
    lc.setDigit(addr, d2, tens, false);
  }
  lc.setDigit(addr, d3, ones, false);
}

/* countDigits()
 * exactly.
 =========================================================================== */
int countDigits(int num) {
  return ( 1 + log10(num) );
}
