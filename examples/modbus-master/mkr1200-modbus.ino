/*
 SigFox simple modbus Master Configuration
 This sketch demonstrates the usage of MKRFox1200 SigFox module.
 Since the board is designed with low power in mind, it depends directly on ArduinoLowPower library

 (c) 2021 by Stefan Denninger
*/

#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <ArduinoModbus.h>

// How often (in milliseconds) the sensors will be read.
const unsigned long REPORT_INTERVAL = 5000;
const unsigned long SENDING_INTERVAL =  660000;   // 11min = 660sec = 660000ms

float Volt_a;
float kWh_tot;
uint16_t kWh_tot1;
uint16_t kWh_tot2;

const unsigned long SIGFOX_MESSAGE_LENGTH = 5;

// stolen from https://github.com/arduino-libraries/SigFox/blob/master/examples/WeatherMonitor/WeatherMonitor.ino
typedef struct __attribute__ ((packed)) sigfox_message {
  uint8_t status;
  uint16_t val1;
  uint16_t val2;
} SigfoxMessage;

// stub for message which will be sent
SigfoxMessage msg;

void setup() {
  Serial.begin(9600);
  //while (!Serial) {};    // wait for serial monitor started - deactivated!
  delay(1000);

  if (!SigFox.begin()) {
    Serial.println("Shield error or not present!");
    return;
  }
  // Enable debug led and disable automatic deep sleep
  // Comment this line when shipping your project :)
  SigFox.debug();
  String version = SigFox.SigVersion();
  String ID = SigFox.ID();
  String PAC = SigFox.PAC();

  // Display module informations
  Serial.println("MKRFox1200 Sigfox - modbus Master configuration");
  Serial.println("SigFox FW version " + version);
  Serial.println("ID  = " + ID);
  Serial.println("PAC = " + PAC);
  Serial.println("");
  Serial.print("Module temperature: ");
  Serial.println(SigFox.internalTemperature());
  delay(100);
  // Send the module to the deepest sleep
  SigFox.end();


  Serial.println("Starting ModbusRTU 9600,8,N,1 Master");    // start the Modbus RTU client
  if (!ModbusRTUClient.begin(9600, SERIAL_8N1)) 
  {      
    Serial.println("Failed to start Modbus RTU Client!");
    while (1);
  }   
}  // end setup()


// The time at which the sensors were last read.
unsigned long lastMillis_report = 1000000;
unsigned long lastMillis_sending = 1000000;

//****************************************************************************
//****************************************************************************
void loop()
{
  // If enough time has elapsed, read again.
  if (millis() - lastMillis_report > REPORT_INTERVAL) {
    lastMillis_report = millis();

    read_modbus();
  }
  if (millis() - lastMillis_sending > SENDING_INTERVAL) {
    lastMillis_sending = millis();

    send_values();
  }
}

//****************************************************************************
void send_values() { 
  Serial.println("Sending values!");

   //sigfox_sendFloat(kWh_tot);
  msg.status = 1;
  sigfox_sendMsg();
}

//****************************************************************************
void read_modbus() {
      // The Modbus RTU  sensor:
    // Address: 0x01
    // Holding Register: 0x00
    // Read Length: 2
    // V_a = result[0]
    // ??? = result[1]
    
  if (!ModbusRTUClient.requestFrom(0x01, HOLDING_REGISTERS, 0x1300, 1))    // Device 1, Addr 0x1300, count 1 = 16bit!
  {
    Serial.print("failed to read registers! ");
    Serial.println(ModbusRTUClient.lastError());
  } 
  else 
  {
    int16_t Volt_a_raw = ModbusRTUClient.read();      
    Volt_a = Volt_a_raw / 10.0;                   
    Serial.print("Volt_a :  ");                 
    //Serial.print(Volt_a_raw);  // 2301               
    //Serial.print("  ");                 
    Serial.print(Volt_a);                                                                       
    Serial.println(" Volt");
  }

  if (!ModbusRTUClient.requestFrom(0x01, HOLDING_REGISTERS, 0x1142, 2))   // Device 1, Addr 0x1142 = Float wKh_tot, count 2 = 2x16bit!
  {
    Serial.print("failed to read registers! ");
    Serial.println(ModbusRTUClient.lastError());
  } 
  else 
  {
    uint16_t rawValue = ModbusRTUClient.read();      
    uint16_t rawValue2 = ModbusRTUClient.read();      
    kWh_tot1 = rawValue;
    kWh_tot2 = rawValue2;
    msg.val1 = rawValue;
    msg.val2 = rawValue2;
    kWh_tot = modbus_16bit_register_pair_to_float(rawValue2, rawValue);   // convert the two registers to one float!
    Serial.print("kWh_tot :  ");                 
    Serial.print(kWh_tot, 6);         // show value at console with 6 digits
    Serial.println(" kWh total");
  }
  delay(100);
} //end read modbus

//****************************************************************************
//*****  convert 2 int16 values to float
float modbus_16bit_register_pair_to_float(uint16_t a, uint16_t b) {
    uint32_t combined = ((uint32_t)a << 16) | b;
    float f;
    memcpy(&f, &combined, sizeof f);
    return f;
}

//****************************************************************************
void sigfox_sendMsg() {
  // Start the module
  SigFox.begin();
  // Wait at least 30mS after first configuration (100mS before)
  delay(100);
  // Clears all pending interrupts
  SigFox.status();
  delay(1);
  SigFox.beginPacket();
  SigFox.write((uint8_t*)&msg, SIGFOX_MESSAGE_LENGTH);
  int ret = SigFox.endPacket();  // send buffer to SIGFOX network
  if (ret > 0) {
    Serial.println("No transmission");
  } else {
    Serial.println("Transmission ok");
  }
  Serial.println(SigFox.status(SIGFOX));
  Serial.println(SigFox.status(ATMEL));
  SigFox.end();
}

//****************************************************************************
//*********************************** nice for helo world ;)
void sigfox_sendString(String str) {
  // Start the module
  SigFox.begin();
  // Wait at least 30mS after first configuration (100mS before)
  delay(100);
  // Clears all pending interrupts
  SigFox.status();
  delay(1);
  SigFox.beginPacket();
  SigFox.print(str);
  int ret = SigFox.endPacket();  // send buffer to SIGFOX network
  if (ret > 0) {
    Serial.println("No transmission");
  } else {
    Serial.println("Transmission ok");
  }
  Serial.println(SigFox.status(SIGFOX));
  Serial.println(SigFox.status(ATMEL));
  SigFox.end();
}

//****************************************************************************
void sigfox_sendStringAndGetResponse(String str) {
  // Start the module
  SigFox.begin();
  // Wait at least 30mS after first configuration (100mS before)
  delay(100);
  // Clears all pending interrupts
  SigFox.status();
  delay(1);
  SigFox.beginPacket();
  SigFox.print(str);
  int ret = SigFox.endPacket(true);  // send buffer to SIGFOX network and wait for a response
  if (ret > 0) {
    Serial.println("No transmission");
  } else {
    Serial.println("Transmission ok");
  }
  Serial.println(SigFox.status(SIGFOX));
  Serial.println(SigFox.status(ATMEL));
  if (SigFox.parsePacket()) {   Serial.println("Response from server:");    
  while (SigFox.available()) {
     Serial.print("0x");
     Serial.println(SigFox.read(), HEX);
   }
 } else {
    Serial.println("Could not get any response from the server");
    Serial.println("Check the SigFox coverage in your area");
    Serial.println("If you are indoor, check the 20dB coverage or move near a window");
  }
  Serial.println();
  SigFox.end();
}
