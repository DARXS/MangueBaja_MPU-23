#include <Arduino.h>
#include <Ticker.h>
#include <SD.h>
#include <EBYTE.h>
#include "hard_defs_rec.h"
#include "packets.h"

#define DEBUG

#define MB1 // Uncomment a line if it is your car choice
// #define MB2 // Uncomment a line if it is your car choice

#ifdef MB1
  #define CAR_ID MB1_ID
#endif

#ifdef MB2
  #define CAR_ID MB2_ID
#endif

unsigned long Last;

EBYTE Lora(&LoRaUART, PIN_M0, PIN_M1, PIN_AUX);

// SD variables
char file_name[20];
File root;
File dataFile;
// bool saveFlag                = 0x00;
uint8_t data[sizeof(volatile_packet)];

void setup()
{
  Serial.begin(115200);
  LoRaUART.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  if(!Lora.init())
  {
    Serial.println("LoRa ERROR!!");
    return;
  }

  Lora.SetAddressH(1);
  Lora.SetAddressL(1);
  Lora.SetChannel(MB1_ID);
  Lora.SetAirDataRate(ADR_1200);
  Lora.SetTransmitPower(OPT_TP30);
  Lora.SetMode(MODE_NORMAL);
  Lora.SetUARTBaudRate(UDR_9600);
  Lora.SetFECMode(OPT_FECENABLE);
  Lora.SaveParameters(PERMANENT);
  #ifdef DEBUG
    Lora.PrintParameters();
  #endif

  //memset(&volatile_packet, 0, sizeof(volatile_packet));
}

void loop()
{
  if(LoRaUART.available())
  {
    Lora.GetStruct(&volatile_packet, sizeof(volatile_packet));
    // saveFlag = 0x01;

    /* Write in Serial */
    // memcpy(&data, (uint8_t *)&volatile_packet, sizeof(volatile_packet));

    // Serial.write(CAR_ID); // CAR ID  

    // for(int i = 0; i < sizeof(data); i++)
    // {
    //   Serial.write(data[i]); 
    // }
    // Serial.write(0xff);

    digitalWrite(LED_BUILTIN, HIGH);

    // Loss = volatile_packet.cont;
    #ifdef DEBUG
      Serial.println("\n\n");
      Serial.print("Acc X: ");          Serial.println(volatile_packet.imu_acc.acc_x);
      Serial.print("Acc Y: ");          Serial.println(volatile_packet.imu_acc.acc_y);
      Serial.print("Acc Z: ");          Serial.println(volatile_packet.imu_acc.acc_z);
      Serial.print("DPS X: ");          Serial.println(volatile_packet.imu_dps.dps_x);
      Serial.print("DPS Y: ");          Serial.println(volatile_packet.imu_dps.dps_y);
      Serial.print("DPS Z: ");          Serial.println(volatile_packet.imu_dps.dps_z);
      Serial.print("RPM: ");            Serial.println(volatile_packet.rpm);
      Serial.print("SPEED: ");          Serial.println(volatile_packet.speed);
      Serial.print("TEMPERATURE: ");    Serial.println(volatile_packet.temperature);
      Serial.print("FLAGS: ");          Serial.println(volatile_packet.flags);
      Serial.print("SOC: ");            Serial.println(volatile_packet.SOC);
      Serial.print("CVT: ");            Serial.println(volatile_packet.cvt);
      Serial.print("VOLT: ");           Serial.println(volatile_packet.volt);
      Serial.print("LATITUDE: ");       Serial.println(volatile_packet.latitude);
      Serial.print("LONGITUDE: ");      Serial.println(volatile_packet.longitude);
      Serial.print("Times: ");          Serial.println(volatile_packet.timestamp);
      //Serial.print("Cont. Transm: ");   Serial.println(volatile_packet.cont);
      //Serial.print("Cont. Rec: ");      Serial.println(contador);

      //contador++;
    #endif

    digitalWrite(LED_BUILTIN, LOW);
    Last = millis();
  }
  else
  {
    if ((millis() - Last) > 1000)
    {
      // Serial.println("\n------------");
      Serial.print("Searching: \n");
      Last = millis();
      // saveFlag ? Loss++ : 0;
      // Serial.println(Loss);
      // Serial.print("Cont. Rec: ");
      // Serial.println(contador);
      // Serial.println("------------");
    }
  }
}

// void TaskSetup()
// {
//   xTaskCreatePinnedToCore(SdStateMachine, "SDStateMachine", 10000, NULL, 5, NULL, 0);
//   // This state machine is responsible for the Basic CAN logging and GPS logging
//   xTaskCreatePinnedToCore(RadioStateMachine, "RadioConectivityStateMachine", 10000, NULL, 5, NULL, 1);
//   // This state machine is responsible for the Radio communitation and possible bluetooth connection
// }

// //SD Functions
// void sdConfig()
// {
//   static bool mounted = false; // SD mounted flag

//   if(!mounted)
//   {
//     if(!SD.begin(SD_CS)) { return; }

//     root = SD.open("/");
//     int num_files = countFiles(root);
//     sprintf(file_name, "/%s%d.csv", "data", num_files + 1);

//     dataFile = SD.open(file_name, FILE_APPEND);

//     if(dataFile)
//     {
//       dataFile.println(packetToString(mounted));
//       dataFile.close();
//     } else {
//       Serial.println(F("FAIL TO OPEN THE FILE"));
//     }
//     mounted = true;
//   }
//   sdSave();
// }

// int countFiles(File dir)
// {
//   int fileCountOnSD = 0; // for counting files
//   for(;;)
//   {
//     File entry = dir.openNextFile();
//     if (!entry)
//     {
//       // no more files
//       break;
//     }
//     // for each file count it
//     fileCountOnSD++;
//     entry.close();
//   }

//   return fileCountOnSD - 1;
// }

// void sdSave()
// {
//   dataFile = SD.open(file_name, FILE_APPEND);

//   if(dataFile)
//   {
//     dataFile.println(packetToString());
//     dataFile.close();
//   } else {
//     Serial.println(F("falha no save"));
//   }
// }

// String packetToString(bool err)
// {
//   String dataString = "";
//     if(!err)
//     {
//       dataString += "Cont";
//       dataString += ",";
//       dataString += "ACCX";
//       dataString += ",";
//       dataString += "ACCY";
//       dataString += ",";
//       dataString += "ACCZ";
//       dataString += ",";
//       dataString += "DPSX";
//       dataString += ",";
//       dataString += "DPSY";
//       dataString += ",";
//       dataString += "DPSZ";
//       dataString += ",";
//       dataString += "RPM";
//       dataString += ",";
//       dataString += "VEL";
//       dataString += ",";
//       dataString += "TEMP_MOTOR";
//       dataString += ",";
//       dataString += "FLAGS";
//       dataString += ",";
//       dataString += "SOC";
//       dataString += ",";
//       dataString += "TEMP_CVT";
//       dataString += ",";
//       dataString += "VOLT";
//       dataString += ",";
//       dataString += "LATITUDE";
//       dataString += ",";
//       dataString += "LONGITUDE";
//       dataString += ",";
//       dataString += "TIMESTAMP";
//       dataString += ",";
//       dataString += "ID=" + String(CAR_ID);
//     }

//     else
//     {
//       // imu,
//       dataString += String(volatile_packet.cont);
//       dataString += String(volatile_packet.imu_acc.acc_x);
//       dataString += ",";
//       dataString += String(volatile_packet.imu_acc.acc_y);
//       dataString += ",";
//       dataString += String(volatile_packet.imu_acc.acc_z);
//       dataString += ",";
//       dataString += String(volatile_packet.imu_dps.dps_x);
//       dataString += ",";
//       dataString += String(volatile_packet.imu_dps.dps_y);
//       dataString += ",";
//       dataString += String(volatile_packet.imu_dps.dps_z);
//       dataString += ",";

//       dataString += String(volatile_packet.rpm);
//       dataString += ",";
//       dataString += String(volatile_packet.speed);
//       dataString += ",";
//       dataString += String(volatile_packet.temperature);
//       dataString += ",";
//       dataString += String(volatile_packet.SOC);
//       dataString += ",";
//       dataString += String(volatile_packet.cvt);
//       dataString += ",";
//       dataString += String(volatile_packet.volt);
//       dataString += ",";
//       dataString += String(volatile_packet.flags);
//       dataString += ",";
//       dataString += String(volatile_packet.latitude);
//       dataString += ",";
//       dataString += String(volatile_packet.longitude);
//       dataString += ",";
//       dataString += String(volatile_packet.timestamp);
//     }

//   return dataString;
// }

// /* SD State Machine */
// void SdStateMachine(void *pvParameters)
// {
//   while(1)
//   {
//     if(saveFlag)
//     {
//       sdConfig();
//       saveFlag = false;
//     }
//     vTaskDelay(1);
//   }
// }

// void RadioStateMachine(void *pvParameters)
// {
//   while(1)
//   {
//     if (Serial2.available()) {
//     Lora.GetStruct(&volatile_packet, sizeof(volatile_packet));

//     digitalWrite(LED_BUILTIN, HIGH);

//     Serial.println("------------");
//     Serial.print("Acc X: ");          Serial.println(volatile_packet.imu_acc.acc_x);
//     Serial.print("Acc Y: ");          Serial.println(volatile_packet.imu_acc.acc_y);
//     Serial.print("Acc Z: ");          Serial.println(volatile_packet.imu_acc.acc_z);
//     Serial.print("DPS X: ");          Serial.println(volatile_packet.imu_dps.dps_x);
//     Serial.print("DPS Y: ");          Serial.println(volatile_packet.imu_dps.dps_y);
//     Serial.print("DPS Z: ");          Serial.println(volatile_packet.imu_dps.dps_z);
//     Serial.print("RPM: ");            Serial.println(volatile_packet.rpm);
//     Serial.print("SPEED: ");          Serial.println(volatile_packet.speed);
//     Serial.print("TEMPERATURE: ");    Serial.println(volatile_packet.temperature);
//     Serial.print("FLAGS: ");          Serial.println(volatile_packet.flags);
//     Serial.print("SOC: ");            Serial.println(volatile_packet.SOC);
//     Serial.print("CVT: ");            Serial.println(volatile_packet.cvt);
//     Serial.print("VOLT: ");           Serial.println(volatile_packet.volt);
//     Serial.print("LATITUDE: ");       Serial.println(volatile_packet.latitude);
//     Serial.print("LONGITUDE: ");      Serial.println(volatile_packet.longitude);
//     Serial.print("Times: ");          Serial.println(volatile_packet.timestamp);

//     digitalWrite(LED_BUILTIN, LOW);
//     Last = millis();
//   }
//   else {
//     if ((millis() - Last) > 1000) {
//       Serial.println("Searching: ");
//       Last = millis();
//     }
//   }
//     vTaskDelay(1000);
//   }
// }

// void ticker1HzISR()
// {
//   saveFlag = true;
// }