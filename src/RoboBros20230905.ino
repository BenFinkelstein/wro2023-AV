// Program code for the RoboBros 2023 automonous driving vehicle with a Arduino Nano Every,
// 3 VL53L0X TOF Sensors, 1 Color Sensor, 1 QMC5883L Magnetometer, 1 Servo and 1 DC Lego Mindstorm Large Motor
// by Benjamin Fineklstein Fell and Dimitri Domzalski.

#include "Adafruit_VL53L0X.h"
#include <Servo.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <QMC5883L.h>

#define LOX0_ADDRESS 0x30
#define LOX1_ADDRESS 0x31
#define LOX2_ADDRESS 0x32

#define SHT_LOX0 9
#define SHT_LOX1 10
#define SHT_LOX2 8

#define cSPEED 80
#define cDELAY 10 

#define cFRONT          0
#define cLEFT           1
#define cRIGHT          2
#define cNUMSENSORS     3
#define cNUMFILTERS     5
#define cCORNERANGLE    25
#define cTURNANGLE      10
#define cMINSTOPD       55
#define cCENTER         122
#define c8K             8000

#define cCORNERLEFT  cCENTER - cCORNERANGLE
#define cCORNERRIGHT cCENTER + cCORNERANGLE

#define INTERRUPT_PIN 2

Adafruit_VL53L0X lox[cNUMSENSORS];

VL53L0X_RangingMeasurementData_t g_aSensorVal[cNUMSENSORS];

int enA = 3;
int in1 = 6;
int in2 = 4;
 
bool   g_bPrint;
int    g_iCount8k[cNUMSENSORS], g_iCurPOS, g_iFilterLoop, g_iSensorLoop, g_iCorner, g_iX, g_iY, g_iZ, g_iLastTurn;
float  g_fOldHeadingDegrees, g_fNewHeadingDegrees;
double g_aAverages[cNUMSENSORS], g_dFactor;

int g_aFilterSensorVal[cNUMFILTERS][cNUMSENSORS], g_aSensor[cNUMSENSORS];

Servo g_steer;

QMC5883L compass;

void setID()
{
  digitalWrite(SHT_LOX0, LOW);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  digitalWrite(SHT_LOX0, HIGH);
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  digitalWrite(SHT_LOX0, HIGH);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  if (!lox[cFRONT].begin(LOX0_ADDRESS))
  {
      Serial.println("Failed to boot first VL53L0X @ LOX1_ADDRESS");
      while (true);
  }

  digitalWrite(SHT_LOX1, HIGH);
  delay(10);

  if (!lox[cLEFT].begin(LOX1_ADDRESS))
  {
      Serial.println("Failed to boot second VL53L0X @ LOX1_ADDRESS");    
      while (true);
  }

  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  if (!lox[cRIGHT].begin(LOX2_ADDRESS))
  {
      Serial.println("Failed to boot third VL53L0X  @ LOX2_ADDRESS");
      while (true);
  }
}

void DCRun()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  analogWrite(enA, cSPEED);
}

void DCStop()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void CheckSTOP()
{
  if (g_aSensor[cFRONT] < cMINSTOPD)
  {
      DCStop();

      if (g_bPrint)
          Serial.println("      STOPING...");
      
      while (true);
  }
}

void setup()
{
  g_bPrint = true;

  g_iCurPOS = cCENTER;
  g_iLastTurn = cFRONT;

  Serial.begin(115200);

  while (!Serial) delay(1);
    
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(SHT_LOX0, OUTPUT);
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println("Shutdown pins inited...");

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  

  digitalWrite(SHT_LOX0, LOW);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println("Both in reset mode...(pins are low)");
  Serial.println("Starting...");           
  Serial.println();
  
  setID();

  compass.init();

  g_steer.attach(5);

  g_steer.write(cCENTER);

  delay(200);

  compass.read(&g_iX, &g_iY, &g_iZ);

  g_fNewHeadingDegrees = ((atan2(g_iY, g_iX) * 180) / PI) + 1.13;

  if (g_fNewHeadingDegrees < 0)
      g_fNewHeadingDegrees += 360;

  DCRun();
}

void PrintValues()
{
  if (g_bPrint)
  {
      Serial.print(" >< ");
      Serial.println(g_aSensor[cFRONT]);
      Serial.print(" << ");
      Serial.println(g_aSensor[cLEFT]);
      Serial.print(" >> ");
      Serial.println(g_aSensor[cRIGHT]);
      Serial.println();

      if (g_iCorner == cFRONT)
      {
        Serial.print(" || g_dFactor: ");
        Serial.println(g_dFactor);        
        Serial.println();

        Serial.print(" ~~ Swerve ");

        if (g_iLastTurn != cFRONT)
        {
            if (g_dFactor < 0.0)
                Serial.print("Left...");
            else
                Serial.print("Right...");

            Serial.print(" Last turn: ");

            if (g_iLastTurn == cLEFT)
                Serial.print("Left");
            else
                Serial.print("Right");
        }

        Serial.println();

      }
      else
      {
        Serial.print(" )( Cornering ");
      
        if (g_iCorner == cLEFT)
            Serial.println("Left...");
        else
            Serial.println("Right...");
      }
          
      Serial.println();
  }
}

void DCCenter()
{
  if (g_iCurPOS > cCENTER)
      for (; g_iCurPOS > cCENTER;g_iCurPOS--)
      {
          Serial.print("    DCCeter: ");
          Serial.println(g_iCurPOS);         
          g_steer.write(g_iCurPOS);
          delay(10);
      }
  else
      for (; g_iCurPOS < cCENTER;g_iCurPOS++)
      {
          g_steer.write(g_iCurPOS);
          delay(10);
      }

  Serial.println();    
}

void Swerve(int iSwerve)
{
  /*Serial.print("    Swerve: ");
  Serial.print(iSwerve);
  Serial.print("  ");
  Serial.println((cCENTER - iSwerve));*/
 
  if (g_dFactor < 0.0)
  {
      if (g_iLastTurn != cLEFT)
      {
          for (; g_iCurPOS > (cCENTER - iSwerve); g_iCurPOS--)
          {
              g_steer.write(g_iCurPOS);
              delay(10);

            if (g_bPrint)
            {
              Serial.print("Swerving Left: ");
              Serial.println(g_iCurPOS);
            }
          }

          g_iLastTurn = cLEFT;
      }
  }
  else if (g_iLastTurn != cRIGHT)
  {
      for (; g_iCurPOS < (cCENTER + iSwerve);g_iCurPOS++)
      {
          g_steer.write(g_iCurPOS++);
          delay(10);
          
          if (g_bPrint)
          {
            Serial.print("Swerving Right: ");
            Serial.println(g_iCurPOS);
          }
      }

      g_iLastTurn = cRIGHT;
  }

  if (g_bPrint)
      Serial.println();

  DCCenter();
  delay(cDELAY);
}

void Corner()
{
  DCStop();
  delay(cDELAY * 100);
  //DCRun();
  while (true);

  g_fOldHeadingDegrees = g_fNewHeadingDegrees;

  if (g_iCorner == cLEFT)   
      do
      {
        compass.read(&g_iX, &g_iY, &g_iZ);

        g_fNewHeadingDegrees = ((atan2(g_iY, g_iX) * 180) / PI) + 1.13;

        if (g_fNewHeadingDegrees < 0)
            g_fNewHeadingDegrees += 360;

        if (g_iCurPOS > cCORNERLEFT)
            g_steer.write(--g_iCurPOS);
      
        delay(10);

      } while (g_fNewHeadingDegrees > (g_fOldHeadingDegrees - 45));
    else
      do
      {
        compass.read(&g_iX, &g_iY, &g_iZ);

        g_fNewHeadingDegrees = ((atan2(g_iY, g_iX) * 180) / PI) + 1.13;

        if (g_fNewHeadingDegrees < 0)
            g_fNewHeadingDegrees += 360;

        if (g_iCurPOS < cCORNERRIGHT)
            g_steer.write(++g_iCurPOS);

        delay(10);
 
      } while (g_fNewHeadingDegrees < (g_fOldHeadingDegrees + 45));

    if (g_bPrint)
    {
        Serial.print("g_iCurPOS: ");
        Serial.print(g_iCurPOS);
        Serial.print("  g_fOldHeadingDegrees: ");
        Serial.print(g_fOldHeadingDegrees);
        Serial.print("  g_fNewHeadingDegrees: ");
        Serial.println(g_fNewHeadingDegrees);
        Serial.println();
    }

  g_fOldHeadingDegrees = g_fNewHeadingDegrees;

  delay(cDELAY);
}

void loop()
{
  g_iCorner = cFRONT;

  for (g_iSensorLoop = 0; g_iSensorLoop < cNUMSENSORS; g_iSensorLoop++)
  {
      g_iCount8k[g_iSensorLoop] = 0;
      g_aAverages[g_iSensorLoop] = 0; 
    
      for (g_iFilterLoop = 0; g_iFilterLoop < cNUMFILTERS && g_iCorner == cFRONT; g_iFilterLoop++)
          g_aFilterSensorVal[g_iFilterLoop][g_iSensorLoop] = 0;
  }

  for (g_iFilterLoop = 0; g_iFilterLoop < cNUMFILTERS && g_iCorner == cFRONT; g_iFilterLoop++)
  {
    for (g_iSensorLoop = 0; g_iSensorLoop < cNUMSENSORS && g_iCorner == cFRONT; g_iSensorLoop++)
    {
        lox[g_iSensorLoop].rangingTest(&g_aSensorVal[g_iSensorLoop], false);
    
        if (g_aSensorVal[g_iSensorLoop].RangeMilliMeter > c8K)
        {
            g_iCount8k[g_iSensorLoop]++;

            if (g_iSensorLoop > cFRONT && g_iCount8k[g_iSensorLoop] > (cNUMFILTERS - 1))
                g_iCorner = g_iSensorLoop;
        }
        else
        {
            g_aFilterSensorVal[g_iFilterLoop][g_iSensorLoop] = g_aSensorVal[g_iSensorLoop].RangeMilliMeter;

            g_aAverages[g_iSensorLoop] += g_aFilterSensorVal[g_iFilterLoop][g_iSensorLoop];
        }

        if (g_bPrint)
        {
            Serial.print("[");
            Serial.print(g_iFilterLoop);
            Serial.print(", ");
            Serial.print(g_iSensorLoop);
            Serial.print("] ");

            if (g_iCorner == cFRONT)
            {
                Serial.print(g_aFilterSensorVal[g_iFilterLoop][g_iSensorLoop]);
                Serial.print("-> ");
                Serial.print(g_aAverages[g_iSensorLoop]);
            }
            else
                Serial.print("Possible Corner ");

            Serial.print("-> ");
            Serial.println(g_iCount8k[g_iSensorLoop]);
        }
    }

    if (g_bPrint)
        Serial.println();
  }

  for (g_iSensorLoop = 0; g_iSensorLoop < cNUMSENSORS  && g_iCorner == cFRONT; g_iSensorLoop++)
      g_aSensor[g_iSensorLoop] = (int)(g_aAverages[g_iSensorLoop] / (cNUMFILTERS - g_iCount8k[g_iSensorLoop]));

  //CheckSTOP();

  g_dFactor = (double)(g_aSensor[cRIGHT] - g_aSensor[cLEFT]) / (double)(g_aSensor[cLEFT] + g_aSensor[cRIGHT]);
    
  PrintValues();

  if (g_iCorner == cFRONT)
      Swerve((int)(cTURNANGLE));// * fabs(g_dFactor)));
  else
      Corner();

  DCCenter();
 }