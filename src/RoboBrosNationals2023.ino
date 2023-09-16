// Program code for the RoboBros 2023 Automonous Driving Vehicle with a Arduino Nano Every,
// 3 VL53L0X TOF Sensors, 1 LCD, 1 Servo and 1 DC Lego Mindstorm Large Motor
// by Benjamin Fineklstein Fell and Dimitri Domzalski.
// ARDI v2.0 20230915

#include "Adafruit_VL53L0X.h"
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include "Wire.h"

#define LOX0_ADDRESS 0x30
#define LOX0_ADDRESS 0x30
#define LOX1_ADDRESS 0x31
#define LOX2_ADDRESS 0x32

#define SHT_LOX0 9
#define SHT_LOX1 10
#define SHT_LOX2 8

#define cSPEED 99
#define cDELAY 50 

#define cFRONT 0
#define cLEFT  1
#define cRIGHT 2

#define cNUMSENSORS  3
#define cNUMSAMPLES  12

#define cSWERVEANGLE 9
#define cCORNERANGLE 17
#define cCENTER      126

#define c1K 1000
#define c8K 8000

#define cCANCEL cNUMSENSORS + 1
#define cTOFTOCALIBRATE cCANCEL

#define cSWERVELEFT  cCENTER - cSWERVEANGLE
#define cSWERVERIGHT cCENTER + cSWERVEANGLE

#define cCORNERLEFT  cCENTER - cCORNERANGLE
#define cCORNERRIGHT cCENTER + cCORNERANGLE

#define cMAXCORNERS 1
#define cMAXSWERVES 2
#define cMINSTOPDST 70

int enA = 3;
int in1 = 6;
int in2 = 4;

bool    g_bCorner, b_bPrintMonitor, g_bDisplayLCD;
uint8_t g_uStatusLox;
int     g_iSensors[cNUMSENSORS], g_iCount8k[cNUMSENSORS], g_iMin[cNUMSENSORS], g_iMax[cNUMSENSORS], g_iFilter, g_iSensor, g_iCanCorner, g_iCornerDir, g_iSwerveDir, g_iLastSwerveDir, g_iCanSwerve, g_iRangeMilliMeter;
double  g_dAverages[cNUMSENSORS];

VL53L0X_RangingMeasurementData_t g_vlSensorVal[cNUMSENSORS];

Adafruit_VL53L0X g_adaLox[cNUMSENSORS];

LiquidCrystal_I2C lcd(0x27,20,4);

Servo g_sSteer;


void setID()
{
  digitalWrite(SHT_LOX0, LOW);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(cDELAY);

  digitalWrite(SHT_LOX0, HIGH);
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(cDELAY);

  digitalWrite(SHT_LOX0, HIGH);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(cDELAY);

  if (!g_adaLox[cFRONT].begin(LOX0_ADDRESS))
  {
      Serial.println("Failed to boot first VL53L0X @ LOX0_ADDRESS");
      while (true);
  }

  digitalWrite(SHT_LOX1, HIGH);
  delay(cDELAY);

  if (!g_adaLox[cLEFT].begin(LOX1_ADDRESS))
  {
      Serial.println("Failed to boot second VL53L0X @ LOX1_ADDRESS");    
      while (true);
  }

  digitalWrite(SHT_LOX2, HIGH);
  delay(cDELAY);

  if (!g_adaLox[cRIGHT].begin(LOX2_ADDRESS))
  {
      Serial.println("Failed to boot third VL53L0X  @ LOX2_ADDRESS");
      while (true);
  }

  delay(cDELAY);

  for (g_iSensor = 0; g_iSensor < cNUMSENSORS; g_iSensor++)
  {
      g_adaLox[g_iSensor].startRangeContinuous();
      delay(cDELAY);
      g_adaLox[g_iSensor].startRange();
      delay(cDELAY);
  }
}

void DCRunFwd()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  analogWrite(enA, cSPEED);

  delay(cDELAY);
}

void DCRunBwd()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  analogWrite(enA, cSPEED);
  
  delay(cDELAY);
}

void DCStop()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  delay(cDELAY);
}

void DCCenter()
{
  g_sSteer.write(cCENTER);

  delay(cDELAY);
}

void lcdPrint(int x, int y, int iSensor)
{
  lcd.setCursor(x,y);

  if (iSensor > cFRONT || g_iSensors[iSensor] < c1K)
      lcd.print(g_iSensors[iSensor]);
  else
      lcd.print("OUT");
}

void lcdSensors()
{
  if (g_bDisplayLCD == true)
  {
    lcd.setCursor(0,0);
    lcd.print("LFT  RHT  FRT DN");
    lcd.setCursor(0,1);
    lcd.print("                ");

    lcdPrint(0,1,cLEFT);

    lcdPrint(5,1,cRIGHT);

    lcdPrint(10,1,cFRONT);

    lcd.setCursor(14,1);
  }
}

void PrintValues(bool bCorner)
{
  Serial.println();
  Serial.print(" >o< ");
  Serial.println(g_iSensors[cFRONT]);
  Serial.print(" <o< ");
  Serial.println(g_iSensors[cLEFT]);
  Serial.print(" >o> ");
  Serial.println(g_iSensors[cRIGHT]);
  Serial.println();

  if (bCorner)
  {
    Serial.print(" )( Cornering ");
  
    if (g_iCornerDir == cLEFT)
        Serial.println("Left...");
    else
        Serial.println("Right...");
  }
  else
  {
      Serial.println();
      Serial.print(" ~o~ Swerve ");

      if (g_iSwerveDir == cLEFT)
          Serial.println("Left...");
      else
          Serial.println("Right...");
  }

  Serial.println();
}

void setup()
{
  g_bDisplayLCD = true,
  
  b_bPrintMonitor = false;
  
  g_iCanCorner = 0;
  
  g_iCanSwerve = 0;

  g_iCornerDir = cFRONT;

  g_iLastSwerveDir = cFRONT;

  lcd.init();

  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Hi! I'm ARDI 2.0");
  lcd.setCursor(0,1);
  lcd.print("by RoboBros 2023");

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
  delay(cDELAY);

  digitalWrite(SHT_LOX0, LOW);
  delay(cDELAY);
  digitalWrite(SHT_LOX1, LOW);
  delay(cDELAY);
  digitalWrite(SHT_LOX2, LOW);
  delay(cDELAY);

  Serial.println("Both in reset mode...(pins are low)");
  Serial.println("Starting...");           
  Serial.println();
  
  setID();

  g_sSteer.attach(5);

  g_sSteer.write(cCENTER);
  delay(500);
  g_sSteer.write(cSWERVELEFT);
  delay(500);
  g_sSteer.write(cCENTER);
  delay(500);
  g_sSteer.write(cSWERVERIGHT);
  delay(500);
  g_sSteer.write(cCENTER);
  delay(cDELAY);

  DCRunFwd();
}

void Swerve()
{
  lcdSensors();

  if (g_iSwerveDir == cLEFT && (g_iLastSwerveDir != g_iSwerveDir || g_iCanSwerve < cMAXSWERVES))
  {  
      g_sSteer.write(cSWERVELEFT);

      delay(200);

      DCCenter();

      g_iCanCorner++;

      if (g_iLastSwerveDir == g_iSwerveDir)
          g_iCanSwerve++;
      else
      {
          g_iCanSwerve = 1;
    
          g_iLastSwerveDir = g_iSwerveDir;
      }

      if (g_bDisplayLCD == true)
        {
          lcd.setCursor(0,0);
          lcd.print("L-SWERVE          ");
          lcd.setCursor(9,0);
          lcd.print(g_iLastSwerveDir);
          lcd.setCursor(12,0);
          lcd.print(g_iSwerveDir);
          lcd.setCursor(15,0);
          lcd.print(g_iCanSwerve);
        }
  }
  else if (g_iSwerveDir == cRIGHT && (g_iLastSwerveDir != g_iSwerveDir || g_iCanSwerve < cMAXSWERVES))
       {  
          g_sSteer.write(cSWERVERIGHT);

          delay(200);

          DCCenter();

          g_iCanCorner++;

          if (g_iLastSwerveDir == g_iSwerveDir)
            g_iCanSwerve++;
          else
          {
            g_iCanSwerve = 1;
        
            g_iLastSwerveDir = g_iSwerveDir;
          }

          if (g_bDisplayLCD == true)
            {
              lcd.setCursor(0,0);
              lcd.print("R-SWERVE          ");
              lcd.setCursor(9,0);
              lcd.print(g_iLastSwerveDir);
              lcd.setCursor(12,0);
              lcd.print(g_iSwerveDir);
              lcd.setCursor(15,0);
              lcd.print(g_iCanSwerve);
            }
        }
        else if (g_bDisplayLCD == true)
              {
                  lcd.setCursor(0,0);  
                  lcd.print("  NOT SWERVE    ");
              }
}

void Corner()
{
  lcdSensors();

  if (g_iCornerDir == cLEFT)
  {
      if (g_bDisplayLCD == true)
      {
          lcd.setCursor(0,0);
          lcd.print("    L-CORNER      ");
      }

      g_sSteer.write(cCORNERLEFT);
  }
  else if (g_iCornerDir == cRIGHT)
      {
          if (g_bDisplayLCD == true)
          {
              lcd.setCursor(0,0);
              lcd.print("    R-CORNER      ");
          }

          g_sSteer.write(cCORNERRIGHT);
      }

  delay(2100);

  DCCenter();

  delay(1500);

  g_iCanCorner = 0; 

  g_iCanSwerve = 0;

  g_iLastSwerveDir = cFRONT;
}

void DCCheckStop()
{
  if (g_iSensors[cFRONT] > 0 && g_iSensors[cFRONT] < cMINSTOPDST)
  {
      DCStop();

      lcd.setCursor(0,0);
      lcd.print("                ");
      lcd.setCursor(0,1);
      lcd.print(" ...STOPPING... ");
    
      lcd.setCursor(5,0);
      lcd.print("[ ");
      lcdPrint(7,0,cFRONT);
      lcd.setCursor(9,0);
      lcd.print(" ]");

      Serial.println(".........STOPING...");
      
      if (g_iLastSwerveDir == cRIGHT)
          g_sSteer.write(cCORNERLEFT);
      else
          g_sSteer.write(cCORNERRIGHT);

      delay(cDELAY);

      DCRunBwd();

      delay(2000);

      DCCenter();
      
      DCStop();

      DCCenter();

      if (g_iLastSwerveDir == cRIGHT)
          g_sSteer.write(cCORNERRIGHT - 3);
      else
          g_sSteer.write(cCORNERLEFT + 3);

      DCRunFwd();

      delay(350);

      DCCenter();
  }
}

void loop()
{
  g_bCorner = false;

  for (g_iSensor = 0; g_iSensor < cNUMSENSORS; g_iSensor++)
  {
      g_iMax[g_iSensor] = 0;
      g_iSensors[g_iSensor] = 0;
      g_iCount8k[g_iSensor] = 0;
      g_dAverages[g_iSensor] = 0.0; 
  
      g_iMin[g_iSensor] = 9999;
  }

  for (g_iFilter = 0; g_iFilter < cNUMSAMPLES; g_iFilter++)
  {
    for (g_iSensor = 0; g_iSensor < cNUMSENSORS; g_iSensor++)
    {
        g_iRangeMilliMeter = 0;

        do
        {
            g_adaLox[g_iSensor].waitRangeComplete();
            g_uStatusLox = g_adaLox[g_iSensor].readRangeStatus();

            g_iRangeMilliMeter++;

            if (g_iSensor == cTOFTOCALIBRATE)
            {
                Serial.print("        Sensor: ");
                Serial.print(g_iSensor);
                Serial.print("  Error status: ");
                Serial.print(g_uStatusLox);
                Serial.print("  Reads: ");
                Serial.println(g_iRangeMilliMeter);
            }
        } while (g_uStatusLox != VL53L0X_ERROR_NONE && g_iRangeMilliMeter < 10);

        g_iRangeMilliMeter = g_adaLox[g_iSensor].readRange();

        if (g_iRangeMilliMeter > c8K)
            g_iCount8k[g_iSensor]++;
        else
        {
          if (g_iRangeMilliMeter < g_iMin[g_iSensor])
              g_iMin[g_iSensor] = g_iRangeMilliMeter;

          if (g_iRangeMilliMeter > g_iMax[g_iSensor])
              g_iMax[g_iSensor] = g_iRangeMilliMeter;

          g_dAverages[g_iSensor] += g_iRangeMilliMeter;
        }

        if (g_iSensor == cTOFTOCALIBRATE)
        {
            Serial.print("[");
            Serial.print(g_iFilter);
            Serial.print(", ");
            Serial.print(g_iSensor);
            Serial.print("] ");
            Serial.print(g_iRangeMilliMeter);
            Serial.print("-> ");
            Serial.print(g_dAverages[g_iSensor]);
            Serial.print("-> ");
            Serial.println(g_iCount8k[g_iSensor]);
        }
     }
  }

  for (g_iSensor = 0; g_iSensor < cNUMSENSORS; g_iSensor++)
  {
      if (g_iCount8k[g_iSensor] > (int)(cNUMSAMPLES * 0.9))
      {
          g_iSensors[g_iSensor] = c8K; 

          if (g_iSensor != cFRONT && g_iCanCorner > cMAXCORNERS && (g_iCornerDir == cFRONT || g_iCornerDir == g_iSensor))
          {
              g_bCorner = true;

              if (g_iCornerDir == cFRONT)
                  g_iCornerDir = g_iSensor;
          }
      }
      else if (g_iCount8k[g_iSensor] == (cNUMSAMPLES - 2))
              g_iSensors[g_iSensor] = (int)(g_dAverages[g_iSensor] / 2.0);
           else
              g_iSensors[g_iSensor] = (int)((g_dAverages[g_iSensor] - g_iMin[g_iSensor] - g_iMax[g_iSensor]) / (cNUMSAMPLES - g_iCount8k[g_iSensor] - 2.0));

      if (g_iSensor == cTOFTOCALIBRATE)
      {
          Serial.println();
          Serial.print("[");
          Serial.print(g_iSensor);
          Serial.print("] ");
          Serial.print(g_dAverages[g_iSensor]);
          Serial.print(" - ");
          Serial.print(g_iMin[g_iSensor]);
          Serial.print(" - ");
          Serial.print(g_iMax[g_iSensor]);
          Serial.print(" / ");
          Serial.println((cNUMSAMPLES - g_iCount8k[g_iSensor] - 2));
      }
  }

  lcdSensors();
  
  /*DCStop();
  delay(1000);
  DCRunFwd();*/

  if (g_iSensors[cRIGHT] > g_iSensors[cLEFT])
      g_iSwerveDir = cRIGHT;
  else
      g_iSwerveDir = cLEFT;

  if (b_bPrintMonitor == true)  
      PrintValues(g_bCorner);

  DCCenter();

  DCCheckStop();

  if (g_iSensors[cRIGHT] != c8K || g_iSensors[cLEFT] != c8K)
      if (g_bCorner == true)
          Corner();
      else
          Swerve();
}