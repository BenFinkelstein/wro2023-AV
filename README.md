The autonomous vehicle was fully designed from the ground up by the team RoboBros formed by Benjamin Finkelstein Fell and Dimitri Domzalski. 95% of the structures (chassis, steering linkage, sensors board, joints, motor platform, etc.) were printed on a 3D printer. The vehicle is controlled by an Arduino Nano Every. The robot also uses three VL53L0X TOF Sensors, one QMC5883L Magnetometer, one TCS230/TCS3200 Color Sensor, one 13Kg Digital Servo and one Lego Mindstorm DC Large Motor connected to an L298N H-Bridge ESC (Electronic Speed Controller). As a power source, the vehicle uses an 11.1v Li-Po battery to supply power to the Arduino, the servo and the DC motor. The three TOF sensors, the magnetometer and the color sensor get their power supply directly from the Arduino. The code has been developed, compiled and transferred into the Arduino using the Arduino IDE 2.1.1.

Arduino and its IDE were chosen because of their proven reliability and their wide range of features. The readily available libraries, already included in the IDE or easily downloaded from the Internet and installed, was another reason for the selection.

These libraries are needed to interact correctly and successfully with all of the electronic components. Once installed they give access to the functions and variables that allow the code to use the components.

The functions in these libraries are defined in the following header files:

#include "Adafruit_VL53L0X.h"

#include <Servo.h>

#include "Wire.h"

#include <QMC5883L.h>

VL53L0X TOF Sensor uses a combination of an invisible micro laser and a sensor to detect the time the laser takes to hit and bounce off an object in its field of view (Time Of Flight). This particular sensor was chosen because its price/performance in addition to its range, makes sense for a project like this one. The one drawback of sensors at this price range is the erratic behavior, where very often they return bad raw data in the form of inaccurate distance reading. The team partially solved this problem by normalizing groups of readings by eliminating anomalies and averaging the rest of readings.

The team first tested a gyro but it proved to unreliable to guide the vehicle. The team then switch to a magnetometer, which when its raw data is correctly processed helps the vehicle maintain the desire course far better than the gyro.

The TCS230/TCS3200 Color Sensor was selected as the tool to identify the color of the obstacle in order to decide with side to drive by. With the help of an 8 x 8 array of photodiodes it senses color light. After first calibrating the sensor for the specific light conditions of the room, the results are quite reliable.

The 13Kg servo was considered tough enough to work as the single steering actuator, as in the case of every other component it is controlled through the functions of its own library.

The team finally settled, after testing several different options, on the Lego Mindstorm Large DC motor because because it is well known to the members as they use it in last year's WRO competition and also because it proved to be a simple and reliable solution that interacted easily with the ESC and the Arduino. The ESC is needed to act as a bridge between the Arduino and motor as the Arduino cannot directly supply the required power.

The code, firstly, defines the connections to the Arduino pins of every electronic component in the user defined function setID() and the IDE function setup() by using the functions pinMode() and digitalWrite(), functions predefine in the IDE. For example:

pinMode(in1, OUTPUT);

pinMode(in2, OUTPUT);

pinMode(SHT_LOX0, OUTPUT);

pinMode(SHT_LOX1, OUTPUT);

pinMode(SHT_LOX2, OUTPUT);

digitalWrite(SHT_LOX0, LOW);

digitalWrite(SHT_LOX1, LOW);

digitalWrite(SHT_LOX2, LOW);

It then continues to initialize the aforementioned components with the previously mentioned functions from the library of the components in questions such as compass.init() and servo.attach().

The program reads raw data from the three TOF Sensors (Time of Flight distance sensors), one for the left side,one for the right side and one in front, using the library function ragingTest() and the object variable RangeMilliMeter().

The code interects with Magnetometer with library function read (&x, &y, &z). The resulting values are further processes with the following formula:

atan2(g_iY, g_iX) * 180) / PI) + LocalMagneticDeviationpinMode(enA, OUTPUT);

The heading calculated by this function (correcting a negative value by adding 360) is used to steer the vehicle using the Digital Servo’s library function write() which takes at is only parameter the desired direction in degress.

The DC motor is controlled by the following user defined functions:

void DCRun()

{

digitalWrite(in1, HIGH);

digitalWrite(in2, LOW);

analogWrite(enA, cSPEED);

}

The user defined function DCRun() uses the IDE function digitalWrite() which sets the Arduino Nano Every pin number 6 to HIGH and the pin 4 to LOW to specify the driving direction forward and sets the driving speed, with another IDE function, analogWrite(), and the pin 3 set to the desired speed.

void DCStop()

{

digitalWrite(in1, LOW);

digitalWrite(in2, LOW);

}

The user defined function DCStop() uses the same IDE function digitalWrite() with the Arduino Nano Every pin6 set to LOW and the pin 4 set to LOW to stop the motor from rotating.

The core of the code is written in the IDE function loop() which, as the name implies, runs over and over again the written code inside of this essencial function. This allows for continuous interactions in real-time of all of the components and results in the vehicle being able to drive by constantly being informed of its surroundings: the distances in relation to the outer and inner walls, the distance, direction, and when close enough, the color of the obstacles. This data-stream allows the vehicle to advance in a swerve-like fashion, trying constantly to place itself in the best possible place to accomplish the task at hand, and drive as fast as possible around the circuit and/or evade obstacles.

Preforming a 90 degree turn around the inner corners is accomplished by the user defined function Corner(), in which the code looks for either a wall fast approaching the front sensor, meaning the vehicle is close to the end of that side of the outer wall, or a dramatic increase in the distance to the inner wall read by the left or right, meaning the vehicle has passed the inner wall and has to corner in the direction of the change in distance.

As previously mentioned, the pre-compiling checks, the compiling, the linking with the appropriate libraries and finally, the uploading to the vehicle through a USB to micro-USB cable is done by the Arduino IDE.
