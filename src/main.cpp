#include <Arduino.h>
#include <Servo.h>

class ServoMotor
{
private:
  int pin;
  float angle;
  float min_angle = 0;
  float max_angle = 180;
  float correction_angle;
  float offset_angle = 0;
  int min_pulse;
  int max_pulse;
  int pulse;
  bool is_reversed = false;

public:
  float home_angle;
  ServoMotor(int pin, int min_pulse, int max_pulse, float min_angle, float max_angle, float home_angle, float correction_angle, float offset_angle, bool is_reversed);
  bool setAngle(float angle, Servo servo);
  float getAngle();
};

ServoMotor::ServoMotor(int pin, int min_pulse, int max_pulse, float min_angle, float max_angle, float home_angle, float correction_angle, float offset_angle, bool is_reversed)
{
  this->pin = pin;
  this->min_pulse = min_pulse;
  this->max_pulse = max_pulse;
  this->home_angle = home_angle;
  this->correction_angle = correction_angle;
  this->offset_angle = offset_angle;
  this->is_reversed = is_reversed;
  this->min_angle = min_angle;
  this->max_angle = max_angle;
  this->angle = home_angle;
}

float ServoMotor::getAngle()
{
  return this->angle;
}

/**
 * @brief   Set the Angle object
 *
 * @param angle   The angle to set
 * @return true   If the angle follows all restrictions
 * @return false  If the angle does not follow all restrictions
 */
bool ServoMotor::setAngle(float angle, Servo servo)
{

  angle -= this->offset_angle;
  angle += this->correction_angle;
  if (angle < this->min_angle || angle > this->max_angle)
  {
    return false;
  }
  this->pulse = map(angle * 100, 0, 180 * 100, this->min_pulse, this->max_pulse);
  if (this->is_reversed)
  {
    angle = 180 - angle;
    this->pulse = map(angle * 100, 0, 180 * 100, this->min_pulse, this->max_pulse);
  }
  // this->servo.writeMicroseconds(this->pulse);
  servo.writeMicroseconds(this->pulse);
  delay(5); // Wait for the servo to reach the position (15ms is a good value
  this->angle = angle;
  return true;
}

Servo s1;
Servo s2;
Servo s3;

// ServoMotor::ServoMotor(int pin, int min_pulse, int max_pulse, float min_angle, float max_angle, float home_angle, float correction_angle, float offset_angle, bool is_reversed)

ServoMotor servo1(9, 750, 2310, 0, 180, 90, 13.4, 0, false);
ServoMotor servo2(10, 750, 2310, 0, 180, 220, -2.5, 90, false);
ServoMotor servo3(11, 750, 2340, 15, 180, 45, -8.4, 0, false);

void setup()
{
  Serial.begin(115200);
  s1.attach(9);
  s2.attach(10);
  s3.attach(11);
  servo1.setAngle(servo1.home_angle, s1);
  servo2.setAngle(servo2.home_angle, s2);
  servo3.setAngle(servo3.home_angle, s3);
  delay(3000);
}
float angle1 = 90;
float angle2 = 90;
float angle3 = 90;

void loop()
{
  while (Serial.available() > 0)
  {
    // Read string from serial bus
    String str = Serial.readStringUntil('\n');

    // Extract the angle values from the string
    angle1 = str.substring(0, str.indexOf(':')).toFloat();
    str.remove(0, str.indexOf(':') + 1); // remove the first part of the string (the part before the ':'  character)
    angle2 = str.substring(0, str.indexOf(':')).toFloat();
    str.remove(0, str.indexOf(':') + 1); // remove the first part of the string (the part before the ':'  character)
    angle3 = str.toFloat();
  }

  servo1.setAngle(angle1, s1);
  servo2.setAngle(angle2, s2);
  servo3.setAngle(angle3, s3);
}