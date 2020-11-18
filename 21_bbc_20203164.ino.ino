#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0

// configurable parameters
#define _DUTY_MIN 500 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1000 // servo neutral position (90 degree)
#define _DUTY_MAX 1500 // servo full counterclockwise position (180 degree)

#define _POS_START (_DUTY_MIN + 100)
#define _POS_END (_DUTY_MAX - 100)

#define _SERVO_SPEED 60 // servo speed limit (unit: degree/second)
#define INTERVAL 20  // servo update interval
int a;
int b;
// global variables
unsigned long last_sampling_time; // unit: ms
int duty_chg_per_interval; // maximum duty difference per interval
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
Servo myservo;
int duty_target, duty_curr;

void setup() {
// a,b setup
  a = 70;
  b = 300;
// initialize GPIO pins
  myservo.attach(PIN_SERVO); 
  duty_target = duty_curr = _POS_START;
  myservo.writeMicroseconds(duty_curr);
  
// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * INTERVAL / 1000;

// remove next three lines after finding answers
//  Serial.print("duty_chg_per_interval:");
//  Serial.println(duty_chg_per_interval);
//  while(1) {}

// initialize variables for servo update.
  pause_time = 1;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
  toggle_interval_cnt = toggle_interval;
  
// initialize last sampling time
  last_sampling_time = 0;
} 
float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 150 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(dist_cali < 255.0) myservo.writeMicroseconds(_DUTY_MAX);
  else myservo.writeMicroseconds(_DUTY_MIN);
  delay(20);
  
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.

// adjust duty_curr toward duty_target by duty_chg_per_interval
  
// update servo position

// update last sampling time
  last_sampling_time += INTERVAL;
}
