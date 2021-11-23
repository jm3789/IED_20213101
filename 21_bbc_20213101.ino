#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 0
#define _DIST_MAX 450

// Servo range
// configurable parameters
#define _DUTY_MIN 1550 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1850 // servo neutral position (90 degree)
#define _DUTY_MAX 2150 // servo full counterclockwise position (180 degree)

#define _POS_START (_DUTY_MIN)
#define _POS_END (_DUTY_MAX)

#define _SERVO_SPEED 50 // servo speed limit (unit: degree/second)
#define _SERVO_INTERVAL 100  // servo update interval
#define _DIST_INTERVAL 20
#define _INTERVAL_SERIAL 100

// global variables
float ema_dist;
float alpha = 0.1; // unit: mm
unsigned long last_sampling_time; // unit: ms
int duty_chg_per_interval; // maximum duty difference per interval
int toggle_interval, toggle_interval_cnt;
Servo myservo;
int duty_target, duty_curr;

int a, b; // unit: mm

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO); 
  duty_target = duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);
  // initialize serial port
  Serial.begin(57600);

  a = 83; //70;
  b = 330; //300;
  
  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _SERVO_INTERVAL / 1000;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  // wait until next sampling time. 
  // millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  //if(millis() < last_sampling_time + INTERVAL) return;
  float raw_dist = ir_distance();
  //float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);

  alpha = 0.1;
  ema_dist = (1-alpha) * ema_dist + alpha * raw_dist;

// output the read value to the serial port
  Serial.print("min:100,max:450,dist:");
  Serial.print(raw_dist);
  Serial.print(",ema_dist:");
  Serial.println(ema_dist);
  if(ema_dist > 90) {
    duty_curr += duty_chg_per_interval;
  }
  if (ema_dist < 90) {
    duty_curr -= duty_chg_per_interval;
  }
  delay(20);

  if(duty_curr > _DUTY_MAX){duty_curr = _DUTY_MAX;}
  if(duty_curr < _DUTY_MIN){duty_curr = _DUTY_MIN;}
  // update servo position
  myservo.writeMicroseconds(duty_curr);
}
