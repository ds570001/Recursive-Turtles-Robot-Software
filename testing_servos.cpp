#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_SSD1306.h>

#define PWMFREQ 2000
#define LEFT_F_MOTOR PB_6
#define LEFT_B_MOTOR PB_7
#define RIGHT_F_MOTOR PB_9
#define RIGHT_B_MOTOR PB_8

#define ELEVATOR_SERVO PB0
#define WHACKER_SERVO PB1
#define BACK_SERVO PA7

#define COLLISION_SENSOR PA6

#define ECHO PA11
#define TRIG PA12

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Servo whacker_servo;
Servo elevator_servo;
Servo back_servo;

// TwoWire Wire(SCL, SDA, SOFT_STANDARD);

// interrupt 
void handle_interrupt();
void handle_interrupt_schmitt();
volatile int i =0;
volatile bool release_back_hatch = false;
volatile int loopcount =0;

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(2000);

  //DC Motors
  pinMode(LEFT_F_MOTOR, OUTPUT);
  pinMode(LEFT_B_MOTOR, OUTPUT);
  pinMode(RIGHT_F_MOTOR, OUTPUT);
  pinMode(RIGHT_B_MOTOR, OUTPUT);

  //servos
  whacker_servo.attach(WHACKER_SERVO);
  elevator_servo.attach(ELEVATOR_SERVO);
  back_servo.attach(BACK_SERVO);

  //LM311
  pinMode(COLLISION_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COLLISION_SENSOR), handle_interrupt, RISING);

  //Sonar 
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  

}

void handle_interrupt() { 
  i++;
  }

// runs a certain servo from angle start to angle end, increment angle by int speed each loop
void run_servo(Servo this_servo, int start, int end, int speed){
  if (start < end){
    for (int angle = start; angle <= end; angle+= speed){
      this_servo.write(angle);
      delay(100);
    }
  }
  else{
    for (int angle = start; angle >= end; angle-= speed){
      this_servo.write(angle);
      delay(100);
    }
  }
}

void loop() {

  // pwm_start(LEFT_F_MOTOR, PWMFREQ, 512, RESOLUTION_9B_COMPARE_FORMAT);
  // pwm_start(LEFT_B_MOTOR, PWMFREQ, 0, RESOLUTION_9B_COMPARE_FORMAT);
  // pwm_start(RIGHT_F_MOTOR, PWMFREQ, 512, RESOLUTION_9B_COMPARE_FORMAT);
  // pwm_start(RIGHT_B_MOTOR, PWMFREQ, 0, RESOLUTION_9B_COMPARE_FORMAT);
  // delay(2000);

  // pwm_start(LEFT_F_MOTOR, PWMFREQ, 0, RESOLUTION_9B_COMPARE_FORMAT);
  // pwm_start(LEFT_B_MOTOR, PWMFREQ, 512, RESOLUTION_9B_COMPARE_FORMAT);
  // pwm_start(RIGHT_F_MOTOR, PWMFREQ, 0, RESOLUTION_9B_COMPARE_FORMAT);
  // pwm_start(RIGHT_B_MOTOR, PWMFREQ, 512, RESOLUTION_9B_COMPARE_FORMAT);
  // delay(2000);

  // pwm_start(LEFT_F_MOTOR, PWMFREQ, 0, RESOLUTION_9B_COMPARE_FORMAT);
  // pwm_start(LEFT_B_MOTOR, PWMFREQ, 0, RESOLUTION_9B_COMPARE_FORMAT);
  // pwm_start(RIGHT_F_MOTOR, PWMFREQ, 0, RESOLUTION_9B_COMPARE_FORMAT);
  // pwm_start(RIGHT_B_MOTOR, PWMFREQ, 0, RESOLUTION_9B_COMPARE_FORMAT);


  // digitalWrite(TRIG, LOW);
  // delay(2);
  // // Sets the trigPin on HIGH state for 10 micro seconds
  // digitalWrite(TRIG, HIGH);
  // delay(10);
  // digitalWrite(TRIG, LOW);
  // // Reads the echoPin, returns the sound wave travel time in microseconds
  // double duration = pulseIn(ECHO, HIGH);
  // // Calculating the distance
  // double distance= duration*0.034/2;

  // display.clearDisplay();
  // display.setTextSize(1);
  // display.setTextColor(SSD1306_WHITE);
  // display.setCursor(0,0);
  // display.print("distance: ");
  // display.println(distance);
  // display.display();

  // display.print("Loop counter: ");
  // display.println(loopcount);
  // display.print("Collisions: ");
  // display.println(i);
  // display.display();
  // loopcount++;

  whacker_servo.write(0);
  elevator_servo.write(0);
  back_servo.write(0);

  run_servo(whacker_servo, 0, 180, 10);
  run_servo(whacker_servo, 180, 0, 10);
  delay(2000);

  run_servo(elevator_servo, 0, 150, 10);
  delay(2000);
  run_servo(elevator_servo, 150, 0, 10);

  delay(5000);
  back_servo.write(180);
  delay(2000);

}

