#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_SSD1306.h>

//***************************************************
/*
NOTE: LEFT AND RIGHT IS ORIENTED AS IF YOU WERE RIDING THE ROBOT (LIKE DRIVING A CAR)
*/
//***************************************************

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible

#define TAPE_SENSOR_R PA3
#define TAPE_SENSOR_L PA2
#define p_pot PA5
#define d_pot PA4

#define MOTOR_L_F PA_6 // will move motor forward when pointed end is away from you (connect pin to top of MCT) (when the potentiometer input > 512)
#define MOTOR_L_R PA_7 // will move the motor backward when pointed end is away from you (connect pin to bottom of MCT) (when the potentiometer input < 512)
#define MOTOR_R_F PB_0
#define MOTOR_R_R PB_1

#define PWMfreq 2000
#define max_input 1023
#define midpoint 512

#define ON_OFF_THRESHOLD 200 //can investigate this number more... above this number is when it is on tape
#define STARTING_DUMMY_VALUE 10000

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int previous_error = STARTING_DUMMY_VALUE; //dummy starting value
int dist_between_sensors = 4; //may have to do more precise measurements on this...
//need to have it be far enough away for this error to have meaning but also for there to be big enough range in reflectance values...
double gain_scaling_factor = 1.0; //scales down the gain input from the potentiometers.
int current_state_count = 1; //beginning of counting
int previous_state_count = STARTING_DUMMY_VALUE; //dummy starting value
int previous_state = STARTING_DUMMY_VALUE; //dummy starting value;
double correction_scaling_factor = 2.0; //play around with this value to get g value to fit within duty range.
int max_motor_duty = 65535; //max number the duty can be in motor format
int min_motor_duty = 1; //min number used for linearization?
int nominal_motor_L_duty = 35000;
int nominal_motor_R_duty = 35000;
int num_loops = 0;
double slope_scaling_factor = 50.0;

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 
  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display.display();
  delay(2000);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Hello world!");
  display.display();

  pinMode(TAPE_SENSOR_L, INPUT);
  pinMode(TAPE_SENSOR_R, INPUT);
  pinMode(p_pot,INPUT);
  pinMode(d_pot,INPUT);

  pinMode(MOTOR_L_F, OUTPUT);
  pinMode(MOTOR_L_R, OUTPUT); //PA_2 and PA_3 dont work
  pinMode(MOTOR_R_F, OUTPUT);
  pinMode(MOTOR_R_R, OUTPUT);
}

void run_motor(int duty, PinName motorPin_F, PinName motorPin_B) {
  //duty: if > 0, turn motor forward as described above
  //      if < 0, turn motor backward as described above
  //if the duty scaling is negative then run the motor backwards, if positive then run motor forwards
  if (duty > 0) {
    pwm_start(motorPin_B,PWMfreq,1,TICK_COMPARE_FORMAT);
    pwm_start(motorPin_F,PWMfreq,duty,TICK_COMPARE_FORMAT);
  } else {
    duty = duty*(-1);
    pwm_start(motorPin_F,PWMfreq,1,TICK_COMPARE_FORMAT);
    if (duty == 0) {
      duty = 1;
    }
    pwm_start(motorPin_B,PWMfreq,duty,TICK_COMPARE_FORMAT);
  }
};

void loop() {

  /*
  convert analog values to digital values (if above threshold or below threshold)
  find error

  *digital*
  if error is greater than the threshold then correct.
  proportional: error*p_gain
  derivative: d_gain*slope
  probably wont need integral but would do i + i_gain*error
  g = p+d+i
  send g (with scaling) to run motor function.
  */

  display.clearDisplay();

  int position_L_analog = analogRead(TAPE_SENSOR_L);
  int position_R_analog = analogRead(TAPE_SENSOR_R);
  int position_L;
  int position_R;

  int error;


  //convert analog tape sensor reading to digital position
  if (position_L_analog > ON_OFF_THRESHOLD) {
    position_L = 1; //on tape
  } else {
    position_L = 0; //off tape
  }

  if (position_R_analog > ON_OFF_THRESHOLD) {
    position_R = 1; //on tape
  } else {
    position_R = 0; //off tape
  }

  if (position_L == 1 and position_R == 1) {
    error = 0; //no error since both are on tape
  } else if (position_L == 0 and position_R == 1) {
    error = -1; //left sensor is off tape and right is on tape... think of negative when robot is moving left
  } else if (position_L == 1 and position_R == 0) {
    error = 1; //left sensor is on tape and right is off tape... think of positive when robot is moving right
  } else if (previous_error == -1 || previous_error == -dist_between_sensors) { //at this point both sensors must be off the tape. Must remember where it was the previous loop...
    error = -dist_between_sensors; //need an error proportional to the distance between the sensors so you get linear PID control
    //negative means that the left tape sensor left the tape first, therefore the robot must turn right to get back on track
  } else if (previous_error == 1 || previous_error == dist_between_sensors) {
    error = dist_between_sensors;
    //positive means that the right tape sensor left the tape first, therefore the robot must turn left to get back on track
  } else if (previous_error == 0) {
    error = dist_between_sensors; // pretty sure this case is impossible, but just in case, I'm going with the 50% chance that the robot will be on the right side of the tape...
  }

  int p_gain = analogRead(p_pot); //purely for testing - once we have the final values we will remove the potentiometers
  int d_gain = analogRead(d_pot);
  //int i_gain - probably shouldnt impement this...

  p_gain = p_gain / gain_scaling_factor;
  d_gain = d_gain / gain_scaling_factor;

  //proportional control
  double p = error*p_gain;

  //derivative control - wrote when i was very tired so I may have made mistakes
  double slope = 0;
  double d = 0; //for very first state the derivative control should be 0

  //should this go before or after the d calculation block? if before, then am taking the difference in time including the 
  if (previous_error == error) {
    current_state_count++;
  } else if (previous_error != STARTING_DUMMY_VALUE) {
    previous_state_count = current_state_count; // if a change in state has occurred, the current number of instances now becomes the number of instances in the previous state.
    previous_state = previous_error; //store the previous state when change occurs
    current_state_count = 1; //start at 1 because 1 instance has occurred.
  }

  if (previous_state_count != STARTING_DUMMY_VALUE) {
     slope = (double)(error - previous_state) / (previous_state_count + current_state_count - 1) * slope_scaling_factor; //-1 because i want it to be the true difference in time (time final - time initial), not with an extra increment (look at the graph at the end of lecture 5 to understand what I mean)
     d = slope*d_gain;
  }
  previous_error = error;
  //potential bug: derivative does not go to 0 as soon as tape sensors are aligned on tape. Have to test and see if this is an issue or not.

  int g = (p+d) * correction_scaling_factor;

  if (num_loops % 10 == 0) { //shouldnt print to screen every loop -> slows down tape following execution
    
    display.setCursor(0,0);
    display.print("pg");
    display.setCursor(25,0);
    display.print(p_gain);
    display.setCursor(63,0);
    display.print("dg");
    display.setCursor(88,0);
    display.print(d_gain);
    display.setCursor(0,20);
    display.print("p");
    display.setCursor(13,20);
    display.print((int)p);
    display.setCursor(0,40);
    display.print("d");
    display.setCursor(13,40);
    display.print((int)d);
    display.setCursor(50,40);
    display.print((int)g);

    /*
    display.setCursor(60,20);
    display.print("g");
    display.setCursor(75,20);
    display.print(g);
    */
    /*
    display.setCursor(0,0);
    display.print(previous_error);
    display.setCursor(0,20);
    display.print(previous_state_count);
    display.setCursor(0,40);
    display.print(current_state_count);
    display.setCursor(25,0);
    display.print(slope);
    display.setCursor(75,20);
    display.print(error);
    display.setCursor(75,40);
    display.print(previous_state);
    */

    display.display();
  }

  if (error > 0) {
    run_motor(nominal_motor_R_duty + g,MOTOR_R_F,MOTOR_R_R);
  } else if (error < 0) {
    run_motor(nominal_motor_L_duty + g*(-1),MOTOR_L_F,MOTOR_L_R);
  } else {
    run_motor(nominal_motor_R_duty,MOTOR_R_F,MOTOR_R_R);
    run_motor(nominal_motor_L_duty,MOTOR_L_F,MOTOR_L_R);
  }

  //have to incorporate linearization through experimentation
  num_loops++;
};