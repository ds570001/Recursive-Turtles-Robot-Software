#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible

//******MOVEMENT******

#define TAPE_SENSOR_R PA4
#define TAPE_SENSOR_L PA3
#define p_pot PA2
#define d_pot PA1

#define MOTOR_L_F PB_9 // will move motor forward when pointed end is away from you (connect pin to top of MCT) (when the potentiometer input > 512)
#define MOTOR_L_R PB_8 // will move the motor backward when pointed end is away from you (connect pin to bottom of MCT) (when the potentiometer input < 512)
#define MOTOR_R_F PB_7
#define MOTOR_R_R PB_6

#define PWMfreq 2000
#define max_input 1023
#define midpoint 512

#define ON_OFF_THRESHOLD 200 //can investigate this number more... above this number is when it is on tape
#define STARTING_DUMMY_VALUE 10000

#define ALUMINUM_THRESHOLD 40
#define REVERSE_CORRECTION 2000

//******PICKING UP CANS******

#define whacker_pin PA7
#define elevator_pin PB1
#define hatch_pin PB0
#define TRIG PB3
#define ECHO PA15
#define COLLISION_SENSOR PA11

#define SONAR_DETECTION_DISTANCE 17


#define HATCH_DOWN 160
#define HATCH_UP 60
#define ELEVATOR_DOWN 10
#define ELEVATOR_DOWN_DRIVING 15
#define ELEVATOR_UP 150
#define WHACKER_OPEN 0
#define WHACKER_CLOSE 180

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//******MOVEMENT******

int previous_error = STARTING_DUMMY_VALUE; //dummy starting value
int dist_between_sensors = 10; //may have to do more precise measurements on this...
//need to have it be far enough away for this error to have meaning but also for there to be big enough range in reflectance values...
double gain_scaling_factor = 2.0; //scales up the gain input from the potentiometers.
int current_state_count = 1; //beginning of counting
int previous_state_count = STARTING_DUMMY_VALUE; //dummy starting value
int previous_state = STARTING_DUMMY_VALUE; //dummy starting value;
double correction_scaling_factor = 2.0; //play around with this value to get g value to fit within duty range.
int max_motor_duty = 65535; //max number the duty can be in motor format
int min_motor_duty = 1; //min number used for linearization?
int nominal_motor_L_duty = 31000;
int nominal_motor_R_duty = 33000;
int num_loops = 0;
double slope_scaling_factor = 1000.0;

//******PICKING UP CANS******
Servo whacker;
Servo elevator;
Servo hatch;

void activate_back_trigger();
int collisions = 0;
int prev_collisions = 0;

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

  //******MOVEMENT******

  //Tape Sensors
  pinMode(TAPE_SENSOR_L, INPUT);
  pinMode(TAPE_SENSOR_R, INPUT);
  pinMode(p_pot,INPUT);
  pinMode(d_pot,INPUT);

  //DC Motors
  pinMode(MOTOR_L_F, OUTPUT);
  pinMode(MOTOR_L_R, OUTPUT); //PA_2 and PA_3 dont work
  pinMode(MOTOR_R_F, OUTPUT);
  pinMode(MOTOR_R_R, OUTPUT);

  //run_motor(nominal_motor_L_duty,MOTOR_L_F,MOTOR_L_R); figure out a better way to start servos up.
  //run_motor(nominal_motor_R_duty,MOTOR_R_F,MOTOR_R_R);

  //******PICKING UP CANS******

  //Servos
  whacker.attach(whacker_pin);
  elevator.attach(elevator_pin);
  hatch.attach(hatch_pin);
  
  whacker.write(WHACKER_OPEN);
  delay(100);
  elevator.write(ELEVATOR_DOWN_DRIVING);
  delay(100);
  hatch.write(HATCH_UP);
  delay(100);

  //Sonar 
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);

  //back hatch trigger
  pinMode(COLLISION_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COLLISION_SENSOR), activate_back_trigger, RISING);

  //delay(10000);
}


//******MOVEMENT******

// runs a motor with the given duty
void run_motor(int duty, PinName motorPin_F, PinName motorPin_B) {
  //duty: if > 0, turn motor forward as described above
  //      if < 0, turn motor backward as described above
  //if the duty scaling is negative then run the motor backwards, if positive then run motor forwards
  if (duty > 0) {
    pwm_start(motorPin_B, PWMfreq, 1, TICK_COMPARE_FORMAT);
    pwm_start(motorPin_F, PWMfreq, duty, TICK_COMPARE_FORMAT);
  } else {
    duty = duty*(-1);
    pwm_start(motorPin_F, PWMfreq, 1, TICK_COMPARE_FORMAT);
    if (duty == 0) {
      duty = 1;
    }
    pwm_start(motorPin_B, PWMfreq, duty, TICK_COMPARE_FORMAT);
  }
}

double prev_distance = 100.0;

//set both motors to 0
void stop_motors(){
  pwm_start(MOTOR_L_F, PWMfreq, 1, TICK_COMPARE_FORMAT);
  pwm_start(MOTOR_L_R, PWMfreq, 1, TICK_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, PWMfreq, 1, TICK_COMPARE_FORMAT);
  pwm_start(MOTOR_R_R, PWMfreq, 1, TICK_COMPARE_FORMAT);
}

bool tape_follow() {
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

  //display.clearDisplay();

  int position_L_analog = analogRead(TAPE_SENSOR_L);
  int position_R_analog = analogRead(TAPE_SENSOR_R);
  int position_L;
  int position_R;

  if (position_L_analog < ALUMINUM_THRESHOLD && position_R_analog < ALUMINUM_THRESHOLD) {
    stop_motors();
    delay(500);
    run_motor((-1)*nominal_motor_L_duty-REVERSE_CORRECTION,MOTOR_L_F,MOTOR_L_R);
    run_motor((-1)*nominal_motor_R_duty,MOTOR_R_F,MOTOR_R_R);
    return true;
  }

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
    error = -dist_between_sensors; // pretty sure this case is impossible, but just in case, I'm going with the 50% chance that the robot will be on the right side of the tape...
  }

  int p_gain = analogRead(p_pot); //purely for testing - once we have the final values we will remove the potentiometers
  int d_gain = analogRead(d_pot);
  //int i_gain - probably shouldnt impement this...

  p_gain = p_gain * gain_scaling_factor;
  d_gain = d_gain * gain_scaling_factor;

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

  if (num_loops % 1000 == 0) { //shouldnt print to screen every loop -> slows down tape following execution
    
    display.clearDisplay();
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
    // display.setCursor(50,40);
    // display.print((int)g);

  //   /*
  //   display.setCursor(60,20);
  //   display.print("g");
  //   display.setCursor(75,20);
  //   display.print(g);
  //   */
  //   /*
  //   display.setCursor(0,0);
  //   display.print(previous_error);
  //   display.setCursor(0,20);
  //   display.print(previous_state_count);
  //   display.setCursor(0,40);
  //   display.print(current_state_count);
  //   display.setCursor(25,0);
  //   display.print(slope);
  //   display.setCursor(75,20);
  //   display.print(error);
  //   display.setCursor(75,40);
  //   display.print(previous_state);
  //   */

    display.display();
  }

  if (error == dist_between_sensors || error == dist_between_sensors*(-1)) {
    if (g > 0) {
      run_motor(nominal_motor_R_duty + g,MOTOR_R_F,MOTOR_R_R);
      run_motor(nominal_motor_L_duty - g*1.5,MOTOR_L_F,MOTOR_L_R);
    } else if (g < 0) {
      run_motor(nominal_motor_L_duty + g*(-1),MOTOR_L_F,MOTOR_L_R);
      run_motor(nominal_motor_R_duty - g*(-1)*3/4,MOTOR_R_F,MOTOR_R_R);
    } else {
      run_motor(nominal_motor_R_duty,MOTOR_R_F,MOTOR_R_R);
      run_motor(nominal_motor_L_duty,MOTOR_L_F,MOTOR_L_R);
    }
  } else {
    if (g > 0) {
      run_motor(nominal_motor_R_duty + g,MOTOR_R_F,MOTOR_R_R);
      run_motor(nominal_motor_L_duty - g/2,MOTOR_L_F,MOTOR_L_R);
    } else if (g < 0) {
      run_motor(nominal_motor_L_duty + g*(-1),MOTOR_L_F,MOTOR_L_R);
      run_motor(nominal_motor_R_duty - g*(-1)/2,MOTOR_R_F,MOTOR_R_R);
    } else {
      run_motor(nominal_motor_R_duty,MOTOR_R_F,MOTOR_R_R);
      run_motor(nominal_motor_L_duty,MOTOR_L_F,MOTOR_L_R);
    }
  }

  //have to incorporate linearization through experimentation
  num_loops++;
  return false;
}

//******PICKING UP CANS******

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

// set all servos to zero position
void zero_servos(){
  whacker.write(WHACKER_OPEN);
  elevator.write(ELEVATOR_DOWN);
  hatch.write(HATCH_UP);
  return;
}

//get distance with the sonar
double get_distance(){
  // read distance with sonar
  digitalWrite(TRIG, LOW);
  delay(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG, HIGH);
  delay(10);
  digitalWrite(TRIG, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  double duration = pulseIn(ECHO, HIGH);
  // Calculating the distance
  double distance = duration*0.034/2;
  return distance;
}

// opens whacker, closes whacker, raises elevator, lowers elevator to collect can
void perform_can_collection(){
  whacker.write(WHACKER_OPEN);
  elevator.write(ELEVATOR_DOWN);
  hatch.write(HATCH_UP);
  //if distance detected by sonar is less than SONAR_DETECTION_DISTANCE, close whacker and raise elevator
  run_servo(whacker, WHACKER_OPEN, WHACKER_CLOSE, 10);
  delay(100);
  run_servo(whacker, WHACKER_CLOSE, WHACKER_OPEN, 10);
  delay(100);
  run_servo(elevator, ELEVATOR_DOWN, ELEVATOR_UP, 15);
  delay(1000);
  run_servo(elevator, ELEVATOR_UP, ELEVATOR_DOWN_DRIVING, 15);
  delay(100);
  return;
}

//increments collisions
void activate_back_trigger() {
  collisions++;
  stop_motors();
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(collisions);
  display.display();
}

bool check_collision() {
if (collisions > prev_collisions) {
  stop_motors();
  run_servo(hatch, HATCH_UP, HATCH_DOWN, 10);
  delay(500);
  run_servo(hatch, HATCH_DOWN, HATCH_UP, 10);
  prev_collisions = collisions;
  return true;
  } else {
    return false;
  }
}

void test_screen() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("f");
  display.display();
}

void test_servo_sonar() {
  int distance = get_distance();
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(distance);
  display.display();
  // take the average of the previous sonar reading and current sonar reading to prevent noise errors
  if ((distance + prev_distance)/2 < SONAR_DETECTION_DISTANCE){
    //if distance detected by sonar is less than SONAR_DETECTION_DISTANCE, close whacker and raise elevator
    perform_can_collection();
  }
  prev_distance = distance;

}
int ran = 0;
void random_function() {
  ran = 2*3;
  ran = 4*7;
  ran = 5*9;
}
int count = 0;
bool is_reversing = false;
bool had_collision = false;
void loop() {

  if (is_reversing) {
    had_collision = check_collision();
    if (had_collision) {
      had_collision = false;
      is_reversing = false;
    } else {
      run_motor((-1)*nominal_motor_L_duty-REVERSE_CORRECTION,MOTOR_L_F,MOTOR_L_R);
      run_motor((-1)*nominal_motor_R_duty,MOTOR_R_F,MOTOR_R_R);
      delay(5000);
    }
  } else {

  

    if (count % 50 == 0) {
      int distance = get_distance();
      if (distance < SONAR_DETECTION_DISTANCE) {
        stop_motors();
        delay(100);
        perform_can_collection();
        delay(100);
      }
    }

    //  if (count % 4000 == 0) {
    // //   stop_motors();
    // //   delay(100);
    // //   perform_can_collection();
    //   // for (int start = WHACKER_OPEN; start < WHACKER_CLOSE; start += 25) {
    //   //   whacker.write(start);
    //   //   delay(10);
    //   //   tape_follow();
    //   // }
    //   // for (int start = WHACKER_CLOSE; start > WHACKER_OPEN; start -= 25) {
    //   //   whacker.write(start);
    //   //   delay(10);
    //   //   tape_follow();
    //   // }

    //   whacker.write(WHACKER_CLOSE);
    //   delay(100);
    //   tape_follow();
    //   delay(100);
    //   whacker.write(WHACKER_OPEN);
    //   delay(100);

    // //   delay(100);
    //  }

    is_reversing = tape_follow(); // returns true if reversing.
    count++;

    // int distance = get_distance();
    // display.clearDisplay();
    // display.setCursor(0,0);
    // display.print(distance);
    // if (distance < SONAR_DETECTION_DISTANCE) {
    //   display.setCursor(0,20);
    //   display.print("triggered!");
    // } else {
    //   display.setCursor(0,20);
    //   display.print(" ");
    // }
    // display.display();

    }

}
