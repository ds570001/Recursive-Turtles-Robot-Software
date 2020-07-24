#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_SSD1306.h>

#define PWMfreq 2000
#define LEFT_F_MOTOR PB_9
#define LEFT_B_MOTOR PB_8

#define RIGHT_F_MOTOR PB_6
#define RIGHT_B_MOTOR PB_7

#define TAPE_SENSOR_R PA3
#define TAPE_SENSOR_L PA4
#define p_pot PA1
#define d_pot PA2
#define max_input 1023
#define midpoint 512
#define ON_OFF_THRESHOLD 200 //can investigate this number more... above this number is when it is on tape
#define BAR_THRESHOLD 800
#define STARTING_DUMMY_VALUE 10000

int previous_error = STARTING_DUMMY_VALUE; //dummy starting value
int dist_between_sensors = 8; //may have to do more precise measurements on this...
//need to have it be far enough away for this error to have meaning but also for there to be big enough range in reflectance values...
double gain_scaling_factor = 1.0; //scales down the gain input from the potentiometers.
int current_state_count = 1; //beginning of counting
int previous_state_count = STARTING_DUMMY_VALUE; //dummy starting value
int previous_state = STARTING_DUMMY_VALUE; //dummy starting value;
double correction_scaling_factor = 5; //play around with this value to get g value to fit within duty range.
int max_motor_duty = 65535; //max number the duty can be in motor format
int min_motor_duty = 1; //min number used for linearization?
int nominal_motor_L_duty = 34500;
int nominal_motor_R_duty = 34500;
int num_loops = 0;
int max_states_before_clear = 5;
double slope_scaling_factor = 100.0;


#define ELEVATOR_SERVO PB0
#define WHACKER_SERVO PB1
#define BACK_SERVO PA7
#define HATCH_DOWN 180
#define HATCH_UP 0
#define ELEVATOR_DOWN 0
#define ELEVATOR_UP 150
#define WHACKER_OPEN 0
#define WHACKER_CLOSE 180


#define COLLISION_SENSOR PA6

#define ECHO PA11
#define TRIG PA12
#define SONAR_DETECTION_DISTANCE -100

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Servo whacker_servo;
Servo elevator_servo;
Servo back_servo;

// TwoWire Wire(SCL, SDA, SOFT_STANDARD);

// interrupt 
void activate_back_trigger();
volatile int collisions = 0;

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
  attachInterrupt(digitalPinToInterrupt(COLLISION_SENSOR), activate_back_trigger, RISING);

  //Sonar 
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  
  //tape sensor pid
  pinMode(TAPE_SENSOR_L, INPUT);
  pinMode(TAPE_SENSOR_R, INPUT);
  pinMode(p_pot,INPUT);
  pinMode(d_pot,INPUT);

  run_motor(nominal_motor_L_duty, LEFT_F_MOTOR, LEFT_B_MOTOR);
  run_motor(nominal_motor_R_duty, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
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

void activate_back_trigger(){
  collisions ++;
}

int prev_collisions = 0;
double prev_distance = 100.0;
double distance = 100;
double prev_d = 0;
int prev_position_L = 0, prev_position_R=0;

void loop() {
  // set all servos at initial positions
  whacker_servo.write(WHACKER_OPEN);
  elevator_servo.write(ELEVATOR_DOWN+20);
  back_servo.write(HATCH_UP);

  if (num_loops % 10 == 0){
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
    distance= duration*0.034/2;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("distance: ");
  display.println(distance);
  display.display();

  if (distance < 30){
    run_motor(1, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
    run_motor(1, LEFT_F_MOTOR,  LEFT_B_MOTOR);
  }
  else {

    // // take the average of the previous sonar reading and current sonar reading to prevent noise errors
    // if ((distance + prev_distance)/2 < SONAR_DETECTION_DISTANCE){
    //   //if distance detected by sonar is less than SONAR_DETECTION_DISTANCE, close whacker and raise elevator
    //   run_servo(whacker_servo, WHACKER_OPEN, WHACKER_CLOSE, 15);
    //   run_servo(whacker_servo, WHACKER_CLOSE, WHACKER_OPEN, 15);
    //   delay(100);

    //   run_servo(elevator_servo, ELEVATOR_DOWN, ELEVATOR_UP, 15);
    //   delay(1000);
    //   run_servo(elevator_servo, ELEVATOR_UP, ELEVATOR_DOWN, 15);
    //   delay(100);
      
    // }
    // prev_distance = distance;

    // // number of times the back hatch has collided
    // display.print("Collisions: ");
    // display.println(collisions);
    // display.display();

    // // if contact switch activated, lower the back hatch
    // if (collisions > prev_collisions){
    //   delay(2000);
    //   back_servo.write(HATCH_DOWN);
    //   delay(3000);
    //   prev_collisions = collisions;
    // }

    int position_L_analog = analogRead(TAPE_SENSOR_L);
    int position_R_analog = analogRead(TAPE_SENSOR_R);

    int position_L;
    int position_R;

    int error;

    //convert analog tape sensor reading to digital position
    if (position_L_analog > ON_OFF_THRESHOLD && position_L_analog < BAR_THRESHOLD) {
      position_L = 1; //on tape
    } 
    else if (position_L_analog > BAR_THRESHOLD){
      position_L = 5;
    }
    else {
      position_L = 0; //off tape
    }

    if (position_R_analog > ON_OFF_THRESHOLD && position_R_analog < BAR_THRESHOLD) {
      position_R = 1; //on tape
    } 
    else if (position_R_analog > BAR_THRESHOLD){
      position_R = 5;
    }
    else {
      position_R = 0; //off tape
    }

    display.print("left sensor: ");
    display.print(position_L_analog);
    display.print("  ");
    display.println(position_L);

    display.print("right sensor: ");
    display.print(position_R_analog);
    display.print("  ");
    display.println(position_R);

    display.display();

    if (position_L == 1 and position_R == 1) {
      if (prev_position_L == 1 && prev_position_R == 1){
        error = 0;
      }
      else if (prev_position_L == 0 && prev_position_R == 1){
        error = -1;
      }
      else if (prev_position_L == 1 && prev_position_R == 0){
        error = 1;
      }
    } 
    else if (position_L == 0 and position_R == 1) {
      if (prev_position_L == 1 && prev_position_R == 1){
        error = -1;
      }
      else if (prev_position_L == 0 && prev_position_R == 1){
        error = -2;
      }
      else if (prev_position_L == 0 && prev_position_R == 0){
        error = -3;
      }
    } 
    else if (position_L == 1 and position_R == 0) {
      if (prev_position_L == 1 && prev_position_R == 1){
        error = 1;
      }
      else if (prev_position_L == 1 && prev_position_R == 0){
        error = 2;
      }
      else if (prev_position_L == 0 && prev_position_R == 0){
        error = 3;
      }
    } 
    else if (position_L == 0 && position_R == 0){
      if (prev_position_L == 0 && prev_position_R == 1){
        error = -5;
      }
      else if (prev_position_L == 1 && prev_position_R == 0){
        error = 5;
      }
      else if (prev_position_L == 0 && prev_position_R == 0){
        if (previous_error < 0){
          error = -dist_between_sensors;
        }
        else {
          error = dist_between_sensors;
        }
      }
    }
    else if (position_R == 5 && position_L == 5){
      run_motor(-nominal_motor_R_duty, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
      run_motor(-nominal_motor_L_duty, LEFT_F_MOTOR, LEFT_B_MOTOR);
      error = 42;
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.print("at the bar ");
      display.display();
      delay(2000);
    }


    int p_gain = analogRead(p_pot); //purely for testing - once we have the final values we will remove the potentiometers
    int d_gain = analogRead(d_pot);


    // //int i_gain - probably shouldnt impement this...

    p_gain = p_gain * gain_scaling_factor;
    d_gain = d_gain * gain_scaling_factor;

    display.print("p gain: ");
    display.println(p_gain);
    display.print("d gain: ");
    display.println(d_gain);
    display.display();

    //proportional control
    double p = error*p_gain;

    // //derivative control - wrote when i was very tired so I may have made mistakes
    // double slope = 0;
    // double d = 0; //for very first state the derivative control should be 0

    // //should this go before or after the d calculation block? if before, then am taking the difference in time including the 
    // if (previous_error == error) {
    //   current_state_count++;
    // } 
    // else if (previous_error != STARTING_DUMMY_VALUE) {
    //   previous_state_count = current_state_count; // if a change in state has occurred, the current number of instances now becomes the number of instances in the previous state.
    //   previous_state = previous_error; //store the previous state when change occurs
    //   current_state_count = 1; //start at 1 because 1 instance has occurred.
    // }

    // if (previous_state_count != STARTING_DUMMY_VALUE) {
    //   slope = (double)(error - previous_state) / (previous_state_count + current_state_count - 1) * slope_scaling_factor; //-1 because i want it to be the true difference in time (time final - time initial), not with an extra increment (look at the graph at the end of lecture 5 to understand what I mean)
    //   d = slope*d_gain;
    // }

    // if (current_state_count > max_states_before_clear && error == 0){
    //   d = 0;
    //   prev_d = 0;
    // }
    double d = (error - previous_error) * d_gain;

    previous_error = error;
    //potential bug: derivative does not go to 0 as soon as tape sensors are aligned on tape. Have to test and see if this is an issue or not.

    int g = (p+d) * correction_scaling_factor;

    display.print("g ");
    display.println((int)g);
    display.print("error ");
    display.println(error);
    display.display();


    int motor_duty;
    if (error > 0) {
      motor_duty = nominal_motor_R_duty + g;
      if (motor_duty > max_motor_duty) {
        motor_duty = max_motor_duty;
      }
      if (motor_duty < 1){
        motor_duty = 1;
      }
      if (error == dist_between_sensors){
        run_motor(nominal_motor_R_duty, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
        run_motor(1, LEFT_F_MOTOR, LEFT_B_MOTOR);
      }
      else{
        run_motor(motor_duty, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
      }
    }
    else if (error < 0) {
      motor_duty = nominal_motor_L_duty + g*(-1);
      if (motor_duty > max_motor_duty) {
        motor_duty = max_motor_duty;
      }
      if (motor_duty < 1){
        motor_duty = 1;
      }

      if (error == -dist_between_sensors){
        run_motor(nominal_motor_L_duty ,LEFT_F_MOTOR, LEFT_B_MOTOR);
        run_motor(1, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
      }
      else{
        run_motor(nominal_motor_L_duty + g*(-1),LEFT_F_MOTOR, LEFT_B_MOTOR);
      }     
    } 
    else {
      run_motor(nominal_motor_R_duty, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
      run_motor(nominal_motor_L_duty, LEFT_F_MOTOR,  LEFT_B_MOTOR);
    }
  }
  //have to incorporate linearization through experimentation
  num_loops++;
};

