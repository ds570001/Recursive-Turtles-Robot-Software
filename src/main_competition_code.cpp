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

#define ON_OFF_THRESHOLD 200 // above this number is when tape sensors detect black tape.
#define STARTING_DUMMY_VALUE 10000 // starting dummy value to begin robot sequences.

#define ALUMINUM_THRESHOLD 52 // below this number is when tape sensors detect aluminum foil.
#define REVERSE_CORRECTION 1300 // make this less to make it turn left less (if looking at robot from straight on)

//******PICKING UP CANS******

#define collector_pin PA7
#define elevator_pin PB1
#define hatch_pin PB0
#define TRIG PB3
#define ECHO PA15
#define COLLISION_SENSOR PA11 // senses when the collision with the recycling bin has occurred (due to contact switch being pressed) when reversing

#define SONAR_DETECTION_DISTANCE 36 // optimal distance to detect cans


#define HATCH_DOWN 150 // sets the hatch servo so that the hatch is down
#define HATCH_UP 60 // sets the hatch servo so that the hatch is up
#define ELEVATOR_DOWN 10 // sets the elevator servo so that the elevator arm is down
#define ELEVATOR_DOWN_DRIVING 26 // sets the elevator arm height for when the robot is driving (slightly raised off floor)
#define ELEVATOR_UP 157 // sets the elevator servo so that the elevator arm is up
#define COLLECTOR_OPEN 0 // sets the collector servo so that the collector arm is open
#define COLLECTOR_CLOSE 180 // sets the collector servo so that collector arm swings closed.

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//******MOVEMENT******

int previous_error = STARTING_DUMMY_VALUE; //dummy starting value
int dist_between_sensors = 13; //metric for the horizontal distance between tape sensors - used as a max turning value
double gain_scaling_factor = 2.0; //scales up the gain input from the potentiometers.
int current_state_count = 1; // starts state count
int previous_state_count = STARTING_DUMMY_VALUE; //dummy starting value
int previous_state = STARTING_DUMMY_VALUE; //dummy starting value;
double correction_scaling_factor = 2.0; //correction factor to get the g value within max motor duty
int max_motor_duty = 65535; //max number the duty can be in motor format - motor spins fastest
int min_motor_duty = 1; //stops the motor
int nominal_motor_L_duty = 33000; //left wheel nominal speed for forward driving
int nominal_motor_R_duty = 35000; //right wheel nominal speed for forward driving
int reverse_motor_L_duty = 31000; //left wheel nominal speed for reverse driving
int reverse_motor_R_duty = 33000; //right wheel nominal speed for reverse driving
int difference_in_L_duty_from_optimal = nominal_motor_L_duty - 31000; //tuning factor
int difference_in_R_duty_from_optimal = nominal_motor_R_duty - 33000; //tuning factor
int num_loops = 0;
double slope_scaling_factor = 1000.0; // scales the d gain by a certain amount.

//******PICKING UP CANS******

Servo collector;
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
  pinMode(MOTOR_L_R, OUTPUT); //PA_2 and PA_3 dont work as analog outputs for motors
  pinMode(MOTOR_R_F, OUTPUT);
  pinMode(MOTOR_R_R, OUTPUT);

  //run_motor(nominal_motor_L_duty,MOTOR_L_F,MOTOR_L_R); figure out a better way to start servos up.
  //run_motor(nominal_motor_R_duty,MOTOR_R_F,MOTOR_R_R);

  //******PICKING UP CANS******

  //Servos
  collector.attach(collector_pin);
  elevator.attach(elevator_pin);
  hatch.attach(hatch_pin);
  
  collector.write(COLLECTOR_OPEN);
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

}


//******MOVEMENT******

void run_motor(int duty, PinName motorPin_F, PinName motorPin_B) {
  /*
  PURPOSE: runs DC motor with the given duty
  duty: if > 0, turn motor forward
        if < 0, turn motor backward
  if the duty scaling is negative then run the motor backwards, if positive then run motor forwards
  */
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

void stop_motors(){
  /*
  PURPOSE: set both DC motors to 0, stopping movement
  */
  pwm_start(MOTOR_L_F, PWMfreq, 1, TICK_COMPARE_FORMAT);
  pwm_start(MOTOR_L_R, PWMfreq, 1, TICK_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, PWMfreq, 1, TICK_COMPARE_FORMAT);
  pwm_start(MOTOR_R_R, PWMfreq, 1, TICK_COMPARE_FORMAT);
}

bool tape_follow() {
  /*
  PURPOSE: perform PID control for following black tape track and stopping when aluminum square is detected.
  */

  int position_L_analog = analogRead(TAPE_SENSOR_L);
  int position_R_analog = analogRead(TAPE_SENSOR_R);
  int position_L;
  int position_R;
  int error;

  //check if aluminum square is detected
  if (position_L_analog < ALUMINUM_THRESHOLD && position_R_analog < ALUMINUM_THRESHOLD) {
    stop_motors();
    delay(50);
    collisions = 0; //resets any false positive collision detections from the contact switch
    prev_collisions = 0;
    run_motor((-1)*reverse_motor_L_duty-REVERSE_CORRECTION,MOTOR_L_F,MOTOR_L_R);
    run_motor((-1)*reverse_motor_R_duty,MOTOR_R_F,MOTOR_R_R); //reverse motors
    delay(1000);
    return true;
  }

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

  int p_gain = analogRead(p_pot); //potentiometers for modifying gains.
  int d_gain = analogRead(d_pot);

  p_gain = p_gain * gain_scaling_factor;
  d_gain = d_gain * gain_scaling_factor;

  // p_gain = 950;
  // d_gain = 60;

  //proportional control
  double p = error*p_gain;

  //derivative control
  double slope = 0;
  double d = 0; //for very first state the derivative control should be 0

  if (previous_error == error) {
    current_state_count++;
  } else if (previous_error != STARTING_DUMMY_VALUE) {
    previous_state_count = current_state_count; // if a change in state has occurred, the current number of instances now becomes the number of instances in the previous state.
    previous_state = previous_error; //store the previous state when change occurs
    current_state_count = 1; //start at 1 because 1 instance has occurred.
  }

  if (previous_state_count != STARTING_DUMMY_VALUE) {
     slope = (double)(error - previous_state) / (previous_state_count + current_state_count - 1) * slope_scaling_factor; //-1 because want it to be the true difference in time (time final - time initial), not with an extra increment
     d = slope*d_gain;
  }
  previous_error = error;

  int g = (p+d) * correction_scaling_factor;

  if (num_loops % 100 == 0) { //shouldnt print to screen every loop -> slows down tape following execution
    
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

    display.display();
  }

  if (error == dist_between_sensors || error == dist_between_sensors*(-1)) {
    if (g > 0) { //turn right
      run_motor(nominal_motor_R_duty + g,MOTOR_R_F,MOTOR_R_R);
      run_motor(nominal_motor_L_duty - g*1.5 - difference_in_L_duty_from_optimal,MOTOR_L_F,MOTOR_L_R);
    } else if (g < 0) {
      //turn left
      run_motor(nominal_motor_L_duty + g*(-1),MOTOR_L_F,MOTOR_L_R);
      run_motor(nominal_motor_R_duty - g*(-1)*3/4 - difference_in_R_duty_from_optimal,MOTOR_R_F,MOTOR_R_R);
    } else {
      //drive straight
      run_motor(nominal_motor_R_duty,MOTOR_R_F,MOTOR_R_R);
      run_motor(nominal_motor_L_duty,MOTOR_L_F,MOTOR_L_R);
    }
  } else {
    if (g > 0) { //turn right
      run_motor(nominal_motor_R_duty + g,MOTOR_R_F,MOTOR_R_R);
      run_motor(nominal_motor_L_duty - g/2,MOTOR_L_F,MOTOR_L_R);
    } else if (g < 0) { //turn left
      run_motor(nominal_motor_L_duty + g*(-1),MOTOR_L_F,MOTOR_L_R);
      run_motor(nominal_motor_R_duty - g*(-1)/2,MOTOR_R_F,MOTOR_R_R);
    } else { //drive straight
      run_motor(nominal_motor_R_duty,MOTOR_R_F,MOTOR_R_R);
      run_motor(nominal_motor_L_duty,MOTOR_L_F,MOTOR_L_R);
    }
  }

  num_loops++;
  return false;
}

//******PICKING UP CANS******

void run_servo(Servo this_servo, int start, int end, int delay_val){
  /*
  PURPOSE: sets servo to desired position at a desired speed.
  */
  if (start < end){
    for (int angle = start; angle <= end; angle+= 1){
      this_servo.write(angle);
      delay(delay_val);
    }
  }
  else{
    for (int angle = start; angle >= end; angle-= 1){
      this_servo.write(angle);
      delay(delay_val);
    }
  }
}


void zero_servos(){
  /*
  PURPOSE: set all servos to starting position.
  */
  collector.write(COLLECTOR_OPEN);
  elevator.write(ELEVATOR_DOWN);
  hatch.write(HATCH_UP);
  return;
}

double get_distance(){
  /*
  PURPOSE: detect the distance to the closest object in front of the sonar
  */
  // read distance with sonar
  digitalWrite(TRIG, LOW);
  delay(1);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG, HIGH);
  delay(5);
  digitalWrite(TRIG, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  double duration = pulseIn(ECHO, HIGH);
  // Calculating the distance
  double distance = duration*0.034/2;
  return distance;
}

void perform_can_collection(){
  /*
  PURPOSE: closes collector arm, opens collector arm, raises elevator arm, lowers elevator arm to collect can
  */
  collector.write(COLLECTOR_OPEN);
  elevator.write(ELEVATOR_DOWN);
  hatch.write(HATCH_UP);
  //if distance detected by sonar is less than SONAR_DETECTION_DISTANCE, close collector and raise elevator
  int collector_delay = 4;
  run_servo(collector, COLLECTOR_OPEN, COLLECTOR_CLOSE, collector_delay);
  run_servo(collector, COLLECTOR_CLOSE, COLLECTOR_OPEN+140, collector_delay);
  run_servo(collector, COLLECTOR_OPEN+140, COLLECTOR_CLOSE, collector_delay);
  delay(400);
  run_servo(collector, COLLECTOR_CLOSE, COLLECTOR_OPEN, collector_delay);

  int elevator_delay = 5;
  run_servo(elevator,ELEVATOR_DOWN,ELEVATOR_UP,elevator_delay);
  delay(100);
  run_servo(elevator, ELEVATOR_UP, ELEVATOR_DOWN_DRIVING, elevator_delay);
  delay(300);
  return;
}

void activate_back_trigger() {
  /*
  PURPOSE: callback function that stops motors once the contact switch is pressed
  */
  collisions++; // increment collisions global variable so that check_collision() function knows the robot
                // has backed up against with the recycling bin
  stop_motors();
}

bool check_collision() {
  /*
  PURPOSE: checks whether the robot has backed up against the recycling bin
  */

  if (collisions > prev_collisions) {
    //performs sequence to drop off cans into the recycling bin
    stop_motors();
    hatch.write(HATCH_DOWN);
    delay(700);
    hatch.write(HATCH_DOWN-5);
    //perform shaking sequence where the robot drives back and forth to shake the cans off the top ramp.
    run_motor(reverse_motor_L_duty,MOTOR_L_F,MOTOR_L_R);
    run_motor(reverse_motor_R_duty,MOTOR_R_F,MOTOR_R_R);
    delay(400);
    stop_motors();
    run_motor((-1)*reverse_motor_L_duty-REVERSE_CORRECTION,MOTOR_L_F,MOTOR_L_R);
    run_motor((-1)*reverse_motor_R_duty,MOTOR_R_F,MOTOR_R_R);
    delay(700);
    stop_motors();
    run_motor(reverse_motor_L_duty,MOTOR_L_F,MOTOR_L_R);
    run_motor(reverse_motor_R_duty,MOTOR_R_F,MOTOR_R_R);
    delay(400);
    stop_motors();
    run_motor((-1)*reverse_motor_L_duty-REVERSE_CORRECTION,MOTOR_L_F,MOTOR_L_R);
    run_motor((-1)*reverse_motor_R_duty,MOTOR_R_F,MOTOR_R_R);
    delay(700);
    stop_motors();
    delay(300);
    hatch.write(HATCH_UP);
    prev_collisions = collisions;
    return true;
  } else {
    return false;
  }
}

int sonar_loop_count = 0;
int num_cans = 0; //counts the number of cans that have been collected
bool is_reversing = false;
bool had_collision = false;
void loop() {
  //general loop function controlling robot sequences

  if (is_reversing) {
    had_collision = check_collision();
    if (had_collision) {
      had_collision = false;
      is_reversing = false;
      num_cans = 0; //reset flag variables and number of cans if collision occurred (cans have been dropped off at this point)
    } else {
      //otherwise continue reversing
      run_motor((-1)*nominal_motor_L_duty-REVERSE_CORRECTION,MOTOR_L_F,MOTOR_L_R);
      run_motor((-1)*nominal_motor_R_duty,MOTOR_R_F,MOTOR_R_R);
      delay(1000);
    }
  } else {
    //if robot is not reversing perform PID control and sonar sampling.

    if (sonar_loop_count % 35 == 0) {
      int distance = get_distance();
      if (distance < SONAR_DETECTION_DISTANCE && num_cans < 4) { //should only do 4 can attempts per run. Robot can only fit at most 4 cans on its top ramp.
        if (distance > 15) {
          for (int j = 0; j <= 50; j++) {
            tape_follow();
            delay(1);
          }
        }
        stop_motors();
        delay(10);
        perform_can_collection();
        num_cans++;
      }
    }

    is_reversing = tape_follow(); // returns true if reversing.
    sonar_loop_count++;
  }
}
