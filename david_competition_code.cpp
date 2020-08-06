#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_SSD1306.h>

#define PWMfreq 2000
#define LEFT_F_MOTOR PB_8
#define LEFT_B_MOTOR PB_9
#define RIGHT_F_MOTOR PB_7
#define RIGHT_B_MOTOR PB_6

#define TAPE_SENSOR_R PA_3
#define TAPE_SENSOR_L PA_4
#define ON_OFF_THRESHOLD 200 //can investigate this number more... above this number is when it is on tape
#define BAR_THRESHOLD_HIGH 37
#define BAR_THRESHOLD_LOW 27
#define STARTING_DUMMY_VALUE 10000
int prev_position_L = 0, prev_position_R = 0;

//values that have worked
// (p,d) = (378, 120)
// (p,d) this follows the whole path with spin (436, 159)
// 507 ,123 -> folows first part but can't turn the last part tight enough
// 645, 123 -> last loop to back up worked once
// 306, 970 -> can do competition

#define p_pot PA1
#define d_pot PA2
#define G_FACTOR 5 //play around with this value to get g value to fit within duty range.
#define P_READ_FACTOR 1.0 //scales down the gain input from the potentiometers.
#define D_READ_FACTOR 0.3
#define DIST_BETWEEN_SENSORS 5 //may have to do more precise measurements on this...
#define MAX_MOTOR_DUTY 65535 //max number the duty can be in motor format
#define NOMINAL_MOTOR_L_DUTY 30000
#define NOMINAL_MOTOR_R_DUTY 30000
#define REVERSE_DUTY -30000
#define REVERSE_FACTOR 1.05
#define OTHER_WHEEL_CORRECTION_FACTOR 1.6
#define MAX_STATES_BEFORE_CLEAR 15
#define SLOPE_SCALING_FACTOR 30.0
#define AT_BAR_ERROR 50
#define SPIN_DUTY 32000
#define RIGHT_SPIN_DUTY 33000
#define LOOPS_BEFORE_SLOW 20
#define COUNTERCLOCKWISE 1
#define CLOCKWISE -1
int previous_error = STARTING_DUMMY_VALUE; //dummy starting value
int current_state_count = 1; //beginning of counting
int previous_state_count = STARTING_DUMMY_VALUE; //dummy starting value
int previous_state = STARTING_DUMMY_VALUE; //dummy starting value;
int num_loops = 0;
int loop_count = 0;

#define ELEVATOR_SERVO PB0
#define WHACKER_SERVO PB1
#define BACK_SERVO PA7
#define HATCH_DOWN 100
#define HATCH_UP 17
#define ELEVATOR_DOWN 45
#define ELEVATOR_UP 170
#define WHACKER_OPEN 0
#define WHACKER_CLOSE 180

#define COLLISION_SENSOR PA6

#define LOOPS_PER_SONAR_DETECTION 15
#define ECHO PA11
#define TRIG PA12
#define MIN_DETECTION_DIST 32
#define COLLECTION_DIST 20
#define WHACKER_DELAY 25
double distance = 0.0;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Servo whacker_servo;
Servo elevator_servo;
Servo back_servo;

// TwoWire Wire(SCL, SDA, SOFT_STANDARD);

// interrupt 
volatile int prev_collisions = 0;
volatile int collisions = 0;
void activate_back_trigger(){
  collisions ++;
}

// set all servos to zero position
void zero_servos(){
  whacker_servo.write(WHACKER_OPEN);
  elevator_servo.write(ELEVATOR_DOWN);
  back_servo.write(HATCH_UP);
  return;
}

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

  zero_servos();
}

// clears display, sets cursor and text size and color
void clear_display(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
}

//set both motors to 0
void stop_motors(){
  pwm_start(LEFT_F_MOTOR, PWMfreq, 1, TICK_COMPARE_FORMAT);
  pwm_start(LEFT_B_MOTOR, PWMfreq, 1, TICK_COMPARE_FORMAT);
  pwm_start(RIGHT_F_MOTOR, PWMfreq, 1, TICK_COMPARE_FORMAT);
  pwm_start(RIGHT_B_MOTOR, PWMfreq, 1, TICK_COMPARE_FORMAT);
}

// runs a motor with the given duty
void run_motor(int duty, PinName motorPin_F, PinName motorPin_B) {
  //duty: if > 0, turn motor forward as described above
  //      if < 0, turn motor backward as described above
  //if the duty scaling is negative then run the motor backwards, if positive then run motor forwards
  if (duty > 0) {
    pwm_start(motorPin_B, PWMfreq, 1, TICK_COMPARE_FORMAT);
    pwm_start(motorPin_F, PWMfreq, duty, TICK_COMPARE_FORMAT);
    delay(2);
  } else {
    duty = duty*(-1);
    pwm_start(motorPin_F, PWMfreq, 1, TICK_COMPARE_FORMAT);
    if (duty == 0) {
      duty = 1;
    }
    pwm_start(motorPin_B, PWMfreq, duty, TICK_COMPARE_FORMAT);
    delay(2);
  }
};

// get the robot to spin in a circle
void spin(int direction){

  if (direction == COUNTERCLOCKWISE){
    run_motor(RIGHT_SPIN_DUTY, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
    run_motor(-20000, LEFT_F_MOTOR, LEFT_B_MOTOR);
    delay(10);
  } 
  else if (direction == CLOCKWISE){
    run_motor(-20000, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
    run_motor(SPIN_DUTY, LEFT_F_MOTOR, LEFT_B_MOTOR);   
    delay(10);
  }

}

void back_spin(int direction){
  if (direction == COUNTERCLOCKWISE){
    run_motor(1, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
    run_motor(-MAX_MOTOR_DUTY, LEFT_F_MOTOR, LEFT_B_MOTOR);
    delay(20);
  } 
  else if (direction == CLOCKWISE){
    run_motor(-MAX_MOTOR_DUTY, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
    run_motor(1, LEFT_F_MOTOR, LEFT_B_MOTOR);
    delay(20);   
  }
}

// runs a certain servo from angle start to angle end, increment angle by int speed each loop
void run_servo(Servo this_servo, int start, int end, int speed){
  if (start < end){
    for (int angle = start; angle <= end; angle++){
      this_servo.write(angle);
      delay(speed);
    }
  }
  else{
    for (int angle = start; angle >= end; angle--){
      this_servo.write(angle);
      delay(speed);
    }
  }
  this_servo.write(end);
}

//get distance with the sonar
double get_distance(){
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

// opens whacker, closes whacker, raises elevator, lowers elevator to collect can
void perform_can_collection(){
  clear_display();
  display.println("performing can collection");
  display.display();

  zero_servos();
  int whacker_speed = 4;
  run_servo(whacker_servo, WHACKER_OPEN, WHACKER_CLOSE - 50, whacker_speed);
  run_servo(whacker_servo, WHACKER_CLOSE - 50, WHACKER_CLOSE - 110, whacker_speed);
  run_servo(whacker_servo, WHACKER_CLOSE - 110, WHACKER_CLOSE, whacker_speed);
  delay(100);
  run_servo(whacker_servo, WHACKER_CLOSE, WHACKER_OPEN, whacker_speed);

  int elevator_speed = 4;
  run_servo(elevator_servo, ELEVATOR_DOWN, ELEVATOR_UP - 30, elevator_speed);
  run_servo(elevator_servo, ELEVATOR_UP - 30, ELEVATOR_UP - 70, elevator_speed);
  run_servo(elevator_servo, ELEVATOR_UP - 60, ELEVATOR_UP, elevator_speed);
  delay(100);
  run_servo(elevator_servo, ELEVATOR_UP, ELEVATOR_DOWN, elevator_speed);
}

// gets the reading of a tape sensor
int get_sensor_position (PinName sensor){
  double position_analog = analogRead(sensor);
  int position = 0;

  //convert analog tape sensor reading to digital position
  if (position_analog > ON_OFF_THRESHOLD) {
    position = 1; //on tape
  } 
  else if (position_analog < BAR_THRESHOLD_HIGH && position_analog > BAR_THRESHOLD_LOW){
    if (analogRead(sensor) < BAR_THRESHOLD_HIGH && analogRead(sensor) > BAR_THRESHOLD_LOW){
      position = AT_BAR_ERROR; //at the bar
    }
    else {
      position = 0;
    }
  }
  else {
    position = 0; //off tape
  }
  
  if (sensor == TAPE_SENSOR_L){
    display.print("left: ");
    display.print(position_analog);
    display.print("   ");
    display.println(position);
  }
  else if (sensor == TAPE_SENSOR_R){
    display.print("right: ");
    display.print(position_analog);
    display.print("   ");
    display.println(position);
  }

  return position;
}

// get the error given the tape sensor positions
int get_error(int prev_position_L, int position_L, int prev_position_R, int position_R){
  int error;
  if (position_L == 1 and position_R == 1) {
    if (prev_position_L == 1 && prev_position_R == 1){
      error = 0;
    }
    else if (prev_position_L == 0 && prev_position_R == 1){
      error = -2;
    }
    else if (prev_position_L == 1 && prev_position_R == 0){
      error = 2;
    }
    else if (prev_position_L == 0 && prev_position_R == 0){
      if (previous_error > 0){
        error = 1;
      }
      else if (previous_error < 0){
        error = -1;
      }
    }
  } 
  else if (position_L == 0 and position_R == 1) {
    if (prev_position_L == 1 && prev_position_R == 1){
      error = -2;
    }
    else if (prev_position_L == 0 && prev_position_R == 1){
      error = -3;
    }
    else if (prev_position_L == 0 && prev_position_R == 0){
      error = -4;
    }
  } 
  else if (position_L == 1 and position_R == 0) {
    if (prev_position_L == 1 && prev_position_R == 1){
      error = 2;
    }
    else if (prev_position_L == 1 && prev_position_R == 0){
      error = 3;
    }
    else if (prev_position_L == 0 && prev_position_R == 0){
      error = 4;
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
        error = -DIST_BETWEEN_SENSORS;
      }
      else {
        error = DIST_BETWEEN_SENSORS;
      }
    }
    else if (prev_position_L == 1 && prev_position_R == 1){
      error = previous_error;
    }
  }
  else if (position_R == 5 && position_L == 5){
    return AT_BAR_ERROR;
  }
  return error;
}

//Dannon's original function
int get_simple_error(int previous_error, int position_L, int position_R){
  int error;
  if (position_L == AT_BAR_ERROR and position_R == AT_BAR_ERROR){
    return AT_BAR_ERROR;
  }
  else if (position_L == 1 and position_R == 1) {
    error = 0; //no error since both are on tape
  } 
  else if (position_L == 0 and position_R == 1) {
    error = -1; //left sensor is off tape and right is on tape... think of negative when robot is moving left
  } 
  else if (position_L == 1 and position_R == 0) {
    error = 1; //left sensor is on tape and right is off tape... think of positive when robot is moving right
  } 
  else if (previous_error == -1 || previous_error == -DIST_BETWEEN_SENSORS) { //at this point both sensors must be off the tape. Must remember where it was the previous loop...
    error = -DIST_BETWEEN_SENSORS; //need an error proportional to the distance between the sensors so you get linear PID control
    //negative means that the left tape sensor left the tape first, therefore the robot must turn right to get back on track
  } 
  else if (previous_error == 1 || previous_error == DIST_BETWEEN_SENSORS) {
    error = DIST_BETWEEN_SENSORS;
    //positive means that the right tape sensor left the tape first, therefore the robot must turn left to get back on track
  } 
  else if (previous_error == 0) {
    error = -DIST_BETWEEN_SENSORS; // pretty sure this case is impossible, but just in case, I'm going with the 50% chance that the robot will be on the right side of the tape...
  }
  return error;
}

// returns proportional term given the error
double get_p(int error){
  //double p_gain = analogRead(p_pot); //purely for testing - once we have the final values we will remove the potentiometers
  double p_gain = 970;
  //p_gain * P_READ_FACTOR;
  
  // display.print("p gain: ");
  // display.println(p_gain);
  return error*p_gain;
}

//returns derivative term given the error and previous error
double get_d(int error, int previous_error){
  // double d_gain = analogRead(d_pot);
  double d_gain = 306;
  // d_gain * D_READ_FACTOR;


  // display.print("d gain: ");
  // display.println(d_gain);

  double d = 0.0;

  if (previous_error == error) {
    current_state_count++;
  } 
  else if (previous_error != STARTING_DUMMY_VALUE) {
    previous_state_count = current_state_count; // if a change in state has occurred, the current number of instances now becomes the number of instances in the previous state.
    previous_state = previous_error; //store the previous state when change occurs
    current_state_count = 1; //start at 1 because 1 instance has occurred.
  }

  if (previous_state_count != STARTING_DUMMY_VALUE) {
    double slope = (double)(error - previous_state) / (previous_state_count + current_state_count - 1) * SLOPE_SCALING_FACTOR; //-1 because i want it to be the true difference in time (time final - time initial), not with an extra increment (look at the graph at the end of lecture 5 to understand what I mean)
    d = slope*d_gain;
  }

  if (current_state_count > MAX_STATES_BEFORE_CLEAR && error == 0){
    d = 0.0;
  }

  return d;
}

// double get_chain_d(){
  // d += error - previous_error;
  // for (int i = ERROR_CHAIN_SIZE-1; i > 0; i--){
  //   d += errors[i] - errors[i-1];
  // }
  // for (int i = 0; i < ERROR_CHAIN_SIZE-1; i++){
  //   errors[i] = errors[i+1];
  // }
  // errors[ERROR_CHAIN_SIZE-1] = error;
  // d = (d/ERROR_CHAIN_SIZE) * d_gain;
  // d = (error - previous_error) * d_gain;
// }

//corrects the motor duties based on g value
void adjust_motors(double g){
  int motor_duty, other_duty;
  if (g > 0) {
    motor_duty = NOMINAL_MOTOR_R_DUTY + g;
    other_duty = NOMINAL_MOTOR_L_DUTY - g*OTHER_WHEEL_CORRECTION_FACTOR;
    // display.print("motor duty: ");
    // display.println(motor_duty);

    if (motor_duty > MAX_MOTOR_DUTY) {
      motor_duty = MAX_MOTOR_DUTY;
    }
    if (motor_duty == 0){
      motor_duty = 1;
    }
    run_motor(motor_duty, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
    run_motor(other_duty, LEFT_F_MOTOR, LEFT_B_MOTOR);
  }
  else if (g < 0) {
    motor_duty = NOMINAL_MOTOR_L_DUTY - g;
    other_duty = NOMINAL_MOTOR_R_DUTY + g*OTHER_WHEEL_CORRECTION_FACTOR;
    // display.print("motor duty: ");
    // display.println(motor_duty);

    if (motor_duty > MAX_MOTOR_DUTY) {
      motor_duty = MAX_MOTOR_DUTY;
    }
    if (motor_duty == 0){
      motor_duty = 1;
    }

    run_motor(motor_duty, LEFT_F_MOTOR, LEFT_B_MOTOR);
    run_motor(other_duty, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
       
  } 
  else {
    run_motor(NOMINAL_MOTOR_R_DUTY, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
    run_motor(NOMINAL_MOTOR_L_DUTY, LEFT_F_MOTOR,  LEFT_B_MOTOR);
  }
}

// returns true if at bar
bool tape_follow(){
  elevator_servo.write(ELEVATOR_DOWN + 10);
  back_servo.write(HATCH_UP);
  int position_L = get_sensor_position(TAPE_SENSOR_L);
  int position_R = get_sensor_position(TAPE_SENSOR_R);

  // int error = get_error(prev_position_L, position_L, prev_position_R, position_R);
  int error = get_simple_error(previous_error, position_L, position_R);
  if (error == AT_BAR_ERROR){
    return true;
  }
  prev_position_L = position_L;
  prev_position_R = position_R;

  double p = get_p(error);
  double d = get_d(error, previous_error);
  previous_error = error;

  double g = (p+d) * G_FACTOR;
  // display.print("error:");
  // display.print(error);
  // display.print("g:");
  // display.println(g);

  if (error == DIST_BETWEEN_SENSORS && current_state_count >= LOOPS_BEFORE_SLOW){
    spin(COUNTERCLOCKWISE);
  } else if (error == -DIST_BETWEEN_SENSORS && current_state_count >= LOOPS_BEFORE_SLOW){
    spin(CLOCKWISE);
  } else{
    adjust_motors(g);
  }
  //have to incorporate linearization through experimentation
  num_loops++;
  return false;
}

// run this to test the servos
void test_servos(){
  run_servo(whacker_servo, WHACKER_OPEN, WHACKER_CLOSE, 15);
  run_servo(whacker_servo, WHACKER_CLOSE, WHACKER_OPEN, 10);
  
  run_servo(elevator_servo, ELEVATOR_DOWN, ELEVATOR_UP, 10);
  run_servo(elevator_servo, ELEVATOR_UP, ELEVATOR_DOWN, 15);

  
  run_servo(back_servo, HATCH_UP, HATCH_DOWN, 15);
  run_servo(back_servo, HATCH_DOWN, HATCH_UP, 10);

}

void back_up_until_bin(){
  // clear_display();
  display.println("backing up");
  back_servo.write(HATCH_UP);
  display.display();
  stop_motors();

  run_motor(REVERSE_DUTY*1.1, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
  run_motor(REVERSE_DUTY, LEFT_F_MOTOR, LEFT_B_MOTOR);
  delay(500);
  stop_motors();
  while(prev_collisions == collisions){
    run_motor(REVERSE_DUTY, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
    run_motor(REVERSE_DUTY*1.05, LEFT_F_MOTOR, LEFT_B_MOTOR);
    delay(10);
  }
  stop_motors();
  prev_collisions = collisions;
}

void lurch(){
  int delay_time_forward = 100;
  int delay_time_back = 200;
  run_motor(NOMINAL_MOTOR_R_DUTY, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
  run_motor(NOMINAL_MOTOR_L_DUTY, LEFT_F_MOTOR, LEFT_B_MOTOR);
  delay(delay_time_forward);
  run_motor(REVERSE_DUTY*REVERSE_FACTOR, RIGHT_F_MOTOR, RIGHT_B_MOTOR);
  run_motor(REVERSE_DUTY, LEFT_F_MOTOR, LEFT_B_MOTOR);
  delay(delay_time_back);
}

void release_cans(){
  clear_display();
  display.println("releasing cans");
  display.display();
  back_servo.write(HATCH_UP);
  run_servo(back_servo, HATCH_UP, HATCH_DOWN, 2);
  delay(500);
  lurch();
  delay(200);
  lurch();
  stop_motors();
  delay(300);
  run_servo(back_servo, HATCH_DOWN, HATCH_UP, 2);
}

void check_sensors(){
  clear_display();
  get_sensor_position (TAPE_SENSOR_L);
  get_sensor_position (TAPE_SENSOR_R);
  // display.print("d gain: ");
  // display.println(analogRead(d_pot) * D_READ_FACTOR);
  // display.print("p gain: ");
  // display.println(analogRead(p_pot) * P_READ_FACTOR);
  display.display();
}

void wag(){
  run_servo(back_servo, HATCH_UP, HATCH_UP + 60, 5);
  run_servo(back_servo, HATCH_UP + 60, HATCH_UP, 5);
}

void do_entertainment(){
  zero_servos();
  stop_motors();
  while (true){
    clear_display();
    display.print("PLAYING CATCH");
    display.display();
    zero_servos();
    if (get_distance() < 35 && get_distance() > 25){
      delay(500);
      whacker_servo.write(WHACKER_OPEN);
      delay(500);
      whacker_servo.write(WHACKER_OPEN + 40);
      delay(500);
      wag();
      wag();
      wag();
    }
  }
}

void loop() {
  if (collisions > 15){
    clear_display();
    display.print("collisions: ");
    display.println(collisions);
    display.display();
    if (collisions > 18){
      stop_motors();
      do_entertainment();
    }
  }
  if (loop_count % 150 == 0){
    check_sensors();
  }
  if (loop_count % LOOPS_PER_SONAR_DETECTION == 0){
    distance = get_distance();
  }

  if (distance < MIN_DETECTION_DIST){
    if (get_distance() < MIN_DETECTION_DIST){
      if (distance < COLLECTION_DIST){
        stop_motors();
        delay(100);
        perform_can_collection();
        distance = get_distance();
      }
      else {
        for (int i = 0; i < WHACKER_DELAY; i ++){
          tape_follow();
        }
        stop_motors();
        delay(100);
        perform_can_collection();
        distance = get_distance();
      }

    }
  } 
  else{
    if(tape_follow()){
      stop_motors();
      delay(1000);
      back_up_until_bin();
      release_cans();
    }
  }
  loop_count ++;
  prev_collisions = collisions;
}

