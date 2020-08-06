#define HATCH_DOWN 100
#define HATCH_UP 17
#define ELEVATOR_DOWN 45
#define ELEVATOR_UP 170
#define WHACKER_OPEN 0
#define WHACKER_CLOSE 180

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
