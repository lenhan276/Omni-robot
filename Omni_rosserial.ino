#include <SoftwareSerial.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

#include <AFMotor.h>
#include <PID_v1.h>

//define encoder pins
const byte encoder_pinA_1 = 18;//A pin -> the interrupt pin 5
const byte encoder_pinB_1 = 34;//B pin -> the digital pin 34

const byte encoder_pinA_2 = 19;//A pin -> the interrupt pin 5
const byte encoder_pinB_2 = 36;//B pin -> the digital pin 36

const byte encoder_pinA_3 = 20;//A pin -> the interrupt pin 5
const byte encoder_pinB_3 = 38;//B pin -> the digital pin 38

const byte encoder_pinA_4 = 21;//A pin -> the interrupt pin 5
const byte encoder_pinB_4 = 40;//B pin -> the digital pin 40

//define message values from ROS topics 
double get_x;
double get_z;
double get_ipspeed;
double max_speed;
double inputDegree;

//define required values for 4 motors
byte encoder_PinALast_1;
byte encoder_PinALast_2;
byte encoder_PinALast_3;
byte encoder_PinALast_4;

//the rotation direction 
boolean Direction_1, Direction_2, Direction_3, Direction_4;

//the number of the pulses
double pulse_1,abs_pulse_1;
double pulse_2,abs_pulse_2;
double pulse_3,abs_pulse_3;
double pulse_4,abs_pulse_4;

//Power supplied to the motor PWM value.
double val_output_1;
double val_output_2;
double val_output_3;
double val_output_4;

//define PID values
boolean result_1,result_2,result_3,result_4;
double Setpoint1,Setpoint2,Setpoint3,Setpoint4;
double Speed1,Speed2,Speed3,Speed4;

double Kp_1 = 1.89, Ki_1 = 0, Kd_1 = 0;
double Kp_2 = 1.86, Ki_2 = 0, Kd_2 = 0;
double Kp_3 = 1.86, Ki_3 = 0, Kd_3 = 0;
double Kp_4 = 1.84, Ki_4 = 0, Kd_4 = 0;

//setup PID lib and motors
PID myPID_1(&abs_pulse_1, &val_output_1, &Setpoint1, Kp_1, Ki_1, Kd_1, DIRECT);
PID myPID_2(&abs_pulse_2, &val_output_2, &Setpoint2, Kp_2, Ki_2, Kd_2, DIRECT);
PID myPID_3(&abs_pulse_3, &val_output_3, &Setpoint3, Kp_3, Ki_3, Kd_3, DIRECT);
PID myPID_4(&abs_pulse_4, &val_output_4, &Setpoint4, Kp_4, Ki_4, Kd_4, DIRECT);

AF_DCMotor motor1(1, MOTOR12_64KHZ); 
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR12_64KHZ);
AF_DCMotor motor4(4, MOTOR12_64KHZ);

ros::NodeHandle node;
geometry_msgs::Twist msg;

//function to get speed value when adjust speed slider in html
void getSpeed(const std_msgs::Int8& get_speed)
{
  double inputSpeed = get_speed.data;
  get_ipspeed = inputSpeed * 2;
}

//function to converse the message from cmd_vel topic to command motors
void roverCallBack(const geometry_msgs::Twist& cmd_vel)
{
  double x = cmd_vel.linear.x;
  double z = cmd_vel.angular.z;

  get_x = x;
  get_z = z;

  //condition of incoming message for the robot to move in desired directions when using joystick
  if(((x>0) && (z>0) && (abs(z) < abs(x))) || ((x>0) && (z<0) && (abs(z) < abs(x)))) 
  {
    forward();
  }
  else
  {
    if(((x>0) && (z>0) && (abs(z) > abs(x))) || ((x<0) && (z>0) && (abs(z) > abs(x)))) 
    {
      left();
    }
    else
    {
      if(((x>0) && (z<0) && (abs(z) > abs(x))) || ((x<0) && (z<0) && (abs(z) > abs(x))))
      {
        right();
      }
      else
      {
        if(((x<0) && (z>0) && (abs(z) < abs(x))) || ((x<0) && (z<0) && (abs(z) < abs(x))))
        {
          backward();
        }
        else
        {
          if(x == 0 && z == 0)
          {
            stopped();
          }        
        }
      }
    }
  }
  
  //condition of incoming message for the robot to move in desired directions when using button 
  if(x == 7){
    forward();
  }
    else{
      if(x == -7){
        left();
      }
      else{
        if(x == 8){
          right();
        }
        else{
          if(x == -8){
            backward();
          }
          else{
            if(x == 9){
              move45();
            }
            else{
              if (x == -9){
                move135();
              }
              else{
                if(x == 10){
                  move225();
                }
                else{
                  if(x == -10){
                    move315();
                  }
                }
              }
            }
          }
        }
      }
    }
    
  if(x == 6){
    clockwise();
  }
  else{
    if (x == -6)
    counterclockwise();
  }
}
ros::Subscriber <geometry_msgs::Twist> sub("cmd_vel", roverCallBack);
ros::Subscriber <std_msgs::Int8> sub2("get_speed", getSpeed);

void setup() {
  Serial.begin(9600);//Initialize the serial port

  //PID is set to automatic mode
  
  myPID_1.SetMode(AUTOMATIC);
  myPID_2.SetMode(AUTOMATIC);
  myPID_3.SetMode(AUTOMATIC);
  myPID_4.SetMode(AUTOMATIC);

  //Set PID sampling frequency is 10ms

  myPID_1.SetSampleTime(10);
  myPID_2.SetSampleTime(10);
  myPID_3.SetSampleTime(10);
  myPID_4.SetSampleTime(10);
  
  //Initialize the modules
  
  EncoderInit_1();
  EncoderInit_2();
  EncoderInit_3();
  EncoderInit_4();
    
  //Initialize the modules
  
  node.initNode(); //Initialize your ROS node handle
  node.subscribe(sub); //subscribe to topics
  node.subscribe(sub2); 

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

}

void loop() {

  if ((get_x > 5)&&(get_z == 0){
    button_setPoint();
  }
  else{
    joystick_setPoint();
  }
  // Apply PID for motor 1
      abs_pulse_1 = abs(pulse_1);
      result_1=myPID_1.Compute();//PID conversion is complete and returns 1
      if(result_1)
      { 
        pulse_1 = 0; //Count clear, wait for the next count
        motor1.setSpeed(val_output_1);
      }
      
  // motor 2    
      abs_pulse_2 = abs(pulse_2);
      result_2=myPID_2.Compute();//PID conversion is complete and returns 1
      if(result_2)
      { 
        pulse_2 = 0; //Count clear, wait for the next count
        motor2.setSpeed(val_output_2);
      }
      
  // motor 3    
      abs_pulse_3 = abs(pulse_3);
      result_3=myPID_3.Compute();//PID conversion is complete and returns 1
      if(result_3)
      { 
        pulse_3 = 0; //Count clear, wait for the next count
        motor3.setSpeed(val_output_3);
      }
  // motor 4   
      abs_pulse_4 = abs(pulse_4);      
      result_4=myPID_4.Compute();//PID conversion is complete and returns 1
      if(result_4)
      { 
        pulse_4 = 0; //Count clear, wait for the next count
        motor4.setSpeed(val_output_4);
      }
  
  node.spinOnce(); //Communication callbacks
  delay(1);
  
}

///////////// Omni Robot direction configuration /////////////

void forward()
{
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  if (abs(get_x) > 5){
    button_setPoint();
  }
}

void left(){
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  if (abs(get_x) > 5){
    button_setPoint();
  }
  }

void right(){
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  if (abs(get_x) > 5){
    button_setPoint();
  }
  }

void backward()
{
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  if (abs(get_x) > 5){
    button_setPoint();
  }
}

void move45(){
  motor1.run(FORWARD);
  motor2.run(RELEASE);
  motor3.run(BACKWARD);
  motor4.run(RELEASE);
  button_setPoint()
  }

void move135(){
  motor1.run(RELEASE);
  motor2.run(FORWARD);
  motor3.run(RELEASE);
  motor4.run(BACKWARD);
  button_setPoint()
  }

void move225(){
  motor1.run(BACKWARD);
  motor2.run(RELEASE);
  motor3.run(FORWARD);
  motor4.run(RELEASE);
  button_setPoint()
  }

void move315(){
  motor1.run(RELEASE);
  motor2.run(BACKWARD);
  motor3.run(RELEASE);
  motor4.run(FORWARD);
  button_setPoint()
  }

void clockwise(){
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  button_setPoint()
  }

void counterclockwise(){
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  button_setPoint()
  }
  
void stopped(){
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  }

///////////// initialize encoders /////////////

void EncoderInit_1()
{
  Direction_1 = true;//default -> Forward  
  pinMode(encoder_pinB_1,INPUT);  
  attachInterrupt(5, wheelSpeed_1, CHANGE);
}
void EncoderInit_2()
{
  Direction_2 = true;//default -> Forward  
  pinMode(encoder_pinB_2,INPUT);  
  attachInterrupt(4, wheelSpeed_2, CHANGE);
}
void EncoderInit_3()
{
  Direction_3 = true;//default -> Forward  
  pinMode(encoder_pinB_3,INPUT);  
  attachInterrupt(3, wheelSpeed_3, CHANGE);
}
void EncoderInit_4()
{
  Direction_4 = true;//default -> Forward  
  pinMode(encoder_pinB_4,INPUT);  
  attachInterrupt(2, wheelSpeed_4, CHANGE);
}

///////////// calculate Speed of motors /////////////

//motor 1
void wheelSpeed_1()
{
  int Lstate_1 = digitalRead(encoder_pinA_1);
  if((encoder_PinALast_1 == LOW) && Lstate_1==HIGH)
  {
    int val_1 = digitalRead(encoder_pinB_1);
    if(val_1 == LOW && Direction_1) //define direction of the motor whether forward or reverse
    {
      Direction_1 = false; //Reverse
    }
    else if(val_1 == HIGH && !Direction_1)
    {
      Direction_1 = true;  //Forward
    }
  }
  encoder_PinALast_1 = Lstate_1;
 
  if(!Direction_1)  pulse_1++;
  else  pulse_1--;
};

//motor 2
void wheelSpeed_2()
{
  int Lstate_2 = digitalRead(encoder_pinA_2);
  if((encoder_PinALast_2 == LOW) && Lstate_2==HIGH)
  {
    int val_2 = digitalRead(encoder_pinB_2);
    if(val_2 == LOW && Direction_2) //define direction of the motor whether forward or reverse
    {
      Direction_2 = false; //Reverse
    }
    else if(val_2 == HIGH && !Direction_2)
    {
      Direction_2 = true;  //Forward
    }
  }
  encoder_PinALast_2 = Lstate_2;
 
  if(!Direction_2)  pulse_2++;
  else  pulse_2--;
};

//motor 3
void wheelSpeed_3()
{
  int Lstate_3 = digitalRead(encoder_pinA_3);
  if((encoder_PinALast_3 == LOW) && Lstate_3==HIGH)
  {
    int val_3 = digitalRead(encoder_pinB_3);
    if(val_3 == LOW && Direction_3) //define direction of the motor whether forward or reverse
    {
      Direction_3 = false; //Reverse
    }
    else if(val_3 == HIGH && !Direction_3)
    {
      Direction_3 = true;  //Forward
    }
  }
  encoder_PinALast_3 = Lstate_3;
 
  if(!Direction_3)  pulse_3++;
  else  pulse_3--;
};

//motor 4
void wheelSpeed_4()
{
  int Lstate_4 = digitalRead(encoder_pinA_4);
  if((encoder_PinALast_4 == LOW) && Lstate_4==HIGH)
  {
    int val_4 = digitalRead(encoder_pinB_4);
    if(val_4 == LOW && Direction_4) //define direction of the motor whether forward or reverse
    {
      Direction_4 = false; //Reverse
    }
    else if(val_4 == HIGH && !Direction_4)
    {
      Direction_4 = true;  //Forward
    }
  }
  encoder_PinALast_4 = Lstate_4;
 
  if(!Direction_4)  pulse_4++;
  else  pulse_4--;
};

//function to calculate max speed of the motor when moving the joystick from center to the egde
double get_speed(){
  if ((abs(get_x)+abs(get_z)) > 5){
     max_speed = 255;
  }
    else{
      max_speed = (abs(get_x)+abs(get_z))*50;
    }
    return max_speed;
}

//function to calculate setpoint speed for PID to move the Omni robot with specific degrees
void joystick_setPoint()    
 {
  inputDegree = atan(double (get_z/get_x))*(180/PI);
   
  double Degree1,Degree2,Degree3,Degree4;
  Degree1 = cos((45 + inputDegree)*(PI/180));
  Degree2 = cos((45 - inputDegree)*(PI/180));
  
  Speed1 = -get_speed()*Degree1;
  Speed2 = -(get_speed())*Degree2;
  Speed3 = (get_speed())*Degree1;
  Speed4 = get_speed()*Degree2;
  
  //Set the output value of the PID when using joystick
  Setpoint1 = abs(Speed1);
  Setpoint2 = abs(Speed2);
  Setpoint3 = abs(Speed3);
  Setpoint4 = abs(Speed4);
 }
 
 //Set the output value of the PID when using button
void button_setPoint(){
   Setpoint1 = get_ipspeed;
   Setpoint2 = get_ipspeed;
   Setpoint3 = get_ipspeed;
   Setpoint4 = get_ipspeed;
}


