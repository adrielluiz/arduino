#include <AFMotor.h>

AF_DCMotor motor_right(3); //Select motor 3
AF_DCMotor motor_left(4); //Select motor 1

float Kp=55,Ki=70,Kd=0;
float Kbgain = 0; //Gain to keep engines balanced
float error=0, P=0, I=0, D=0, PID_value=0; //PID constants
float previous_error=0, previous_I=0;
int sensor[5]={0, 0, 0, 0, 0};
int initial_motor_speed=200, velocmax = 255, velocmin = 0;
int flag_final = 0;
int time_min =0; //Minimal time to stop

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

void setup()
{
 pinMode(9,OUTPUT);     //PWM Pin 1
 pinMode(10,OUTPUT);    //PWM Pin 2
 pinMode(4,OUTPUT);     //Left Motor Pin 1
 pinMode(5,OUTPUT);     //Left Motor Pin 2
 pinMode(6,OUTPUT);     //Right Motor Pin 1
 pinMode(7,OUTPUT);     //Right Motor Pin 2

 // Line sensors
 pinMode(A0,INPUT);  
 pinMode(A1,INPUT);  
 pinMode(A2,INPUT);  
 pinMode(A3,INPUT);  
 pinMode(A4,INPUT);  

 Serial.begin(9600); //Enable Serial Communications
 delay(3000);
}

void loop()
{
    read_sensor_values();
    calculate_pid();
    motor_control();
    time_min = millis();
}

void read_sensor_values()
{
  sensor[0]=digitalRead(A0);
  sensor[1]=digitalRead(A1);
  sensor[2]=digitalRead(A2);
  sensor[3]=digitalRead(A3);
  sensor[4]=digitalRead(A4);
  
   if(((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0)) || ((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)) || ((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)))
  error=-4;  
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))//detect sensors 3 e 4
  error=-3;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==1))//detect only sensor 3
  error=-2;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1))//detect sensors 2 e 3
  error=-1;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))//detect only central sensor 
  error=0;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))//detect sensors 1 e 2
  error=1;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))//detect only sensor 1
  error=2;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))//detect sensors 0 e 1
  error=3;
  else if(((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)) || ((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))|| ((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1)))
  error=4;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))//no detect 
  {
    if(error < 0)
        error=-4;
    else if(error > 0)
        error=4;
    else
        error = 0;
  }    
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0) && (time_min > 5000))//parar
  {
    flag_final++;
    time_min = 0;
  }

        
}

void calculate_pid()
{
    P = error;
    I = I + previous_I;
    D = error-previous_error;
    
    PID_value = (Kp*P) + (Ki*I) + (Kd*D);
    
    previous_I=I;
    previous_error=error;
}

void motor_control()
{
    // Calculating the effective motor speed:
    int left_motor_speed = initial_motor_speed+PID_value;
    int right_motor_speed = initial_motor_speed-PID_value;
    
    // The motor speed should not exceed the max PWM value
    constrain(left_motor_speed+Kbgain,0,110);
    constrain(right_motor_speed,0,110);  
      
    if (left_motor_speed < velocmin)
      left_motor_speed = velocmin;
      
    if (right_motor_speed < velocmin)
      right_motor_speed = velocmin; 
    
    if (left_motor_speed > velocmax)
      left_motor_speed = velocmax;
      
    if (right_motor_speed > velocmax)
      right_motor_speed = velocmax;
  
        
    motor_left.setSpeed(left_motor_speed);
    motor_right.setSpeed(right_motor_speed);
   
   
    motor_left.run(FORWARD);
    motor_right.run(FORWARD); 
}
