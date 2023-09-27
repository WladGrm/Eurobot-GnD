#if (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

//Dynamyxel lib prepare
#include "AX12A.h"
#define DirectionPin (10u)
#define BaudRate (1000000ul)
#define ID (1u)
int initial_pos = 512;
int maxt = 612;
int mint = 712;

AX12A dyn1;
AX12A dyn2;
AX12A dyn3;


#include <Servo.h>

#include "ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt16MultiArray.h"
#include "ros/time.h"
//header file for publishing velocities for odom
#include "lino_msgs/Velocities.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
//header file for pid server
#include "lino_msgs/PID.h"
//header file for imu
#include "lino_msgs/Imu.h"

#include "lino_base_config.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"
#include "Imu.h"

#define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
#include "Encoder.h"

#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20 //hz
#define BUTTON_RATE 5 //hz
#define DEBUG_RATE 5

//Emergency_button
const int e_pin = 31;
bool last_reading_e;
unsigned long last_debounce_time_e = 0;
bool published_e = true;

//Start_button
const int start_pin = 32;
bool last_reading;
unsigned long last_debounce_time = 0;
unsigned long debounce_delay = 50;
bool published = true;

//Cake grippers
Servo mg1;
Servo mg2;
Servo mg3;
Servo mg4;
Servo mg5;
Servo mg6;


Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV); 
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV); 
//Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV); 



Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B, EN1);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B, EN2); 
Controller motor3_controller(Controller::MOTOR_DRIVER, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B, EN3);


PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;


//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const lino_msgs::PID& pid);
void servoCallback(const std_msgs::UInt16MultiArray& servo_msg);

ros::NodeHandle nh;

//Subscribing to a servo topic
ros::Subscriber<std_msgs::UInt16MultiArray> servo_sub("servo", &servoCallback);

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);

lino_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

//Topic for emergency_button
std_msgs::Bool emergency_msg;
ros::Publisher emergency_pub("emergency", &emergency_msg);

//Topic for start_button
std_msgs::Bool start_msg;
ros::Publisher start_pub("start", &start_msg);


void setup()
{
//Initialize Dynamixels and set initial pose
dyn1.begin(BaudRate, DirectionPin, &Serial1);
dyn2.begin(BaudRate, DirectionPin, &Serial5);
dyn3.begin(BaudRate, DirectionPin, &Serial7);

dyn1.move(ID, initial_pos);
dyn2.move(ID, initial_pos);
dyn3.move(ID, initial_pos);
//dyn3.move(ID, initial_pos+100+100);
//Initialize gripper pins and open them
    //45 - opened; 155 - closed
    mg1.attach(2);
    mg1.write(45);
    //160 - opened; 40 - closed
    mg2.attach(3);
    mg2.write(160);
    //10 - opened; 140 - closed
    mg3.attach(4);
    mg3.write(10);
    //140 - opened; 20 closed
    mg4.attach(24);
    mg4.write(140);
    //15 - opened; 130 - closed 
    mg5.attach(25);
    mg5.write(15);
     //170 - opened; 65 closed
    mg6.attach(28);
    mg6.write(170);
    
     
    
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    //subscribe for servo topic
    nh.subscribe(servo_sub);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);
    //For emergency_button
    nh.advertise(emergency_pub);
    //For start_button
    nh.advertise(start_pub);
    
    //Emergency button pinmode
    pinMode(e_pin, INPUT);
    last_reading_e = !digitalRead(e_pin);
    
    //Start button pinmode
    pinMode(start_pin, INPUT);
    last_reading = !digitalRead(start_pin);   

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("LINOBASE CONNECTED");
    delay(1);
}

void loop()
{   
    static unsigned long prev_control_time = 0;
    static unsigned long prev_imu_time = 0;
    static unsigned long prev_debug_time = 0;
    //static unsigned long prev_servo_time = 0;
    static unsigned long prev_button_time = 0;

    
    static bool imu_is_initialized;

//Серво колбэк
//if ((millis() - prev_servo_time) >= (1000 / BUTTON_RATE))
//{
//   servoCallback(const std_msgs::UInt16MultiArray& servo_msg);
//  prev_servo_time = millis();
//}

    //Кнопка
 if ((millis() - prev_button_time) >= (1000 / BUTTON_RATE))
 {
    startCallback();
    emergencyCallback();
    prev_button_time = millis();
 }

    //this block drives the robot based on defined rate
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        moveBase();
        prev_control_time = millis();
    }

    //this block stops the motor when no command is received
    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
    }

    //this block publishes the IMU data based on defined rate
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        //sanity check if the IMU is connected
        if (!imu_is_initialized)
        {
            imu_is_initialized = initIMU();

            if(imu_is_initialized)
                nh.loginfo("IMU Initialized");
            else
                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
        }
        else
        {
            publishIMU();
        }
        prev_imu_time = millis();
    }

    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(DEBUG)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
            //printDebug();
            prev_debug_time = millis();
        }
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
    delay(25);
}

//Callback for servos
void servoCallback(const std_msgs::UInt16MultiArray& servo_msg)
{
    dyn1.move(ID, servo_msg.data[6]);
    dyn2.move(ID, servo_msg.data[7]);
    dyn3.move(ID, servo_msg.data[8]);

    mg1.write(servo_msg.data[0]);
    mg2.write(servo_msg.data[1]);
    mg3.write(servo_msg.data[2]);
    mg4.write(servo_msg.data[3]);
    mg5.write(servo_msg.data[4]);
    mg6.write(servo_msg.data[5]);
}



//For emergency button state publication
void emergencyCallback()
{
 bool reading_e = ! digitalRead(e_pin);
    if (last_reading_e!= reading_e){
        last_debounce_time_e = millis();
        published_e = false;
    }
    
    if (!published_e && (millis() - last_debounce_time_e) > debounce_delay) {
    emergency_msg.data = reading_e;
    emergency_pub.publish(&emergency_msg);
    published_e = true;
    }
    last_reading_e = reading_e;
}

//For start buttom publication
void startCallback()
{ 
    bool reading = ! digitalRead(start_pin);
    if (last_reading!= reading){
        last_debounce_time = millis();
        published = false;
    }
    
    if (!published && (millis() - last_debounce_time) > debounce_delay) {
    start_msg.data = reading;
    start_pub.publish(&start_msg);
    published = true;
    }
    last_reading = reading;
}

void PIDCallback(const lino_msgs::PID& pid)
{
    //callback function every time PID constants are received from lino_pid for tuning
    //this callback receives pid object where P,I, and D constants are stored
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
    motor3_pid.updateConstants(pid.p, pid.i, pid.d);
    //motor4_pid.updateConstants(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    g_prev_command_time = millis();
}

void moveBase()
{
        //char buffer[50];
        
        
    //get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
    
       // sprintf (buffer, "Motor1 rpm : %ld", req_rpm.motor1);
       // nh.loginfo(buffer);
       // sprintf (buffer, "Motor2 rpm  : %ld", req_rpm.motor2);
       // nh.loginfo(buffer);
       // sprintf (buffer, "Motor3 rpm  : %ld", req_rpm.motor3);
       // nh.loginfo(buffer);
        
    //get the current speed of each motor
    int current_rpm1 = motor1_encoder.getRPM();
    int current_rpm2 = motor2_encoder.getRPM();
    int current_rpm3 = motor3_encoder.getRPM();

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    
    //motor1_controller.spin(-100);
    //motor2_controller.spin(-100);
    //motor3_controller.spin(-100);
    
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
    
    //motor3_controller.spin(motor1_pid.compute(req_rpm.motor3, current_rpm3));  
        //nh.loginfo(buffer);   

    Kinematics::velocities current_vel;

    if(kinematics.base_platform == Kinematics::ACKERMANN || kinematics.base_platform == Kinematics::ACKERMANN1)
    {
        float current_steering_angle;
        
       // current_steering_angle = steer(g_req_angular_vel_z);
        //current_vel = kinematics.getVelocities(current_steering_angle, current_rpm1, current_rpm2);
    }
    else
    {
        current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3);
    }
    
    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void publishIMU()
{
    //pass accelerometer data to imu object
    raw_imu_msg.linear_acceleration = readAccelerometer();

    //pass gyroscope data to imu object
    raw_imu_msg.angular_velocity = readGyroscope();

    //pass accelerometer data to imu object
    raw_imu_msg.magnetic_field = readMagnetometer();

    //publish raw_imu_msg
    raw_imu_pub.publish(&raw_imu_msg);
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printDebug()
{
    char buffer[50];
    printf("TEST MESSAGE");

   sprintf (buffer, "Encoder M1  : %ld", motor1_encoder.read());
   nh.loginfo(buffer);
   sprintf (buffer, "Encoder M2 : %ld", motor2_encoder.read());
   nh.loginfo(buffer);
   sprintf (buffer, "Encoder M3   : %ld", motor3_encoder.read());
   nh.loginfo(buffer);
}
