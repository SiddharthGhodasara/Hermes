/////// TEENSY CODE FOR HERMES ///////

#define MAX_RPM 66               // motor's maximum RPM
#define COUNTS_PER_REV 39524      // wheel encoder's no of ticks per rev 
#define WHEEL_DIAMETER 0.070      // wheel's diameter in meters
#define WHEEL_SEPARATION 0.312
#define PWM_MAX 255           // PWM Resolution of the microcontroller
#define PWM_MIN -255

#define K_P 0.5 // P constant
#define K_I 0.35 // I constant
#define K_D 0.3 // D constant
 
#define LEFT_MOTOR_ENCODER_A 2
#define LEFT_MOTOR_ENCODER_B 1 

#define RIGHT_MOTOR_ENCODER_A 8
#define RIGHT_MOTOR_ENCODER_B 7 
#define USE_MPU9250_IMU


#include "ros.h"
#include "ros/time.h"
#include "I2Cdev.h"
#include "geometry_msgs/Vector3.h"
//header file for publishing velocities for odom
#include "robot_controller/velocities.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"

//header file for imu
#include "robot_controller/imu.h"
#include "CytronMotorDriver.h"
#include "Kinematics.h"
#include "PID.h"
//#include "MPU9250.h"
#include "Imu.h"
/*#define G_TO_ACCEL 9.81
#define MGAUSS_TO_UTESLA 0.1
#define UTESLA_TO_TESLA 0.000001
 #define ACCEL_SCALE 1 / 16384 // LSB/g
    #define GYRO_SCALE 1 / 131 // LSB/(deg/s)
    #define MAG_SCALE 0.6 // uT/LSB
    
    MPU9250 accelerometer;
    MPU9250 gyroscope;    
    MPU9250 magnetometer;*/


//MPU9250 IMU(Wire,0x68);
//int status;

#define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
#include "Encoder.h"

#define IMU_PUBLISH_RATE 20 //hz
#define PUBLISH_RATE 30 //hz

CytronMD leftmotor(PWM_PWM, 3, 4);
CytronMD rightmotor(PWM_PWM, 5, 6);

Encoder left_motor_encoder(LEFT_MOTOR_ENCODER_A, LEFT_MOTOR_ENCODER_B, COUNTS_PER_REV);
Encoder right_motor_encoder(RIGHT_MOTOR_ENCODER_A, RIGHT_MOTOR_ENCODER_B, COUNTS_PER_REV);  

PID motor_left_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor_right_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(Kinematics::DIFFERENTIAL_DRIVE, MAX_RPM, WHEEL_DIAMETER, 0, WHEEL_SEPARATION);

float req_linear_vel_x = 0;
float req_linear_vel_y = 0;
float req_angular_vel_z = 0;

unsigned long prev_command_time = 0;

//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);

robot_controller::imu raw_imu_msg;
ros::Publisher raw_imu_pub("imu_raw", &raw_imu_msg);

robot_controller::velocities raw_vel_msg;
ros::Publisher raw_vel_pub("vel_raw", &raw_vel_msg);

void setup()
{    
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    //nh.advertise(raw_imu_pub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("HERMES CONNECTED");
    delay(1);
}

void loop()
{
    static unsigned long prev_control_time = 0;
    //static unsigned long prev_imu_time = 0;
    //static unsigned long prev_debug_time = 0;
    //static bool imu_is_initialized;

    //this block drives the robot based on defined rate
    if ((millis() - prev_control_time) >= (1000 / PUBLISH_RATE))
    {
        moveBase();
        prev_control_time = millis();
    }

    //this block stops the motor when no command is received
    if ((millis() - prev_command_time) >= 400)
    {
        stopBase();
    }

    //this block publishes the IMU data based on defined rate
   /* if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
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
    }*/

    //call all the callbacks waiting to be called
    nh.spinOnce();
}


void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    req_linear_vel_x = cmd_msg.linear.x;
    req_linear_vel_y = cmd_msg.linear.y;
    req_angular_vel_z = cmd_msg.angular.z;

    prev_command_time = millis();
}

void moveBase()
{
    //get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(req_linear_vel_x, req_linear_vel_y, req_angular_vel_z);

    //get the current speed of each motor
    int current_rpm_left = left_motor_encoder.getRPM();
    int current_rpm_right = right_motor_encoder.getRPM();

    leftmotor.setSpeed(motor_left_pid.compute(req_rpm.motor1, current_rpm_left));
    rightmotor.setSpeed(motor_right_pid.compute(req_rpm.motor2, current_rpm_right));   

    Kinematics::velocities current_vel;
 
    current_vel = kinematics.getVelocities(current_rpm_left, current_rpm_right, 0, 0);
    
    //pass velocities to object to be published
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
    req_linear_vel_x = 0;
    req_linear_vel_y = 0;
    req_angular_vel_z = 0;
}

/*bool initIMU()
{
    Wire.begin();
   bool ret;
    status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);  }
    accelerometer.initialize();
    ret = accelerometer.testConnection();
    if(!ret)
        return false;

    gyroscope.initialize();
    ret = gyroscope.testConnection();
    if(!ret)
        return false;
  
    magnetometer.initialize();
    ret = magnetometer.testConnection();
    if(!ret)
        return false;

    return true;
}

bool initIMU()
{
    Wire.begin();
    bool ret;
    
    accelerometer.initialize();
    ret = accelerometer.testConnection();
    if(!ret)
        return false;

    gyroscope.initialize();
    ret = gyroscope.testConnection();
    if(!ret)
        return false;
  
    magnetometer.initialize();
    ret = magnetometer.testConnection();
    if(!ret)
        return false;

    return true;
}

geometry_msgs::Vector3 readAccelerometer()
{
    geometry_msgs::Vector3 accel;
    int16_t ax, ay, az;
    
    accelerometer.getAcceleration(&ax, &ay, &az);

    accel.x = ax * (double) ACCEL_SCALE * G_TO_ACCEL;
    accel.y = ay * (double) ACCEL_SCALE * G_TO_ACCEL;
    accel.z = az * (double) ACCEL_SCALE * G_TO_ACCEL;

    return accel;
}

geometry_msgs::Vector3 readGyroscope()
{
    geometry_msgs::Vector3 gyro;
    int16_t gx, gy, gz;

    gyroscope.getRotation(&gx, &gy, &gz);

    gyro.x = gx * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.y = gy * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.z = gz * (double) GYRO_SCALE * DEG_TO_RAD;

    return gyro;
}

geometry_msgs::Vector3 readMagnetometer()
{
    geometry_msgs::Vector3 mag;
    int16_t mx, my, mz;

    magnetometer.getHeading(&mx, &my, &mz);

    mag.x = mx * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.y = my * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.z = mz * (double) MAG_SCALE * UTESLA_TO_TESLA;

    return mag;
}

void publishIMU()
{
    //pass accelerometer data to imu object
    raw_imu_msg.linear_acceleration = readAccelerometer();

    //pass gyroscope data to imu object
    raw_imu_msg.angular_velocity = readGyroscope();

    //pass accelerometer data to imu object
    raw_imu_msg.magnetic_field = readMagnetometer();*/

   /* raw_imu_msg.angular_velocity.x =IMU.getGyroX_rads()* 180/M_PI  ;
         raw_imu_msg.angular_velocity.y =IMU.getGyroY_rads()* 180/M_PI ; 
         raw_imu_msg.angular_velocity.z =IMU.getGyroZ_rads()* 180/M_PI; 
         raw_imu_msg.linear_acceleration.x =IMU.getAccelX_mss() ;
         raw_imu_msg.linear_acceleration.y =IMU.getAccelY_mss() ; 
         raw_imu_msg.linear_acceleration.z =IMU.getAccelZ_mss();*/
         //data.magnetic_field.x=IMU.getMagX_uT();
         //data.magnetic_field.y=IMU.getMagY_uT();
         //data.magnetic_field.z=IMU.getMagZ_uT();
         //publish raw_imu_msg
         //raw_imu_pub.publish(&raw_imu_msg);
//}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
