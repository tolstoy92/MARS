#include <Arduino.h>
#include <Wire.h>
#include <AX12A.h>
#include "math.h"
#include <I2Cdev.h>
#include <MotorControl.h>
#include <MqttClient.h>
#include <PubSubClient.h>
#include <stdio.h>

#define Kp  25  /*Too small Kp will cause the robot to fall, because the fix is ​​not enough.
Too much Kp forces the robot to go wildly forward and backward.
A good Kp will make the robot move very little back and forth (or slightly oscillates).*/
#define Kd  0.02 /*A good Kd value will reduce the vibrations until the robot becomes almost steady.
In addition, the correct Kd will hold the robot, even if it is pushed. */
#define Ki  5  /*The correct Ki value will shorten the time required to stabilize the robot.*/
#define sample_time  0.0005
#define target_angle -1.5
#define BETA 0.22f

// hw_timer_t *timer = NULL;
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile int interruptCounter;
int totalInterruptCounter = 0;
int encoderRightCnt = 0;
int encoderLeftCnt = 0;

char outputData[10] = "HI";

double duration;

float minSpeed = 0;
float maxSpeed = 1;
float minControlSignal = 100;
float maxControlSignal = 200;
float maxError = 0.5;
float rightSpeed = 0;
float leftSpeed = 0;
float controlSignal;

float kp = 10;
float kd = 0.2;
float ki = 0.3;

float errorRightWheel;
float errorLeftWheel;
float integral;
float prevError;
float minIntegral = -30;
float maxIntegral = 30;
float targetSpeed = 0.5;
float reduceSpeed = 5;
float reduceSpeedSide = 5;
float yRight;
float yLeft;
float uRight;
float uLeft;

std::string receivedData;
std::string sign;
std::string angle;
std::string move;
std::string rotate;
std::string finish;
std::string valuex;
std::string valuey;

short speed = 100;
short x, y, x1, x2;
short frequency = 500;
short xCoord;
short yCoord;
short correctValue = 0;
short errorValue;
short hallSensorLeft = 0;
short hallSensorRight = 0;

uint8_t finishValue = -1;
uint8_t resolution = 10;
uint8_t splitindex;
uint8_t channel_R = 0;
uint8_t channel_L = 1;
uint8_t rotateValue = -1;
uint8_t moveForwardValue = -1;


uint8_t platformNumber = 2;

// const char* ssid = "SPEECH_405";
// const char* password = "multimodal";
// const char* mqtt_server = "192.168.0.105";

// const char* ssid = "mqtt_router";
// const char* password = "multimodal";
// const char* mqtt_server = "192.168.0.18";

const char* ssid = "iGarage";
const char* password = "igarage18";
const char* mqtt_server = "172.16.30.45";

// const char* ssid = "Redmi";
// const char* password = "qweqweqw";
// const char* mqtt_server = "192.168.43.229";

mqttClient mqtt(ssid, password, mqtt_server);
MotorControl GyroRobot;
// void IRAM_ATTR onTimer()
// {

//     portENTER_CRITICAL_ISR(&timerMux);
//     interruptCounter++;
//     portEXIT_CRITICAL_ISR(&timerMux);

// }

// void init_Timer()
// {
//     timer = timerBegin(0, 80, true);
//     timerAttachInterrupt(timer, &onTimer, true);
//     timerAlarmWrite(timer, 5000, true);
//     timerAlarmEnable(timer);
// }

void callback(char* topic, byte* message, unsigned int length)
{
    char platformControlTopic[64];

    sprintf(platformControlTopic, "platforms/%d", platformNumber);

    if (strcmp(topic, platformControlTopic)==0) {

        receivedData = "";
        sign = "";
        angle = "";
        move = "";
        rotate = "";
        finish = "";

        int digit_sign;

        for (int i = 0; i < length; i++)
            {
                receivedData += (char)message[i];
            }
            sign = receivedData[0];
            angle = receivedData.substr(1, 3);
            move = receivedData[4];
            rotate = receivedData[5];
            finish = receivedData[6];

            if (sign == "0") {
                digit_sign = -1;
            }
            else {
                digit_sign = 1;
            }

            correctValue = digit_sign * atoi(angle.c_str());
            moveForwardValue = atoi(move.c_str());
            rotateValue = atoi(rotate.c_str());
            finishValue = atoi(finish.c_str());
        }
}

void setup()
{

    GyroRobot = MotorControl();
    GyroRobot.setupMotor();
    mqtt.setupWifi();
    mqtt.setCallback(*callback);
    // init_Timer();
    mqtt.subscribe(platformNumber);

}

void loop()
{
    //GyroRobot.enableCentralMotor();

    //GyroRobot.goForward(200);
    mqtt.initClientLoop();
    if (moveForwardValue == 1 && rotateValue == 0) {
        GyroRobot.goForward(speed);
    }
    if (rotateValue == 0 && moveForwardValue == 0) {
        GyroRobot.stopMovement();
    }
    if (rotateValue == 1 && moveForwardValue == 0)
    {
       GyroRobot.stopMovement();
        if (correctValue > 0) {
        GyroRobot.turnLeft(speed);
        }
        else if (correctValue < 0) {
        GyroRobot.turnRight(speed);
        }
    }
    else if (finishValue == 1) {
    GyroRobot.stopMovement();
    }
    // //mqtt.pubFeedback(outputData,platformNumber);
}
