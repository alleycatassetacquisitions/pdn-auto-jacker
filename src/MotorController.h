#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <DynamixelShield.h>

class MotorController {
public:
    MotorController(HardwareSerial& debugSerial, HardwareSerial& dxlSerial);
    void setup();
    void loop();
    void processSerialInput();

private:
    // Serial references
    HardwareSerial& DEBUG_SERIAL;
    HardwareSerial& DXL_SERIAL;

    // Dynamixel
    Dynamixel2Arduino dxl;

    // Constants
    static constexpr uint8_t DXL_ID = 1;
    static constexpr float DXL_PROTOCOL_VERSION = 2.0;
    static constexpr float RACK_TRAVEL_MM = 21.0;
    static constexpr float PINION_CIRCUMFERENCE_MM = 82.896;
    static constexpr float MAX_ANGLE_DEGREES = (RACK_TRAVEL_MM / PINION_CIRCUMFERENCE_MM) * 360.0;
    static constexpr float MOVE_SPEED = 30;
    static constexpr float CALIBRATION_SPEED = 30;
    static constexpr float POSITION_THRESHOLD = 0.5;
    static constexpr long interval = 50;
    static constexpr long stallTimeout = 1000;
    static constexpr float JOG_INCREMENT = 5.0;

    // State variables
    float minPosition = 47.52;
    float maxPosition = 141.06;
    float currentPosition = 0.0;
    bool movingToMax = true;
    bool isCalibrated = false;
    bool hitEndstop = false;
    bool isMotorMoving = false;
    bool isJogMode = false;
    float targetPosition = 0.0;
    unsigned long previousMillis = 0;
    unsigned long stallCheckStart = 0;

    // Private methods
    void calibrateMotor();
    void findEndstop(bool searchingForMax);
    bool pingDynamixel();
    void testSimpleMovement();
    void setMotorMoving(bool moving);
    bool isMotorCurrentlyMoving();
};

#endif // MOTOR_CONTROLLER_H 