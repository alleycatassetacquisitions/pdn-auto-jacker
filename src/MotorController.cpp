#include "MotorController.h"
#include <Arduino.h>

using namespace ControlTableItem;

MotorController::MotorController(HardwareSerial& debugSerial, HardwareSerial& dxlSerial)
    : DEBUG_SERIAL(debugSerial), DXL_SERIAL(dxlSerial), dxl(DXL_SERIAL, DXL_DIR_PIN) {
    // Initialize other members if needed
}

void MotorController::setup() {
    // Start serial communication and wait for connection
    DEBUG_SERIAL.begin(115200);

    // Wait for serial port to connect
    unsigned long startTime = millis();
    while(!DEBUG_SERIAL && (millis() - startTime < 5000));

    DEBUG_SERIAL.println("\n\n=== DYNAMIXEL TEST PROGRAM STARTING ===");
    DEBUG_SERIAL.println("Testing communication with motor...");

    // Initialize Dynamixel communication
    dxl.begin(57600);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    
    // Test if the motor responds to ping
    if (!pingDynamixel()) {
        DEBUG_SERIAL.println("MOTOR NOT DETECTED - CHECK CONNECTIONS AND POWER");
        DEBUG_SERIAL.println("1. Is the motor powered? (USB power is not enough)");
        DEBUG_SERIAL.println("2. Is the motor properly connected to the shield?");
        DEBUG_SERIAL.println("3. Is the shield properly connected to the Arduino?");
        DEBUG_SERIAL.println("4. Is the motor ID set to 1?");
        
        // Enter a simple diagnostic loop
        while(1) {
            if (pingDynamixel()) {
                DEBUG_SERIAL.println("MOTOR DETECTED! Restarting program...");
                delay(1000);
                break;
            }
            delay(5000); // Check every 5 seconds
        }
    }
    
    DEBUG_SERIAL.println("Motor detected! Setting up motor parameters...");
    
    // Configure motor
    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_POSITION);
    
    // Check motor model information
    DEBUG_SERIAL.print("Motor Model Number: ");
    DEBUG_SERIAL.println(dxl.readControlTableItem(MODEL_NUMBER, DXL_ID));
    
    // Set conservative velocity profile during calibration
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, CALIBRATION_SPEED);
    
    // Set higher torque limit (75% of max)
    dxl.writeControlTableItem(GOAL_CURRENT, DXL_ID, 750);  // Assuming 1000 is max
    // Turn torque on to start controlling the motor
    dxl.torqueOn(DXL_ID);

    // Read initial position
    currentPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    
    DEBUG_SERIAL.println("Starting basic motor test...");
    testSimpleMovement();
    
    DEBUG_SERIAL.println("Skipping automatic calibration.");
    DEBUG_SERIAL.println("Entering manual tuning mode...");
    DEBUG_SERIAL.println("Use '+' to move in positive direction");
    DEBUG_SERIAL.println("Use '-' to move in negative direction");
    DEBUG_SERIAL.println("Use 'm' to mark minimum position");
    DEBUG_SERIAL.println("Use 'M' to mark maximum position");
    DEBUG_SERIAL.println("Use 'c' to toggle continuous movement mode");
    DEBUG_SERIAL.println("Use 's' to save current limits and start normal operation");
    
    // Comment out calibration call and directly set as calibrated
    // calibrateMotor();
    isCalibrated = true;
    isJogMode = true;
    
    // Set to operational speed but lower for fine tuning
    // dxl.torqueOff(DXL_ID);
    // dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, CALIBRATION_SPEED);
    // dxl.torqueOn(DXL_ID);
}

void MotorController::loop() {
    // Check for serial input regardless of mode
    processSerialInput();
    
    // Handle motor stop when in jog mode
    if (isJogMode) {
        // Check if motor has reached its target and stop LED
        currentPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
        // Log the current position in jog mode
        if (millis() % 1000 == 0) {
            DEBUG_SERIAL.print("Current position in jog mode: ");
            DEBUG_SERIAL.println(currentPosition);
        }
        
        // Check motor movement status using control table register
        bool motorCurrentlyMoving = isMotorCurrentlyMoving();
        
        // If motor was moving but is now stopped, update the LED
        if (isMotorMoving && !motorCurrentlyMoving) {
            setMotorMoving(false);
            DEBUG_SERIAL.print("Position reached: ");
            DEBUG_SERIAL.println(currentPosition);
        }
        
        return; // Exit early in jog mode
    }
    
    // Only run continuous movement routine if calibration is complete
    if (!isCalibrated) return;
    
    unsigned long currentMillis = millis();
    
    // Check position and update direction periodically
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        
        // Read current position
        currentPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
        
        // Check if we're at the limits and need to change direction
        if (movingToMax && (currentPosition >= maxPosition - POSITION_THRESHOLD)) {
            // Reached max position, change direction
            movingToMax = false;
            DEBUG_SERIAL.println("Reached maximum position, moving toward minimum");
            targetPosition = minPosition;
            setMotorMoving(true);
            dxl.setGoalPosition(DXL_ID, targetPosition, UNIT_DEGREE);
        } 
        else if (!movingToMax && (currentPosition <= minPosition + POSITION_THRESHOLD)) {
            // Reached min position, change direction
            movingToMax = true;
            DEBUG_SERIAL.println("Reached minimum position, moving toward maximum");
            targetPosition = maxPosition;
            setMotorMoving(true);
            dxl.setGoalPosition(DXL_ID, targetPosition, UNIT_DEGREE);
        }
        
        // Check if motor has stopped moving using control table register
        bool motorCurrentlyMoving = isMotorCurrentlyMoving();
        
        // If motor was moving but is now stopped, update the LED
        if (isMotorMoving && !motorCurrentlyMoving) {
            setMotorMoving(false);
        }
        
        // Display current position every second
        if (currentMillis % 1000 < interval) {
            DEBUG_SERIAL.print("Current position: ");
            DEBUG_SERIAL.print(currentPosition);
            DEBUG_SERIAL.print(" | Target: ");
            DEBUG_SERIAL.print(movingToMax ? maxPosition : minPosition);
            DEBUG_SERIAL.print(" | Moving: ");
            DEBUG_SERIAL.println(motorCurrentlyMoving ? "Yes" : "No");
        }
    }
}

void MotorController::processSerialInput() {
    if (DEBUG_SERIAL.available() > 0) {
        char input = DEBUG_SERIAL.read();
        
        switch (input) {
            case '-':
                // Move in positive direction 
                currentPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
                DEBUG_SERIAL.print("Current position: ");
                DEBUG_SERIAL.println(currentPosition);
                
                targetPosition = currentPosition + JOG_INCREMENT;
                DEBUG_SERIAL.print("Jog increment: ");
                DEBUG_SERIAL.println(JOG_INCREMENT);
                
                setMotorMoving(true);
                DEBUG_SERIAL.println("Motor movement started");
                
                dxl.setGoalPosition(DXL_ID, targetPosition, UNIT_DEGREE);
                DEBUG_SERIAL.print("Moving to: ");
                DEBUG_SERIAL.println(targetPosition);
                
                DEBUG_SERIAL.println("Positive movement command sent");
                break;
                
            case '+':
                // Move in negative direction
                currentPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
                DEBUG_SERIAL.print("Current position: ");
                DEBUG_SERIAL.println(currentPosition);
                
                targetPosition = currentPosition - JOG_INCREMENT;
                DEBUG_SERIAL.print("Jog increment: ");
                DEBUG_SERIAL.println(JOG_INCREMENT);
                
                setMotorMoving(true);
                DEBUG_SERIAL.println("Motor movement started");
                
                dxl.setGoalPosition(DXL_ID, targetPosition, UNIT_DEGREE);
                DEBUG_SERIAL.print("Moving to: ");
                DEBUG_SERIAL.println(targetPosition);
                
                DEBUG_SERIAL.println("Negative movement command sent");
                break;
                
            case 'm':
                // Mark minimum position
                currentPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
                minPosition = currentPosition;
                DEBUG_SERIAL.print("Minimum position set to: ");
                DEBUG_SERIAL.println(minPosition);
                break;
                
            case 'M':
                // Mark maximum position
                currentPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
                maxPosition = currentPosition;
                DEBUG_SERIAL.print("Maximum position set to: ");
                DEBUG_SERIAL.println(maxPosition);
                break;
                
            case 'c':
                // Toggle continuous movement mode
                isJogMode = !isJogMode;
                if (isJogMode) {
                    DEBUG_SERIAL.println("Entering manual jog mode");
                    setMotorMoving(false);
                } else {
                    DEBUG_SERIAL.println("Entering continuous movement mode");
                    DEBUG_SERIAL.print("Min: ");
                    DEBUG_SERIAL.print(minPosition);
                    DEBUG_SERIAL.print(", Max: ");
                    DEBUG_SERIAL.println(maxPosition);
                    movingToMax = true;
                    targetPosition = maxPosition;
                    setMotorMoving(true);
                    dxl.setGoalPosition(DXL_ID, targetPosition, UNIT_DEGREE);
                }
                break;
                
            case 's':
                // Save current limits and start normal operation
                isJogMode = false;
                DEBUG_SERIAL.println("Saving current limits and starting normal operation");
                DEBUG_SERIAL.print("Min: ");
                DEBUG_SERIAL.print(minPosition);
                DEBUG_SERIAL.print(", Max: ");
                DEBUG_SERIAL.println(maxPosition);
                
                // Set to operational speed
                dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, MOVE_SPEED);
                
                // Start moving to max
                movingToMax = true;
                targetPosition = maxPosition;
                setMotorMoving(true);
                dxl.setGoalPosition(DXL_ID, targetPosition, UNIT_DEGREE);
                break;
                
            default:
                // Ignore other characters
                break;
        }
    }
}

// Private methods (stubs)
void MotorController::calibrateMotor() {
    DEBUG_SERIAL.println("Beginning calibration...");
    
    // First find the minimum position by moving in the negative direction
    findEndstop(false); // Move toward minimum position
    delay(500);         // Short pause
    
    // Set the discovered minimum position
    minPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    DEBUG_SERIAL.print("Minimum position found at: ");
    DEBUG_SERIAL.println(minPosition);
    
    // Now find the maximum position by moving in the positive direction
    findEndstop(true);  // Move toward maximum position
    delay(500);         // Short pause
    
    // Set the discovered maximum position
    maxPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    DEBUG_SERIAL.print("Maximum position found at: ");
    DEBUG_SERIAL.println(maxPosition);
    
    // Calculate the actual travel range
    float travelRange = maxPosition - minPosition;
    DEBUG_SERIAL.print("Measured travel range (degrees): ");
    DEBUG_SERIAL.println(travelRange);
    DEBUG_SERIAL.print("Estimated travel distance (mm): ");
    DEBUG_SERIAL.println((travelRange / 360.0) * PINION_CIRCUMFERENCE_MM);
    
    // Set to operational speed
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, MOVE_SPEED);
    
    // Move to the middle position to start normal operation
    float middlePosition = minPosition + (travelRange / 2.0);
    setMotorMoving(true);
    dxl.setGoalPosition(DXL_ID, middlePosition, UNIT_DEGREE);
    
    // Wait until we reach the middle position
    while(abs(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - middlePosition) > POSITION_THRESHOLD) {
        delay(10);
    }
    setMotorMoving(false);
    
    isCalibrated = true;
    DEBUG_SERIAL.println("Calibration complete. Starting normal operation.");
}

void MotorController::findEndstop(bool searchingForMax) {
    DEBUG_SERIAL.print("Searching for ");
    DEBUG_SERIAL.print(searchingForMax ? "maximum" : "minimum");
    DEBUG_SERIAL.println(" endstop...");
    
    // Set direction - slightly beyond expected range to ensure we hit the limit
    float searchDirection = searchingForMax ? 1.0 : -1.0;
    float searchAmount = MAX_ANGLE_DEGREES * 1.2; // 20% beyond expected range
    
    // Get current position as starting point
    float startPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    float targetPosition = startPosition + (searchDirection * searchAmount);
    
    // Command motor to move
    setMotorMoving(true);
    dxl.setGoalPosition(DXL_ID, targetPosition, UNIT_DEGREE);
    
    // Initialize variables for stall detection
    hitEndstop = false;
    stallCheckStart = millis();
    float lastPosition = startPosition;
    int stationaryCount = 0;
    
    // Monitor for stall condition
    while (!hitEndstop) {
        delay(50); // Small delay to not overwhelm serial
        
        // Read current position
        float presentPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
        
        // Check if the motor is stalled (not making progress)
        if (abs(presentPosition - lastPosition) < 0.1) {
            stationaryCount++;
            
            // If stalled for multiple consecutive readings, we've hit the endstop
            if (stationaryCount >= 5) {
                hitEndstop = true;
                DEBUG_SERIAL.print("Endstop detected at position: ");
                DEBUG_SERIAL.println(presentPosition);
                break;
            }
        } else {
            stationaryCount = 0; // Reset if movement detected
        }
        
        // Timeout safety
        if (millis() - stallCheckStart > 5000) {
            DEBUG_SERIAL.println("Timeout while finding endstop. Using current position.");
            hitEndstop = true;
            break;
        }
        
        lastPosition = presentPosition;
    }
    
    // Stop the motor once endstop is detected
    dxl.setGoalPosition(DXL_ID, dxl.getPresentPosition(DXL_ID, UNIT_DEGREE), UNIT_DEGREE);
    setMotorMoving(false);
}

bool MotorController::isMotorCurrentlyMoving() {
    // Read the Moving Status register (address 123)
    // 0 = not moving, 1 = moving
    return dxl.readControlTableItem(MOVING, DXL_ID) == 1;
}

void MotorController::setMotorMoving(bool moving) {
    isMotorMoving = moving;
    if (moving) {
        dxl.ledOn(DXL_ID);
    } else {
        dxl.ledOff(DXL_ID);
    }
}

bool MotorController::pingDynamixel() {
    // Try to ping the motor
    for (int attempts = 0; attempts < 3; attempts++) {
        DEBUG_SERIAL.print("Ping attempt ");
        DEBUG_SERIAL.print(attempts + 1);
        DEBUG_SERIAL.print("... ");
        
        // Check if the motor responds to ping
        if (dxl.ping(DXL_ID)) {
            DEBUG_SERIAL.println("SUCCESS!");
            return true;
        } else {
            DEBUG_SERIAL.println("FAILED");
        }
        
        delay(500); // Short delay between attempts
    }
    return false;
}

void MotorController::testSimpleMovement() {
    // (Commented out in original, left as stub for now)
} 