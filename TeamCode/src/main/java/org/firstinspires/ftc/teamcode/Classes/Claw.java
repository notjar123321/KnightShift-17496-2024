package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    // Servo instances
    private Servo leftServo;
    private Servo rightServo;

    // Positions for open and closed states
    private static final double LEFT_SERVO_OPEN = 0.0;
    private static final double LEFT_SERVO_CLOSED = 1.0;
    private static final double RIGHT_SERVO_OPEN = 1.0;
    private static final double RIGHT_SERVO_CLOSED = 0.0;

    // Constructor
    public Claw(HardwareMap hardwareMap, String leftServoName, String rightServoName) {
        leftServo = hardwareMap.get(Servo.class, leftServoName);
        rightServo = hardwareMap.get(Servo.class, rightServoName);
    }

    // Method to open the claw
    public void open() {
        leftServo.setPosition(LEFT_SERVO_OPEN);
        rightServo.setPosition(RIGHT_SERVO_OPEN);
    }

    // Method to close the claw
    public void close() {
        leftServo.setPosition(LEFT_SERVO_CLOSED);
        rightServo.setPosition(RIGHT_SERVO_CLOSED);
    }

    // Method to set custom positions for the servos
    public void setPositions(double leftPosition, double rightPosition) {
        leftServo.setPosition(leftPosition);
        rightServo.setPosition(rightPosition);
    }
}
