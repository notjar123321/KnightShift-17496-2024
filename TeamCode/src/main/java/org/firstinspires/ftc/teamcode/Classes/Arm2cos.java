package org.firstinspires.ftc.teamcode.Classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Arm2cos {
    private DcMotor motor1;
    private DcMotor motor2;

    private ElapsedTime timer;
    private Telemetry telemetry;

    public static int target_position = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    // PID Constants
    public static double kP = 0.005;
    public static double kI = 0.001;
    public static double kD = 0.001;
    public static double kCos = 0.1; // Gravity compensation constant, to be tuned

    public Arm2cos(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
        target_position = 0;
        motor1 = hardwareMap.get(DcMotor.class, "ArmR");
        motor2 = hardwareMap.get(DcMotor.class, "ArmL");

        timer = elapsedTime;
        telemetry = telemetryIn;

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);


        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
        // Get the average position of both motors
        double currentPosition = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2.0;

        // Calculate the current angle of the arm in radians
        double currentAngle = encoderTicksToRadians(currentPosition);

        // Calculate error
        double error = target_position - currentPosition;

        // Time elapsed for PID calculation
        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;

        // Proportional term
        double pTerm = kP * error;

        // Integral term with anti-windup
        integralSum = Range.clip(integralSum + error * deltaTime, -1.0, 1.0);
        double iTerm = kI * integralSum;

        // Derivative term
        double deltaError = error - lastError;
        double dTerm = (deltaTime > 0) ? kD * (deltaError / deltaTime) : 0;

        // Gravity compensation feedforward term
        double gravityCompensation = kCos * Math.cos(currentAngle);

        // Compute final motor power
        double motorPower = pTerm + iTerm + dTerm + gravityCompensation;
        motorPower = Range.clip(motorPower, -1.0, 1.0);

        // Apply power to both motors
        motor1.setPower(motorPower);
        motor2.setPower(motorPower);

        // Update previous values for next loop
        lastError = error;
        lastTime = currentTime;

        // Telemetry for debugging
        telemetry.addData("Target Position", target_position);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Error", error);
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("Gravity Compensation", gravityCompensation);
        telemetry.update();
    }

    private double encoderTicksToRadians(double ticks) {
        // Replace with your encoder's ticks-to-radians conversion
        double ticksPerRevolution = 1440; // Example value
        return (ticks / ticksPerRevolution) * 2 * Math.PI;
    }

    public void setTargetPosition(int position) {
        target_position = position;
    }
}
