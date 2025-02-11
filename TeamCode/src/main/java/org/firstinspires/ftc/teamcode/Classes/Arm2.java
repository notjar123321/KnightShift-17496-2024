package org.firstinspires.ftc.teamcode.Classes;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm2 {
    private DcMotor motor1;
    private DcMotor motor2; // Second motor

    private Servo wrist1 = null; // First wrist servo
    private Servo wrist2 = null; // Second wrist servo

    private ElapsedTime timer;
    private Telemetry telemetry;

    public static int target_position = 0; // Target position for the arm
    private double integralSum = 0; // Integral for PID control
    private double lastError = 0; // Last error value for PID
    private double lastTime = 0; // Last time update was called
    private double gravityCompensation = 0.0005; // Gravity compensation factor (adjust as needed)

    // PID Constants
    private double kP = 0.01; // Proportional constant
    private double kI = 0.001; // Integral constant
    private double kD = 0.001; // Derivative constant
    public static double reduce = .8;

    private double maxIntegral = .05; // Limit integral to prevent windup
    private double maxDerivative = 0.0003; // Limit derivative changes

    public Arm2(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
        target_position = 0;
        motor1 = hardwareMap.get(DcMotor.class, "ArmR");
        motor2 = hardwareMap.get(DcMotor.class, "ArmL"); // Second motor

        timer = elapsedTime;
        telemetry = telemetryIn;

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE); // Set direction for second motor
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder for second motor
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use the encoder for second motor

    }

    public void update() {
        // Get the current position of both motors
        double pos1 = motor1.getCurrentPosition();
        double pos2 = motor2.getCurrentPosition();
        double error = target_position - pos1;

        if (pos1 < 0) {
            target_position = 0;
        }

        // Time elapsed for PID calculation
        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;

        // Proportional term
        double pTerm = kP * error;

        // Integral term with limit
        integralSum = Range.clip(integralSum + error * deltaTime, -maxIntegral, maxIntegral);
        double iTerm = kI * integralSum;

        // Derivative term with limit
        double deltaError = error - lastError;
        double dTerm = (deltaTime > 0) ? kD * Range.clip(deltaError / deltaTime, -maxDerivative, maxDerivative) : 0;

        // PID output with gravity compensation
        double pidOutput = pTerm + iTerm + dTerm;
        double motorPower = Range.clip(pidOutput + gravityCompensation * Math.signum(error), -0.5, 0.5);

        // Apply power to both motors
        motor1.setPower(motorPower);
        motor2.setPower(motorPower);

        // Update previous values for next loop
        lastError = error;
        lastTime = currentTime;
        sleep(5);
        telemetry.update();
    }

    public void moveElbowTo(int ticks) {
        target_position = ticks;
        target_position = Range.clip(target_position, 0, 10000);
        motor1.setTargetPosition(target_position);
        motor2.setTargetPosition(target_position); // Set the target for second motor as well
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set second motor mode

        while (motor1.isBusy() && motor2.isBusy()) {
            double power = 1;
            motor1.setPower(power);
            motor2.setPower(power); // Apply power to both motors
        }
        telemetry.update();
    }

    public void moveElbow(int ticks) {
        target_position += ticks;
        target_position = Range.clip(target_position, 0, 10000);
        motor1.setTargetPosition(target_position);
        motor2.setTargetPosition(target_position); // Set target for second motor
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set second motor mode

        while (motor1.isBusy() && motor2.isBusy()) {
            double currentPower = 1;
            motor1.setPower(currentPower);
            motor2.setPower(currentPower); // Apply power to both motors
        }
    }

    public void setTargetPosition(int position) {
        target_position = position;
    }
}
