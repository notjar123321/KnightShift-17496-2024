package org.firstinspires.ftc.teamcode.Classes;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Arm2 {
    private DcMotor motor1;

    private Servo wrist1 = null; // First wrist servo
    private Servo wrist2 = null;
    private DcMotorSimple wrist3 = null;
    private ElapsedTime timer;
    private Telemetry telemetry;

    public static int target_position = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private double gravityCompensation = 0.02; // Tune as needed

    // PID Constants
    private double kP = 0.1;
    private double kI = 0.01;
    private double kD = 0.01;
    private double maxIntegral = 1; // Limit integral to prevent windup
    private double maxDerivative = 0.1; // Limit derivative changes

    public Arm2(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
        target_position=0;
        motor1 = hardwareMap.get(DcMotor.class, RobotConstants.arm1);

        wrist1 = hardwareMap.get(Servo.class, "wrist1");

        wrist3 = hardwareMap.get(DcMotorSimple.class, "wrist3");
        timer = elapsedTime;
        telemetry = telemetryIn;

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use the encoder for power control



    }

    public void update() {
        /**if(motor1.getCurrentPosition() < 300) {
            wrist1.setPosition(.95);
            wrist2.setPosition(.95);
        }**/
        // Get the current position of the arm
        double pos = (motor1.getCurrentPosition());
        double error = target_position - pos;

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

        // Apply power to motors
        motor1.setPower(motorPower);


        // Update previous values for next loop
        lastError = error;
        lastTime = currentTime;
    }

    public void moveElbowTo(int ticks) {
        target_position = ticks;
        motor1.setTargetPosition(target_position);


        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double currentPower = 0.4; // Initial low power
        double maxPower = 0.9; // Maximum allowable power

        while (motor1.isBusy()) {
            int currentPos = (motor1.getCurrentPosition());
            int distanceToTarget = Math.abs(target_position - currentPos);

            // Gradual power increase
            currentPower = Range.clip(currentPower + 0.01, 0.1, maxPower);
            if (distanceToTarget < 50) {
                currentPower *= 0.5; // Slow down near target
            }

            motor1.setPower(currentPower);

        }
    }
    public void moveElbow(int ticks) {

        target_position += ticks;
        motor1.setTargetPosition(target_position);


        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double currentPower = 0.6; // Initial low power
        double maxPower = 0.8; // Maximum allowable power

        while (motor1.isBusy() ) {
            int currentPos = (motor1.getCurrentPosition());
            int distanceToTarget = Math.abs(target_position - currentPos);

            // Gradual power increase
            currentPower = Range.clip(currentPower + 0.01, 0.1, maxPower);
            if (distanceToTarget < 50) {
                currentPower *= 0.5; // Slow down near target
            }

            motor1.setPower(currentPower);

        }
    }

    public void moveElbowSmoothly(int targetPosition) {
        target_position = targetPosition;

        int error = targetPosition - targetPosition;

        // Calculate the number of steps for smooth movement
        int steps = 20;
        int stepSize = error / steps;
        stepSize = (stepSize == 0) ? (error > 0 ? 1 : -1) : stepSize;

        for (int i = 0; i < steps; i++) {
            int partialTarget = targetPosition + stepSize * (i + 1);

            // Move arm gradually towards the target
            motor1.setTargetPosition(partialTarget);


            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor1.setPower(0.8);


            // Delay to control smoothness
            sleep(10);
        }

        // Final adjustment at lower power for precision
        motor1.setPower(0.8);

    }

    public void setTargetPosition(int position) {
        target_position = position;
    }
}
