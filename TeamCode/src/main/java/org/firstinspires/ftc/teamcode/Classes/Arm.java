package org.firstinspires.ftc.teamcode.Classes;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm {
    private DcMotor motor1;
    private DcMotor motor2;
    private ElapsedTime timer;
    private Telemetry telemetry;

    public static int target_position = 0; // Target position for the arm
    private double integralSum = 0; // Integral for PID control
    private double lastError = 0; // Last error value for PID
    private double lastTime = 0; // Last time update was called
    private double gravityCompensation = 0.005; // Gravity compensation factor (adjust as needed)
    private double target_velocity = .05; // Target velocity for constant velocity control

    // PID Constants
    private double kP = 0.1; // Proportional constant
    private double kI = 0.01; // Integral constant
    private double kD = 0.01; // Derivative constant

    public Arm(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
        motor1 = hardwareMap.get(DcMotor.class, "ArmL");
        motor2 = hardwareMap.get(DcMotor.class, "ArmR");
        timer = elapsedTime;
        telemetry = telemetryIn;

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE); // Reverse motor2 if needed
    }

    public void update() {
        // Get the current position of the arm
        double pos = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2;
        double error = target_position - pos; // Position error

        // Time elapsed since last update (for PID calculation)
        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;
        double deltaError = error - lastError;

        // Calculate PID output
        double pTerm = kP * error; // Proportional term
        integralSum += error * deltaTime;
        double iTerm = kI * integralSum; // Integral term
        double dTerm = (deltaTime > 0) ? kD * (deltaError / deltaTime) : 0; // Derivative term

        // Total PID output
        double pidOutput = pTerm + iTerm + dTerm;

        // Add gravity compensation to the PID output
        double gravityCompensationOutput = gravityCompensation * Math.signum(error); // Apply gravity compensation in the direction of the error

        // Combine PID and gravity compensation to get final motor power
        double motorPower = Range.clip(pidOutput + gravityCompensationOutput, -1, 1);

        // Apply the motor power to both motors
        motor1.setPower(motorPower);
        motor2.setPower(motorPower);

        // Store the current error and time for the next update
        lastError = error;
        lastTime = currentTime;
    }

    public void moveElbow(int ticks) {
        target_position += ticks;
        motor1.setTargetPosition(target_position);
        motor2.setTargetPosition(target_position);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(.1); // Adjust power as needed
        motor2.setPower(.1);
    }
    public void moveElbowTo(int ticks) {
        target_position = ticks;
        motor1.setTargetPosition(target_position);
        motor2.setTargetPosition(target_position);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(.3); // Adjust power as needed
        motor2.setPower(.3);
    }

    public void moveElbowSmoothly(int targetPosition) {
        int currentPos = (motor1.getCurrentPosition()+ motor2.getCurrentPosition())/2;
        int error = targetPosition - currentPos;

        // Calculate the number of steps for the movement
        int steps = 20;  // You can adjust this value based on your needs
        int stepSize = error / steps;  // Size of each step
        stepSize = (stepSize == 0) ? (error > 0 ? 1 : -1) : stepSize; // Prevent step size of 0

        // Gradually move the arm in small steps towards the target position
        for (int i = 0; i < steps; i++) {
            // Calculate new target position for each step
            sleep(5);
            int partialTarget = currentPos + stepSize;  // Incremental target position

            // Update the arm position
            motor1.setTargetPosition(partialTarget);
            motor2.setTargetPosition(partialTarget);

            // Move arm to the new target position with some small power
            motor1.setPower(0.8);  // Set a moderate speed for smooth movement
            motor2.setPower(0.8);

            // Allow time for the arm to reach the current target position

            }

            // Optionally, reduce speed as we get closer to the target

        }

    public void setTargetVelocity(double velocity) {
        target_velocity = velocity; // Set target velocity in ticks per second
    }

    public void resetTimer() {
        timer.reset(); // Reset the timer
    }

    public int getTargetPosition() {
        return target_position; // Return the target position
    }
}
