package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm2 {
    private DcMotor motor1;
    private DcMotor motor2;
    private ElapsedTime timer;
    private Telemetry telemetry;

    public static int target_position = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private double gravityCompensation = 0.005; // Tune as needed

    // PID Constants
    private double kP = 0.1;
    private double kI = 0.01;
    private double kD = 0.01;
    private double maxIntegral = 10; // Limit integral to prevent windup
    private double maxDerivative = 0.1; // Limit derivative changes

    public Arm2(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
        motor1 = hardwareMap.get(DcMotor.class, RobotConstants.arm1);
        motor2 = hardwareMap.get(DcMotor.class, RobotConstants.arm2);
        timer = elapsedTime;
        telemetry = telemetryIn;

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);

    }

    public void update() {
        // Get the current position of the arm
        double pos = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2.0;
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
        motor2.setPower(motorPower);

        // Update previous values for next loop
        lastError = error;
        lastTime = currentTime;
    }

    public void moveElbowTo(int ticks) {
        target_position = ticks;
        motor1.setTargetPosition(target_position);
        motor2.setTargetPosition(target_position);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double currentPower = 0.1; // Initial low power
        double maxPower = 0.5; // Maximum allowable power

        while (motor1.isBusy() && motor2.isBusy()) {
            int currentPos = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2;
            int distanceToTarget = Math.abs(target_position - currentPos);

            // Gradual power increase
            currentPower = Range.clip(currentPower + 0.01, 0.1, maxPower);
            if (distanceToTarget < 50) {
                currentPower *= 0.5; // Slow down near target
            }

            motor1.setPower(currentPower);
            motor2.setPower(currentPower);
        }
    }
    public void moveElbow(int ticks) {
        target_position += ticks;
        motor1.setTargetPosition(target_position);
        motor2.setTargetPosition(target_position);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double currentPower = 0.1; // Initial low power
        double maxPower = 0.5; // Maximum allowable power

        while (motor1.isBusy() && motor2.isBusy()) {
            int currentPos = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2;
            int distanceToTarget = Math.abs(target_position - currentPos);

            // Gradual power increase
            currentPower = Range.clip(currentPower + 0.01, 0.1, maxPower);
            if (distanceToTarget < 50) {
                currentPower *= 0.5; // Slow down near target
            }

            motor1.setPower(currentPower);
            motor2.setPower(currentPower);
        }
    }

    public void moveElbowSmoothly(int targetPosition) {
        int currentPos = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2;
        int error = targetPosition - currentPos;

        // Calculate the number of steps for smooth movement
        int steps = 20;
        int stepSize = error / steps;
        stepSize = (stepSize == 0) ? (error > 0 ? 1 : -1) : stepSize;

        for (int i = 0; i < steps; i++) {
            int partialTarget = currentPos + stepSize * (i + 1);

            // Move arm gradually towards the target
            motor1.setTargetPosition(partialTarget);
            motor2.setTargetPosition(partialTarget);

            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motor1.setPower(0.3);
            motor2.setPower(0.3);

            // Delay to control smoothness
            sleep(10);
        }

        // Final adjustment at lower power for precision
        motor1.setPower(0.5);
        motor2.setPower(0.5);
    }

    public void setTargetPosition(int position) {
        target_position = position;
    }
}
