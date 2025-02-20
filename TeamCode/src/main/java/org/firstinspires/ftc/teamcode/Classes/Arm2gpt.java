package org.firstinspires.ftc.teamcode.Classes;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm2gpt {
    private DcMotor motor1;
    private DcMotor motor2; // Second motor

    private ElapsedTime timer;
    private Telemetry telemetry;

    public static int target_position = 0; // Target position for the arm
    private double integralSum = 0; // Integral for PID control
    private double lastError = 0; // Last error value for PID
    private double lastTime = 0; // Last time update was called
    public static double gravityCompensation = 0.025; // Gravity compensation factor (adjust as needed)
    public static long sleepytime = 20;

    // PID Constants (adjusted)
    public static double kP = 0.002; // Lower proportional constant to reduce overshoot
    public static double kI = 0.00005; // Lower integral constant for smoother control
    public static double kD = 0.00005; // Lower derivative constant for smoother adjustment
    public static double reduce = .8;
    public static double IntegralSumLimit = 1;
    public static double errordivisor = 10;

    public static double maxIntegral = .2; // Limit integral to prevent windup
    public double maxDerivative = 0.0003; // Limit derivative changes

    public Arm2gpt(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
        target_position = 0;
        motor1 = hardwareMap.get(DcMotor.class, "ArmR");
        motor2 = hardwareMap.get(DcMotor.class, "ArmL"); // Second motor

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
        // Average position from both motors
        target_position = Range.clip(target_position, 0, 300);
        double posAverage = motor2.getCurrentPosition();
        double error = target_position - posAverage;

        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;

        // Calculate velocity (change in position over time)
        double velocity = (motor2.getCurrentPosition()) / (deltaTime*1000);

        // PID calculations
        double pTerm = kP * error;
        integralSum += error * deltaTime;
        integralSum = Range.clip(integralSum, -IntegralSumLimit, IntegralSumLimit);
        double iTerm = kI * integralSum;
        double deltaError = error - lastError;
        double dTerm = (deltaTime > 0) ? kD * (deltaError / deltaTime) : 0;

        double pidOutput = pTerm + iTerm + dTerm;

        // Apply gravity compensation and clip the final output
        double motorPower = Range.clip(pidOutput + gravityCompensation * Math.signum(error) + 0.005, -1, 1);
        double scaledPower = motorPower * (1 - Math.abs(posAverage) / 10000.0);

        // Limit power if position is under 50 ticks
        if (Math.abs(posAverage) < 50) {
            scaledPower = Range.clip(scaledPower, -0.05, 0.25);
        }

        // Apply braking when approaching the target
        if (Math.abs(error) < 10) {
            // If we're getting close to the target, apply more aggressive braking if velocity is high
            if (Math.abs(velocity) > 4) {  // If velocity is too high, apply brakes
                motor1.setPower(Range.clip(scaledPower, -0.2, .2));  // Apply braking force
                motor2.setPower(Range.clip(scaledPower, -0.2, .2));
            } else {
                motor1.setPower(Range.clip(scaledPower, -0.3, 0.3));  // Lower power when close
                motor2.setPower(Range.clip(scaledPower, -0.3, 0.3));
            }
        } else {
            motor1.setPower(scaledPower);
            motor2.setPower(scaledPower);
        }

        // Update for next loop
        lastError = error;

        // Telemetry for debugging
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Motor1 Pos", motor1.getCurrentPosition());
        telemetry.addData("Motor2 Pos", motor2.getCurrentPosition());
        telemetry.addData("Error", error);
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("Target Position", target_position);
        telemetry.update();

        sleep(sleepytime);
        lastTime = currentTime;
    }

    public void moveElbowTo(int ticks) {
        target_position = Range.clip(ticks, 0, 10000);
        motor1.setTargetPosition(target_position);
        motor2.setTargetPosition(target_position);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motor1.isBusy() || motor2.isBusy()) {
            double power = 0.4;
            motor1.setPower(power);
            motor2.setPower(power);
        }
        motor1.setPower(0);
        motor2.setPower(0);
        telemetry.update();
    }

    public void moveElbow(int ticks) {
        target_position = Range.clip(target_position + ticks, 0, 10000);
        motor1.setTargetPosition(target_position);
        motor2.setTargetPosition(target_position);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motor1.isBusy() || motor2.isBusy()) {
            double power = 0.4;
            motor1.setPower(power);
            motor2.setPower(power);
        }
        motor1.setPower(0);
        motor2.setPower(0);
        telemetry.update();
    }

    public void moveArmBy(int position) {
        target_position += position;
    }
}
