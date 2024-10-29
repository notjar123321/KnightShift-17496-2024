package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    public final static double Kp = 0.05; // Proportional gain (adjust with testing)
    public final static double Ki = 0.05; // Integral gain (adjust with testing)
    public final static double Kd = 0.05; // Derivative gain (adjust with testing)

    public static int target_position = 0; // Target position for the arm
    private double integralSum = 0; // Integral for PID control
    private double lastError = 0; // Last error value for PID
    private double lastTime = 0; // Last time update was called
    private double gravityCompensation = 0.005; // Gravity compensation factor
    private double target_velocity = .005; // Target velocity for constant velocity control

    public Arm(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
        motor1 = hardwareMap.get(DcMotor.class, RobotConstants.arm1);
        motor2 = hardwareMap.get(DcMotor.class, RobotConstants.arm2);
        timer = elapsedTime;
        telemetry = telemetryIn;

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE); // Reverse motor2 if needed
    }

    public Arm(HardwareMap hardwareMap, Telemetry telemetryIn) {
        motor1 = hardwareMap.get(DcMotor.class, RobotConstants.arm1);
        motor2 = hardwareMap.get(DcMotor.class, RobotConstants.arm2);
        timer = new ElapsedTime();
        telemetry = telemetryIn;

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE); // Reverse motor2 if needed
    }

    public void update() {
        // Get the current position of the arm
        double pos = (motor1.getCurrentPosition()+motor2.getCurrentPosition())/2;
        double error = target_position - pos; // Position error
        double timePassed = timer.seconds() - lastTime;

        lastTime = timer.seconds();

        // PID calculations
        double derivative;
        if (timePassed == 0) { // Prevent division by zero
            derivative = 0;
        } else {
            derivative = (error - lastError) / timePassed; // Derivative calculation
        }
        integralSum += error * timePassed;

        // Gravity compensation
        double output = Kp * error + Ki * integralSum + Kd * derivative + gravityCompensation;

        // Set motor powers based on target velocity
        double velocityOutput = target_velocity; // For constant velocity control
        motor1.setPower(Range.clip(output + velocityOutput, -1.0, 1.0)); // Ensure power is within limits
        motor2.setPower(Range.clip(output + velocityOutput, -1.0, 1.0));

        // Telemetry updates for debugging
        // Update last error for next iteration
        lastError = error;
    }

    public void moveElbow(int ticks) {
        target_position += ticks;
        motor1.setTargetPosition(target_position);
        motor2.setTargetPosition(target_position);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(0.5); // Adjust power as needed
        motor2.setPower(0.5);
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
