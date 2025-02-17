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
    private DcMotor motor2; // Second motor



    private ElapsedTime timer;
    private Telemetry telemetry;

    public static int target_position = 0; // Target position for the arm
    private double integralSum = 0; // Integral for PID control
    private double lastError = 0; // Last error value for PID
    private double lastTime = 0; // Last time update was called
    public static double gravityCompensation = 0.025; // Gravity compensation factor (adjust as needed)

    // PID Constants
    public static double kP = 0.005; // Proportional constant
    public static double kI = 0.001; // Integral constant
    public static double kD = 0.001; // Derivative constant
    public static double reduce = .8;

    public static double maxIntegral = .2; // Limit integral to prevent windup
    public double maxDerivative = 0.0003; // Limit derivative changes

    public Arm2(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
        target_position = 0;
        motor1 = hardwareMap.get(DcMotor.class, "ArmR");
        motor2 = hardwareMap.get(DcMotor.class, "ArmL"); // Second motor

        timer = elapsedTime;
        telemetry = telemetryIn;

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
        // Get the average position of both motors
        double pos1 = motor2.getCurrentPosition() ;

        double error = target_position-pos1;



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
        double motorPower = Range.clip(.1*Math.cos(pos1-60) + gravityCompensation * Math.signum(error), -1, 1);

        // Apply power to both motors


        // Set a fixed power level for `RUN_TO_POSITION`
        motor1.setPower(motorPower);
        motor2.setPower(motorPower);

        // Update previous values for next loop
        lastError = error;
        lastTime = currentTime;
        telemetry.addData("inegral windup", integralSum + error * deltaTime);
        telemetry.addData("P Term", pTerm);
        telemetry.addData("I Term", iTerm);
        telemetry.addData("D Term", dTerm);
        telemetry.addData("motor1 pos", motor1.getCurrentPosition());
        telemetry.addData("motor2 pos", motor2.getCurrentPosition());
        telemetry.addData("error", error);
        telemetry.addData("motor power", pidOutput);
        telemetry.addData("real motor power", motor1.getPower());
        telemetry.addData("target position", target_position);


        telemetry.update();
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

    public void setTargetPosition(int position) {

        target_position = position;
    }
}
