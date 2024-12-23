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


    private ElapsedTime timer;
    private Telemetry telemetry;

    public static int target_position = 0; // Target position for the arm
    private double integralSum = 0; // Integral for PID control
    private double lastError = 0; // Last error value for PID
    private double lastTime = 0; // Last time update was called
    private double gravityCompensation = 0.00005; // Gravity compensation factor (adjust as needed)
    private double target_velocity = .05; // Target velocity for constant velocity control

    // PID Constants
    private double kP = 0.01; // Proportional constant
    private double kI = 0.001; // Integral constant
    private double kD = 0.001; // Derivative constant

    private double maxIntegral = .05; // Limit integral to prevent windup
    private double maxDerivative = 0.0003; // Limit derivative changes

    public Arm2(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
        target_position=0;
        motor1 = hardwareMap.get(DcMotor.class, "INTAKE");


        timer = elapsedTime;
        telemetry = telemetryIn;

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use the encoder for power control
        wrist1 = hardwareMap.get(Servo.class, "INPUTLEFT");
        wrist2 = hardwareMap.get(Servo.class, "OUTPUTRIGHT"); //remember to change in config
        wrist2.setDirection(Servo.Direction.REVERSE);



    }

    public void update() {

        // Get the current position of the arm
        double pos = (motor1.getCurrentPosition());
        double error = target_position - pos;
        if(pos<0)
        {
            target_position=0;
        }
        if(pos<100){
            wrist1.setPosition(0);
            wrist2.setPosition(0);
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

        // Apply power to motors
        motor1.setPower(motorPower);


        // Update previous values for next loop
        lastError = error;
        lastTime = currentTime;
        sleep(20);
    }

    public void moveElbowTo(int ticks) {
        target_position = ticks;
        target_position=Range.clip(target_position, 0, 10000);
        motor1.setTargetPosition(target_position);


        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double currentPower=1;

        while (motor1.isBusy()) {
            int currentPos = (motor1.getCurrentPosition());
            int distanceToTarget = Math.abs(target_position - currentPos);
            motor1.setPower(currentPower);
            if(distanceToTarget<50){
                currentPower*=.1;
            }

        }
    }
    public void moveElbow(int ticks) {

        target_position += ticks;
        target_position=Range.clip(target_position, 0, 10000);
        motor1.setTargetPosition(target_position);


        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double currentPower=1;
        double maxPower = 1; // Maximum allowable power



        while (motor1.isBusy() ) {
            int currentPos = (motor1.getCurrentPosition());
            int distanceToTarget = Math.abs(target_position - currentPos);
            // Gradual power increase
            motor1.setPower(currentPower);

        }
    }



    public void setTargetPosition(int position) {
        target_position = position;
    }
}
