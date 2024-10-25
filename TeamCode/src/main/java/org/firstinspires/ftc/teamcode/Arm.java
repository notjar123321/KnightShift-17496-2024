package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm {
    private DcMotor motor1;
    private DcMotor motor2;
    private ElapsedTime timer;
    private Telemetry telemetry;
    public final static double Kp = 0.1; //fix with testing
    public final static double Ki = 0.1; //fix with testing
    public final static double Kd = 0.1; //fix with testing
    private int target_position = 0;
    private double integralSum = 0; //fix with testing
    private double lastError = 0;
    private double lastTime = 0;
    private double timePassed = 0;


    public Arm(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn)
    {
        motor1 = hardwareMap.get(DcMotor.class, RobotConstants.arm1);
        motor2 = hardwareMap.get(DcMotor.class, RobotConstants.arm2);
        timer = elapsedTime;
        telemetry = telemetryIn;


        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
    }

    public Arm(HardwareMap hardwareMap, Telemetry telemetryIn)
    {
        motor1 = hardwareMap.get(DcMotor.class, RobotConstants.arm1);
        motor2 = hardwareMap.get(DcMotor.class, RobotConstants.arm2);
        timer = new ElapsedTime();
        telemetry = telemetryIn;
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
    }

    public void update(){

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Output: ", "not started");
        telemetry.addData("Target Position: ", target_position);
        telemetry.update();

        //setting variables
        double pos = motor1.getCurrentPosition();
        double error = target_position - pos;
        timePassed = timer.seconds() - lastTime;

        lastTime = timer.seconds();

        //doing math
        double derivative;
        if (timePassed == 0)    //preventing division by zero
            derivative = 0;
        else
            derivative = (error - lastError) / timePassed;
        integralSum += error * timePassed;

        //actually outputting
        double output = Kp * error + Ki * integralSum + Kd * derivative;
        motor1.setPower(output);
        motor2.setPower(output);

        telemetry.addData("Status", "Run Time: " + timer.seconds());
        telemetry.addData("Output: ", output);

        //setting up for next loop
        lastError = error;
    }
    public void moveElbow(int ticks)
    {
        target_position += ticks;
    }

    public void resetTimer()
    {
        timer.reset();
    }

}