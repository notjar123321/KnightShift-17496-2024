package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Classes.Arm2;
import org.firstinspires.ftc.teamcode.Classes.IntakeClaw;
import org.firstinspires.ftc.teamcode.Classes.LinearSlide;

@Config
@TeleOp(name = "March2nd", group = "Linear Opmode")
public class March2nd extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();



    private double sens = .8;
    boolean isClawOpen = false;
    boolean isOutputClawOpen = false;
    private DcMotor FrontLeftMotor=null;
    private DcMotor FrontRightMotor=null;
    private DcMotor BackLeftMotor=null;
    private DcMotor BackRightMotor=null;
    private DcMotor IntakeMotor=null;




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeHardware();



        runtime.reset();


        waitForStart();
        while (opModeIsActive()) {
            //drive with gamepad1
            telemetry.update();

            // POV Mode control
            double y = gamepad1.left_stick_y; // Forward/backward (left stick vertical)
            double x = -gamepad1.left_stick_x;  // Left/right strafing (left stick horizontal)
            double rx = gamepad1.right_stick_x; // Rotation (right stick horizontal)

            // Calculate the largest possible input sum to scale the powers properly
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // Calculate motor powers
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Apply power scaling factor
            FrontLeftMotor.setPower(-frontLeftPower * sens);
            BackLeftMotor.setPower(-backLeftPower * sens);
            FrontRightMotor.setPower(frontRightPower * sens);
            BackRightMotor.setPower(backRightPower * sens);



            telemetry.update();

        }
    }


    private void nonBlockingDelay(double milliseconds) {
        ElapsedTime delayTimer = new ElapsedTime();
        delayTimer.reset();
        while (opModeIsActive() && delayTimer.milliseconds() < milliseconds) {
            telemetry.update();
        }
    }
    private void initializeHardware() {






        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");









    }
    private static final long DEBOUNCE_DELAY = 200; // 200 milliseconds
    private boolean isButtonPressed(long currentTime, long lastPressedTime) {
        if (currentTime - lastPressedTime > DEBOUNCE_DELAY) {
            return true;
        }
        return false;
    }
}