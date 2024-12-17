package org.firstinspires.ftc.teamcode;

import android.renderscript.ScriptGroup;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutoModes.LinearSlide;
import org.firstinspires.ftc.teamcode.Classes.Arm2;
import org.firstinspires.ftc.teamcode.Classes.Claw;


@TeleOp(name = "Jan5th", group = "Linear Opmode")
public class Jan5th extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor ls1 = null; // First motor for arm rotation
    private DcMotor ls2 = null;
    private double y;
    private double x;
    private double rx;
    private double sens = 1;
    boolean isClawOpen = false;
    boolean isOutputClawOpen = false;
    private DcMotor FrontLeftMotor=null;
    private DcMotor FrontRightMotor=null;
    private DcMotor BackLeftMotor=null;
    private DcMotor BackRightMotor=null;
    private DcMotor IntakeMotor=null;
    private DcMotorSimple OutputArmServo=null;
    private Servo INPUTLEFT = null;
    private Servo INPUTRIGHT = null;
    private Servo CLAWLEFT = null;
    private Servo CLAWRIGHT = null;
    private double InputWristPosition;





    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeHardware();

        waitForStart();

        runtime.reset();
        LinearSlide LS = new LinearSlide(hardwareMap, new ElapsedTime(), telemetry);
        Arm2 intake = new Arm2(hardwareMap, new ElapsedTime(), telemetry);
        Claw intakeclaw = new Claw(hardwareMap, "CLAWLEFT", "CLAWRIGHT");
        //Claw outputclaw = new Claw(hardwareMap, "OUTPUTCLAWLEFT", "OUTPUTCLAWRIGHT");


        while (opModeIsActive()) {
            //drive with gamepad1
            double leftPower;
            double rightPower;

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

            //Linear Slide Commands(Dpad)
            if (gamepad2.dpad_left) {
                nonBlockingDelay(10);
                LS.moveLSTo(1500);
            }
            if (gamepad2.dpad_down) {
                nonBlockingDelay(10);
                LS.moveLSTo(0);
            }
            if (gamepad2.dpad_up) {
                nonBlockingDelay(10);
                LS.moveLSTo(3100);
            }
            if(gamepad2.left_stick_y!=0){
                intake.moveElbow((int) (gamepad2.left_stick_y*30));
            }
            if (gamepad2.x) {
                isClawOpen = !isClawOpen; // Toggle state
                if (isClawOpen) {
                    intakeclaw.open(); // Open claw
                } else {
                    intakeclaw.close(); // Close claw
                }
                nonBlockingDelay(20);
            }
            /**if (gamepad2.y) {
                isOutputClawOpen = !isOutputClawOpen; // Toggle state
                if (isOutputClawOpen) {
                    outputclaw.open(); // Open claw
                } else {
                    outputclaw.close(); // Close claw
                }
            }**/
            if (gamepad2.right_bumper) {
                InputWristPosition=INPUTLEFT.getPosition();
                INPUTLEFT.setPosition(InputWristPosition+.1);
                INPUTRIGHT.setPosition(InputWristPosition+.1);
                nonBlockingDelay(50);
            } if (gamepad2.left_bumper) {
                InputWristPosition=INPUTLEFT.getPosition();
                INPUTLEFT.setPosition(InputWristPosition-.1);
                INPUTRIGHT.setPosition(InputWristPosition-.1);
                nonBlockingDelay(50);
            }





                telemetry.addData("Right stick y", gamepad2.right_stick_y);
            if(gamepad2.right_stick_y!=0){
                OutputArmServo.setPower(gamepad2.right_stick_y);
            }
            else{
                OutputArmServo.setPower(0);
            }


            //Claw Commands
            


            telemetry.addData("LS1 position", ls1.getCurrentPosition());
            telemetry.addData("LS1 position", ls2.getCurrentPosition());
            telemetry.addData("Intake Position", IntakeMotor.getCurrentPosition());
            telemetry.addData("INPUTLEFT", INPUTLEFT.getPosition());
            telemetry.addData("INPUTRIGHT", INPUTRIGHT.getPosition());

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

        ls1 = hardwareMap.get(DcMotor.class, "LS1");
        ls2 = hardwareMap.get(DcMotor.class, "LS2");
        // Initialize motors
        ls1.setDirection(DcMotor.Direction.FORWARD);
        ls2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set up scissor lift motors with encoders
        ls1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ls2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LinearSlide LS = new LinearSlide(hardwareMap, new ElapsedTime(), telemetry);
        Arm2 intake = new Arm2(hardwareMap, new ElapsedTime(), telemetry);
        Claw claw = new Claw(hardwareMap, "CLAWLEFT", "CLAWRIGHT");

        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");

        IntakeMotor = hardwareMap.get(DcMotor.class, "INTAKE");

        //wrists
        INPUTLEFT = hardwareMap.get(Servo.class, "INPUTLEFT");
        INPUTRIGHT = hardwareMap.get(Servo.class, "OUTPUTRIGHT"); //remember to change in config

        OutputArmServo = hardwareMap.get(DcMotorSimple.class, "OUTPUTARM");
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




    }
}