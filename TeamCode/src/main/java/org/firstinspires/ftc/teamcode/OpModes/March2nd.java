package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Classes.Arm2;
import org.firstinspires.ftc.teamcode.Classes.Arm2gpt;
import org.firstinspires.ftc.teamcode.Classes.IntakeClaw;
import org.firstinspires.ftc.teamcode.Classes.LinearSlide;
import org.firstinspires.ftc.teamcode.Classes.OutputArm;

@Config
@TeleOp(name = "March2nd", group = "Linear Opmode")
public class March2nd extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private double sens = .8;
    private DcMotor FrontLeftMotor = null;
    private DcMotor FrontRightMotor = null;
    private DcMotor BackLeftMotor = null;
    private DcMotor BackRightMotor = null;
    private DcMotor LS1;
    private DcMotor LS2;
    private Servo  intakeRotate = null;
    private Servo intakeWrist = null;

    private long lastPressedTimeBumper = 0;
    private double wristPosition = .5;
    private double rotatePosition = 0.5;

    private Servo outputClaw;
    private Servo outputArm;
    private static final double WRIST_INCREMENT = 0.01;
    public static double FLconstant = .45;
    public static double FRconstant = .45;
    public static double BRconstant = .6;
    public static double BLconstant = .6;
    public static int LSTOP = 4000;
    public static int LSBOT = 0;
    private boolean intakeclawClosed = false;
    private boolean outputclawClosed = false;
    private long lastPressedTimeDpadLeft = 0;
    private long lastPressedTimeDpadRight = 0;
    private long lastPressedTimeDpadUp = 0;
    private long lastPressedTimeDpadDown = 0;
    private long lastPressedTimeB = 0;
    private long lastPressedTimeA = 0;
    private long lastPressedTimeLeftBumper = 0;
    private long lastPressedTimeRightBumper = 0;


    // Initial positions for OutputClaw and OutputWrist




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize hardware
        initializeHardware();

        // Now initialize Arm2 after hardware is set up
        Arm2 intake = new Arm2(hardwareMap,  telemetry);
        IntakeClaw intakeclaw = new IntakeClaw(hardwareMap, "CLAW", "WRIST", "ROTATE");
        OutputArm output = new OutputArm(hardwareMap, "OUTPUTCLAW", "OUTPUTARM");
        LinearSlide LS = new LinearSlide(hardwareMap, new ElapsedTime(), telemetry);
        wristPosition = .5;
        rotatePosition = .5;
        double outputClawPosition = outputClaw.getPosition();
        double outputWristPosition = outputArm.getPosition();

        runtime.reset();
        waitForStart();


        // Start drivetrain control on a separate thread
        Thread drivetrainThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    driveControl();
                    telemetry.update();
                }
            }
        });

        // Start the drivetrain thread
        drivetrainThread.start();

        // Continue with other opera2tions, for example, controlling other components

        //Gamepad2: a->open/close outputclaw, b->open/close inputclaw, left joytstick ->move output arm, dpad up/down->arm ->bumpers wristup/down,
        while (opModeIsActive()) {
            if(Math.abs(gamepad2.right_stick_y)>.1){
                LS1.setPower(gamepad2.right_stick_y);
                LS2.setPower(gamepad2.right_stick_y);}
            LS1.setPower(0);
            LS2.setPower(0);

            if (gamepad2.right_bumper && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeRightBumper)) {
                rotatePosition = Range.clip(rotatePosition - 0.05, 0, 1);
                intakeclaw.setRotatePosition(rotatePosition);
                nonBlockingDelay(50);
            } //check
            if (gamepad1.left_bumper && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeLeftBumper)) {
                rotatePosition = Range.clip( rotatePosition + 0.05, 0 , 1);
                intakeclaw.setRotatePosition(rotatePosition);
                nonBlockingDelay(50);
            } //check
            if (Math.abs(gamepad2.right_trigger)>.1) {
                wristPosition = Math.max(0, wristPosition - 0.05);
                intakeclaw.setWristPosition(wristPosition);
                nonBlockingDelay(50);
            } //check
            if (Math.abs(gamepad1.left_trigger)>.1) {
                wristPosition = Math.min(1, wristPosition + 0.05);
                intakeclaw.setWristPosition(wristPosition);
                nonBlockingDelay(50);
            } //check

            if(gamepad2.dpad_up ){ //check
                intake.moveArmBy(20);
                nonBlockingDelay(10);
            }
            if(gamepad2.dpad_down ){ //check
                intake.moveArmBy(-20);
                nonBlockingDelay(10);
            }
            if(gamepad2.dpad_up){
                intake.moveArmTo(580);
                nonBlockingDelay(10);
            }
            if(gamepad2.dpad_down){
                intake.moveArmTo(580);
                nonBlockingDelay(10);
            }
            if(gamepad2.b && intakeclawClosed && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeB)){
                intakeclawClosed=false;
                intakeclaw.close();
                nonBlockingDelay(50);
            }
            if(gamepad2.b && !intakeclawClosed && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeB)){
                intakeclawClosed=true;
                intakeclaw.open();
                nonBlockingDelay(50);
            }

            if (Math.abs(-gamepad2.left_stick_y) > 0.1) { // Deadzone to prevent accidental movement
                outputWristPosition += -gamepad2.left_stick_y * WRIST_INCREMENT;
                outputWristPosition = Range.clip(outputWristPosition, 0.0, 1.0);
                outputArm.setPosition(outputWristPosition);
                nonBlockingDelay(10);
            }
            if(gamepad2.a && outputclawClosed && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeA)){
                outputclawClosed=false;
                output.OpenOutputClaw();
                nonBlockingDelay(50);
            }
            if(gamepad2.a && !outputclawClosed && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeA)){
                outputclawClosed=true;
                output.CloseOutputClaw();
                nonBlockingDelay(50);
            }
            if(gamepad2.y){
                LS.moveLSTo(LSTOP);
                nonBlockingDelay(10);
            }
            if(gamepad2.x){
                LS.moveLSTo(LSBOT);
                nonBlockingDelay(10);
            }


            telemetry.addData("LS1 Power", LS1.getPower());
            telemetry.addData("LS2 Power", LS2.getPower());
            telemetry.addData("LS1 Position", LS1.getCurrentPosition());
            telemetry.addData("LS2 Position", LS2.getCurrentPosition());
            telemetry.update();
            intake.update();
        }

        // Wait for the drivetrain thread to finish before ending the opmode
        drivetrainThread.join();
    }

    private void driveControl() {
        if(Math.abs(gamepad1.right_trigger)>.1) {
            double y = -gamepad1.left_stick_y; // Forward/backward (left stick vertical)
            double x = gamepad1.left_stick_x;  // Left/right strafing (left stick horizontal)
            double rx = gamepad1.right_stick_x; // Rotation (right stick horizontal)

            // Calculate the largest possible input sum to scale the powers properly
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // Calculate motor powers
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Apply power scaling factor
            FrontLeftMotor.setPower(-frontLeftPower * FLconstant*.25);
            BackLeftMotor.setPower(-backLeftPower * BLconstant*.25);
            FrontRightMotor.setPower(frontRightPower * FRconstant*.25);
            BackRightMotor.setPower(backRightPower * BRconstant*.25);
        }
        else{
            double y = -gamepad1.left_stick_y; // Forward/backward (left stick vertical)
            double x = gamepad1.left_stick_x;  // Left/right strafing (left stick horizontal)
            double rx = gamepad1.right_stick_x; // Rotation (right stick horizontal)

            // Calculate the largest possible input sum to scale the powers properly
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // Calculate motor powers
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Apply power scaling factor
            FrontLeftMotor.setPower(-frontLeftPower * FLconstant);
            BackLeftMotor.setPower(-backLeftPower * BLconstant);
            FrontRightMotor.setPower(frontRightPower * FRconstant);
            BackRightMotor.setPower(backRightPower * BRconstant);

        }
    }




    private void initializeHardware() {
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");
        intakeRotate = hardwareMap.get(Servo.class, "ROTATE");
        intakeWrist = hardwareMap.get(Servo.class, "WRIST");
        outputClaw = hardwareMap.get(Servo.class, "OUTPUTCLAW");
        outputArm = hardwareMap.get(Servo.class, "OUTPUTARM");
        LS1 = hardwareMap.get(DcMotor.class, "LS1");
        LS2 = hardwareMap.get(DcMotor.class, "LS2");




    }
    private static final long DEBOUNCE_DELAY = 200; // 200 milliseconds
    private void nonBlockingDelay(double milliseconds) {
        ElapsedTime delayTimer = new ElapsedTime();
        delayTimer.reset();
        while (opModeIsActive() && delayTimer.milliseconds() < milliseconds) {
            telemetry.update();
        }
    }
    private boolean isButtonPressed(long currentTime, long lastPressedTime) {
        if (currentTime - lastPressedTime > DEBOUNCE_DELAY) {
            lastPressedTime = currentTime;
            return true;
        }
        return false;
    }
}
