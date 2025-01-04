package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Classes.IntakeClaw;
import org.firstinspires.ftc.teamcode.Classes.LinearSlide;
import org.firstinspires.ftc.teamcode.Classes.Arm2;
import org.firstinspires.ftc.teamcode.Classes.OutputClaw;

@Config
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
    private Servo OutputArmServo=null;
    private Servo INPUTLEFT = null;
    private Servo INPUTRIGHT = null;
    private Servo CLAWLEFT = null;
    private Servo CLAWRIGHT = null;
    private Servo OutputArmWrist = null;
    private double InputWristPosition;
    private double OutputArmPosition;
    private long lastPressedTimeX = 0;
    private long lastPressedTimeBumper = 0;
    private double OutputWristPosition;
    public int intakeZero = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeHardware();



        runtime.reset();
        LinearSlide LS = new LinearSlide(hardwareMap, new ElapsedTime(), telemetry);
        Arm2 intake = new Arm2(hardwareMap, new ElapsedTime(), telemetry);
        IntakeClaw intakeclaw = new IntakeClaw(hardwareMap, "CLAWLEFT", "CLAWRIGHT");

        intakeclaw.close();

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

            //Linear Slide Commands(Dpad)
            if (gamepad2.dpad_left  && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)) {
                nonBlockingDelay(10);
                LS.moveLSTo(1500);
            }
            if (gamepad2.dpad_down  && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)) {
                nonBlockingDelay(10);
                LS.moveLSTo(0);
            }
            if (gamepad2.dpad_up  && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)) {
                nonBlockingDelay(10);
                LS.moveLSTo(3300);
            }
            if(gamepad2.a  && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)){
                INPUTLEFT.setPosition(0);
                INPUTRIGHT.setPosition(0);
                intakeclaw.close();
                intake.moveElbowTo(100);
                nonBlockingDelay(20);
                sleep(10);
            }
            if(gamepad2.b  && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)){
                intake.moveElbowTo(520);
                intake.moveElbow(50);
                nonBlockingDelay(10);
            }
            if(gamepad2.left_stick_y!=0){
                intake.moveElbow((int) (gamepad2.left_stick_y*20));
            }
            if (gamepad2.right_stick_y != 0) {
                double delta = 0.005 * gamepad2.right_stick_y;
                OutputArmPosition = Range.clip(OutputArmPosition + delta, 0.05, 1);
                OutputArmServo.setPosition(OutputArmPosition);
            }

            if(gamepad2.y && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)){ //whole sequence
                intake.moveElbowTo(400);
                OutputArmServo.setPosition(.05);
                OutputArmWrist.setPosition(.7);
                nonBlockingDelay(1500);
                intake.moveElbowTo(223);
                nonBlockingDelay(1500);
                INPUTLEFT.setPosition(Range.clip( .5, 0, 1));
                INPUTRIGHT.setPosition(Range.clip( .5, 0, 1));
                nonBlockingDelay(800);
                intakeclaw.open();
                nonBlockingDelay(500);
                INPUTLEFT.setPosition(Range.clip( .4, 0, 1));
                INPUTRIGHT.setPosition(Range.clip( .4, 0, 1));
                nonBlockingDelay(300);
                INPUTLEFT.setPosition(Range.clip( .1, 0, 1));
                INPUTRIGHT.setPosition(Range.clip( .1, 0, 1));
                intake.moveElbow(90);
                LS.moveLSTo(3350);
                nonBlockingDelay(2500);
                OutputArmServo.setPosition(.85);
                nonBlockingDelay(1200);
                OutputArmWrist.setPosition(.65);
                nonBlockingDelay(300);
                OutputArmWrist.setPosition(0);
                nonBlockingDelay(500);
                OutputArmServo.setPosition(.2);
                nonBlockingDelay(5);
                OutputArmWrist.setPosition(.3);
                LS.moveLSTo(0);
                OutputArmPosition=OutputArmServo.getPosition();
                OutputWristPosition=OutputArmWrist.getPosition();
            }
            if (gamepad2.x && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeX)) {
                lastPressedTimeX = (long) runtime.milliseconds();
                isClawOpen = !isClawOpen; // Toggle state
                if (isClawOpen) {
                    intakeclaw.open(); // Open claw
                } else {
                    intakeclaw.close(); // Close claw
                }
            }

            // Input Wrist Movement with Debounce for Bumper Buttons
            if (gamepad2.right_bumper && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)) {
                lastPressedTimeBumper = (long) runtime.milliseconds();
                OutputWristPosition = OutputArmWrist.getPosition();
                OutputArmWrist.setPosition(Range.clip(OutputWristPosition + 0.1, 0, 1));


            }
            if(gamepad2.right_trigger!=0 && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)){
                lastPressedTimeBumper = (long) runtime.milliseconds();
                OutputWristPosition = OutputArmWrist.getPosition();
                OutputArmWrist.setPosition(Range.clip(OutputWristPosition - 0.1, 0, 1));

            }


            if (gamepad2.left_bumper && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper) ) {
                lastPressedTimeBumper = (long) runtime.milliseconds();
                InputWristPosition = INPUTLEFT.getPosition();
                INPUTLEFT.setPosition(Range.clip(InputWristPosition + 0.1, 0, 1));
                INPUTRIGHT.setPosition(Range.clip(InputWristPosition + 0.1, 0, 1));
            }
            if(gamepad2.left_trigger!=0 && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)){
                lastPressedTimeBumper = (long) runtime.milliseconds();
                InputWristPosition = INPUTLEFT.getPosition();
                INPUTLEFT.setPosition(Range.clip(InputWristPosition - 0.1, 0, 1));
                INPUTRIGHT.setPosition(Range.clip(InputWristPosition - 0.1, 0, 1));
            }


            if(gamepad1.y && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)){
                intakeclaw.close();


            }
            telemetry.addData("Right stick y", gamepad2.right_stick_y);
            telemetry.addData("LS1 position", ls1.getCurrentPosition());
            telemetry.addData("LS1 position", ls2.getCurrentPosition());
            telemetry.addData("Intake Position", IntakeMotor.getCurrentPosition());
            telemetry.addData("INPUTLEFT", INPUTLEFT.getPosition());
            telemetry.addData("INPUTRIGHT", INPUTRIGHT.getPosition());
            telemetry.addData("Output Arm Servo", OutputArmServo.getPosition());
            telemetry.addData("Claw pos 1", CLAWLEFT.getPosition());
            telemetry.addData("Claw pos 2", CLAWRIGHT.getPosition());
            telemetry.addData("Gampad2 Left Stick", gamepad2.left_stick_y);
            telemetry.addData("OutputWrist", OutputArmWrist.getPosition());
            telemetry.addData("Input Wrist", INPUTLEFT.getPosition());

            telemetry.update();
            intake.update();
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
        IntakeClaw intakeClaw = new IntakeClaw(hardwareMap, "CLAWLEFT", "CLAWRIGHT");

        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");

        IntakeMotor = hardwareMap.get(DcMotor.class, "INTAKE");
        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use the encoder for power control

        //Claws
        CLAWLEFT = hardwareMap.get(Servo.class, "CLAWLEFT");
        CLAWRIGHT = hardwareMap.get(Servo.class, "CLAWRIGHT");


        //wrists
        INPUTLEFT = hardwareMap.get(Servo.class, "INPUTLEFT");
        INPUTRIGHT = hardwareMap.get(Servo.class, "INPUTRIGHT"); //remember to change in config
        INPUTRIGHT.setDirection(Servo.Direction.REVERSE);

        OutputArmServo = hardwareMap.get(Servo.class, "OUTPUTARM");
        OutputArmWrist = hardwareMap.get(Servo.class, "OUTPUTWRIST");
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




    }
    private static final long DEBOUNCE_DELAY = 200; // 200 milliseconds
    private boolean isButtonPressed(long currentTime, long lastPressedTime) {
        if (currentTime - lastPressedTime > DEBOUNCE_DELAY) {
            return true;
        }
        return false;
    }
}