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

    // Initial positions for OutputClaw and OutputWrist




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize hardware
        initializeHardware();

        // Now initialize Arm2 after hardware is set up
        Arm2 intake = new Arm2(hardwareMap, new ElapsedTime(), telemetry);
        IntakeClaw intakeclaw = new IntakeClaw(hardwareMap, "CLAW", "WRIST", "ROTATE");
        OutputArm output = new OutputArm(hardwareMap, "OUTPUTCLAW", "OUTPUTARM");
        wristPosition = 1-intakeWrist.getPosition();
        rotatePosition = intakeRotate.getPosition();
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
                }
            }
        });

        // Start the drivetrain thread
        drivetrainThread.start();

        // Continue with other operations, for example, controlling other components
        while (opModeIsActive()) {

            if (gamepad1.dpad_down && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)) {
                rotatePosition = Math.max(0, rotatePosition - 0.05);
                intakeclaw.setRotatePosition(rotatePosition);
                nonBlockingDelay(200);
            }
            if (gamepad1.dpad_up && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)) {
                rotatePosition = Math.min(1, rotatePosition + 0.05);
                intakeclaw.setRotatePosition(rotatePosition);
                nonBlockingDelay(200);
            }
            if (gamepad1.dpad_left && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)) {
                wristPosition = Math.max(0, wristPosition - 0.05);
                intakeclaw.setWristPosition(wristPosition);
                nonBlockingDelay(200);
            }
            if (gamepad1.dpad_right && isButtonPressed((long) runtime.milliseconds(), lastPressedTimeBumper)) {
                wristPosition = Math.min(1, wristPosition + 0.05);
                intakeclaw.setWristPosition(wristPosition);
                nonBlockingDelay(200);
            }
            if(gamepad2.dpad_up){
                intake.moveArmBy(10);
                nonBlockingDelay(100);
            }
            if(gamepad2.dpad_down ){
                intake.moveArmBy(-10);
                nonBlockingDelay(100);
            }

            if (gamepad1.a) {
                intakeclaw.close();

                nonBlockingDelay(200);
            } else if(gamepad1.y) {
                intakeclaw.open();
                nonBlockingDelay(200);
            }
            // OutputWrist control using gamepad2 left stick y
            double leftStickY = -gamepad2.left_stick_y; // Invert to match desired direction
            if (Math.abs(leftStickY) > 0.1) { // Deadzone to prevent accidental movement
                outputWristPosition += leftStickY * WRIST_INCREMENT;
                outputWristPosition = Range.clip(outputWristPosition, 0.0, 1.0);
                outputArm.setPosition(outputWristPosition);
                nonBlockingDelay(10);
            }
            if(gamepad2.a){
                output.OpenOutputClaw();
            }
            else if(gamepad2.y){
                output.CloseOutputClaw();
            }
            // You can add additional logic here if needed
            telemetry.update();
            intake.update();
        }

        // Wait for the drivetrain thread to finish before ending the opmode
        drivetrainThread.join();
    }

    private void driveControl() {
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




    private void initializeHardware() {
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");
        intakeRotate = hardwareMap.get(Servo.class, "ROTATE");
        intakeWrist = hardwareMap.get(Servo.class, "WRIST");
        outputClaw = hardwareMap.get(Servo.class, "OUTPUTCLAW");
        outputArm = hardwareMap.get(Servo.class, "OUTPUTARM");



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
            return true;
        }
        return false;
    }
}
