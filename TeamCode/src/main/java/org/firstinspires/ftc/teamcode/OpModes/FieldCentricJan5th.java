package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Classes.MecanumDrive.PARAMS;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Classes.Arm2;
import org.firstinspires.ftc.teamcode.Classes.IntakeClaw;
import org.firstinspires.ftc.teamcode.Classes.LinearSlide;

@TeleOp(name = "Jan5th-Fieldcentric test", group = "Linear Opmode")
public class FieldCentricJan5th extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor ls1 = null;
    private DcMotor ls2 = null;
    private double y;
    private double x;
    private double rx;
    private double sens = 1;
    boolean isClawOpen = false;
    boolean isOutputClawOpen = false;
    private DcMotor FrontLeftMotor = null;
    private DcMotor FrontRightMotor = null;
    private DcMotor BackLeftMotor = null;
    private DcMotor BackRightMotor = null;
    private DcMotor IntakeMotor = null;
    private Servo OutputArmServo = null;
    private Servo INPUTLEFT = null;
    private Servo INPUTRIGHT = null;
    private Servo CLAWLEFT = null;
    private Servo CLAWRIGHT = null;
    private DcMotorSimple OutputArmWrist = null;
    private double InputWristPosition;
    private double OutputArmPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeHardware();
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        imu.initialize(parameters);

        waitForStart();
        runtime.reset();
        LinearSlide LS = new LinearSlide(hardwareMap, new ElapsedTime(), telemetry);
        Arm2 intake = new Arm2(hardwareMap, new ElapsedTime(), telemetry);
        IntakeClaw intakeclaw = new IntakeClaw(hardwareMap, "CLAWLEFT", "CLAWRIGHT");

        while (opModeIsActive()) {
            // Get joystick inputs
            double y = -gamepad1.left_stick_y; // Inverted for correct direction
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Get the robot's heading
            double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Adjust joystick inputs for field-centric control
            double adjustedX = x * Math.cos(robotAngle) - y * Math.sin(robotAngle);
            double adjustedY = x * Math.sin(robotAngle) + y * Math.cos(robotAngle);

            // Calculate motor powers
            double denominator = Math.max(Math.abs(adjustedY) + Math.abs(adjustedX) + Math.abs(rx), 1);
            double frontLeftPower = (adjustedY + adjustedX + rx) / denominator;
            double backLeftPower = (adjustedY - adjustedX + rx) / denominator;
            double frontRightPower = (adjustedY - adjustedX - rx) / denominator;
            double backRightPower = (adjustedY + adjustedX - rx) / denominator;

            // Set motor powers
            FrontLeftMotor.setPower(-frontLeftPower * sens);
            BackLeftMotor.setPower(-backLeftPower * sens);
            FrontRightMotor.setPower(frontRightPower * sens);
            BackRightMotor.setPower(backRightPower * sens);

            // Linear Slide Commands (Dpad)
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
            if (gamepad2.left_stick_y != 0) {
                intake.moveElbow((int) (gamepad2.left_stick_y * 30));
            }
            if (gamepad2.right_stick_y != 0) {
                OutputArmPosition = OutputArmServo.getPosition();
                OutputArmServo.setPosition(OutputArmPosition + .005 * gamepad2.right_stick_y);
                nonBlockingDelay(10);
            } else {
                OutputArmPosition = OutputArmServo.getPosition();
                OutputArmServo.setPosition(OutputArmPosition);
                nonBlockingDelay(10);
            }
            if (gamepad2.x) {
                isClawOpen = !isClawOpen; // Toggle state
                if (isClawOpen) {
                    intakeclaw.open(); // Open claw
                } else {
                    intakeclaw.close(); // Close claw
                }
                sleep(60);
            }

            if (gamepad2.right_bumper) {
                InputWristPosition = INPUTLEFT.getPosition();
                INPUTLEFT.setPosition(Range.clip(InputWristPosition + 0.1, 0, 1)); // Clamp between 0 and 1
                INPUTRIGHT.setPosition(Range.clip(InputWristPosition + 0.1, 0, 1)); // Clamp between 0 and 1
                nonBlockingDelay(200);
            }
            if (gamepad2.left_bumper) {
                InputWristPosition = INPUTLEFT.getPosition();
                INPUTLEFT.setPosition(Range.clip(InputWristPosition - 0.1, 0, 1)); // Clamp between 0 and 1
                INPUTRIGHT.setPosition(Range.clip(InputWristPosition - 0.1, 0, 1)); // Clamp between 0 and 1
                nonBlockingDelay(200);
            }

            telemetry.addData("Right stick y", gamepad2.right_stick_y);
            telemetry.addData("LS1 position", ls1.getCurrentPosition());
            telemetry.addData("LS2 position", ls2.getCurrentPosition());
            telemetry.addData("Intake Position", IntakeMotor.getCurrentPosition());
            telemetry.addData("INPUTLEFT", INPUTLEFT.getPosition());
            telemetry.addData("INPUTRIGHT", INPUTRIGHT.getPosition());
            telemetry.addData("Output Arm Servo", OutputArmServo.getPosition());
            telemetry.addData("Claw pos 1", CLAWLEFT.getPosition());
            telemetry.addData("Claw pos 2", CLAWRIGHT.getPosition());
            telemetry.addData("Gampad2 Left Stick", gamepad2.left_stick_y);
            telemetry.addData("angle", Math.toDegrees(robotAngle));

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
        IntakeClaw intakeClaw = new IntakeClaw(hardwareMap, "CLAWLEFT", "CLAWRIGHT");

        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");

        IntakeMotor = hardwareMap.get(DcMotor.class, "INTAKE");
        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Claws
        CLAWLEFT = hardwareMap.get(Servo.class, "CLAWLEFT");
        CLAWRIGHT = hardwareMap.get(Servo.class, "CLAWRIGHT");

        // Wrists
        INPUTLEFT = hardwareMap.get(Servo.class, "INPUTLEFT");
        INPUTRIGHT = hardwareMap.get(Servo.class, "OUTPUTRIGHT"); // Remember to change in config
        INPUTRIGHT.setDirection(Servo.Direction.REVERSE);

        OutputArmServo = hardwareMap.get(Servo.class, "OUTPUTARM");
        OutputArmWrist = hardwareMap.get(DcMotorSimple.class, "OUTPUTWRIST");
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
