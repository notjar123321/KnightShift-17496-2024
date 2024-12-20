package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Classes.Arm;
import org.firstinspires.ftc.teamcode.Classes.ScissorLift;

@TeleOp(name="no", group="Linear OpMode")
public class BasicOpMode_Linear extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeftMotor = null;
    private DcMotor BackLeftMotor = null;
    private DcMotor FrontRightMotor = null;
    private DcMotor BackRightMotor = null;
    private DcMotor armMotor1 = null; // First motor for arm rotation
    private DcMotor armMotor2 = null;
    private DcMotor SC1 = null; // First motor for scissor lift
    private DcMotor SC2 = null;
    private Servo wrist1 = null; // First wrist servo
    private Servo wrist2 = null;
    private DcMotorSimple wrist3 = null;

    // Odometer variables
    private final double wheelDiameter = 4.0; // Wheel diameter in inches
    private final double encoderCountsPerRevolution = 1120; // REV 20 motor encoder counts
    private double distanceTravelled = 0.0; // in inches
    private double wristPower = 0.0;

    // Current position variables
    private double posX = 0.0; // X position in inches
    private double posY = 0.0; // Y position in inches
    private double angle = 0.0; // Robot's orientation in degrees
    private boolean scissorMode = false;

    // Scissor Lift positions
    public final int upPosition = 3100;
    public final int downPosition = 0;

    // Wrist position
    public double wristPosition = 0;
    public double clawPosition = 0;

    // Sensitivity
    private double sens = .7;

    private int currentPos;
    private int scissorLiftPosition;

    private volatile boolean odometryRunning = true;
    private boolean armControlMode = false;
    private boolean isSCup = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");
        armMotor1 = hardwareMap.get(DcMotor.class, "CLAW1"); // First arm motor
        armMotor2 = hardwareMap.get(DcMotor.class, "CLAW2"); // second arm motor
        SC1 = hardwareMap.get(DcMotor.class, "Scissor1");
        SC2 = hardwareMap.get(DcMotor.class, "Scissor2");
        wrist1 = hardwareMap.get(Servo.class, "wrist1");
        wrist2 = hardwareMap.get(Servo.class, "wrist2");
        wrist3 = hardwareMap.get(DcMotorSimple.class, "wrist3");

        // Set motor directions
        FrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        BackRightMotor.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor1.setDirection(DcMotor.Direction.FORWARD);
        armMotor2.setDirection(DcMotor.Direction.REVERSE);
        wrist1.setDirection(Servo.Direction.REVERSE);
        wrist2.setDirection(Servo.Direction.FORWARD);

        int groundPosition = 0;
        int midPosition = 560;
        int highPosition = 780;

        // Default target position

        // Reset encoders
        resetEncoders();
        Arm arm = new Arm(hardwareMap, runtime, telemetry);
        ScissorLift scissor1 = new ScissorLift(hardwareMap, runtime, telemetry);
        ScissorLift scissor2 = new ScissorLift(hardwareMap, runtime, telemetry);
        // Start the odometry thread
        new Thread(new OdometerTask()).start();

        // Wait for the game to start
        waitForStart();
        runtime.reset();
        wristPosition = (wrist1.getPosition() + wrist2.getPosition()) / 2;

        // Run until the end of the match
        while (opModeIsActive()) {
            // Drive control variables
            arm.update();
            SC1.setPower(gamepad1.right_stick_y);
            if (gamepad1.a) {
                armControlMode = !armControlMode; // Toggle control mode
                sleep(200); // Debounce delay
                telemetry.addLine("arm is on");
            }
            if (gamepad1.x) {
                scissorMode = !scissorMode;  // Toggle the scissor mode
                sleep(200);  // Add a small debounce delay
                telemetry.addLine("scissor is on");
            }
            if(scissorMode)
            {

                if (gamepad1.left_stick_y != 0) {
                    scissorLiftPosition += (int)(gamepad1.left_stick_y * 50);  // Adjust step size as needed
                } else if (gamepad1.right_stick_y != 0) {
                    scissorLiftPosition += (int)(gamepad1.right_stick_y * 50);  // Adjust step size as needed
                }

                // Ensure the scissor lift position stays within the bounds
                scissorLiftPosition = Math.max(0, Math.min(scissorLiftPosition, 3100));

                // Move scissor lift motors to the desired position
                SC1.setTargetPosition(scissorLiftPosition);
                SC2.setTargetPosition(scissorLiftPosition);
                SC1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SC2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SC1.setPower(0.8);  // Adjust power as necessary for smooth operation
                SC2.setPower(0.8);
            }
            if (!armControlMode) {
                double leftPower;
                double rightPower;

                // POV Mode control
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
                FrontLeftMotor.setPower(frontLeftPower * sens);
                BackLeftMotor.setPower(backLeftPower * sens);
                FrontRightMotor.setPower(frontRightPower * sens);
                BackRightMotor.setPower(backRightPower * sens);

                // Show the elapsed game time and odometer data
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Distance Travelled (in)", distanceTravelled);
                telemetry.addData("Current Position (X,Y)", String.format("X: %.2f, Y: %.2f", posX, posY));
                telemetry.addData("Orientation", String.format("Angle: %.2f", angle));
                telemetry.addData("Arm Position Motor1", armMotor1.getCurrentPosition());
                telemetry.addData("Arm Position Motor2", armMotor2.getCurrentPosition());
                telemetry.addData("Target Position", Arm.target_position);
                telemetry.addData("Scissor Lift Position SC1", SC1.getCurrentPosition());
                telemetry.addData("Scissor Lift Position SC2", SC2.getCurrentPosition());
                telemetry.addData("Wrist1 Position", wrist1.getPosition());
                telemetry.addData("Wrist2 Position", wrist2.getPosition());
                telemetry.update();
                arm.update();
            }

            if (armControlMode) {
                arm.update();
                wristPower = wristPower + gamepad1.right_stick_x;
                wrist3.setPower(wristPower); // Set motor power based on joystick input

                if (gamepad1.dpad_down) {
                    arm.moveElbow(-5);
                    sleep(10);
                } else if (gamepad1.dpad_left) {
                    arm.moveElbowTo(midPosition);

                    sleep(10);
                } else if (gamepad1.dpad_right) {
                    arm.moveElbowTo(highPosition);

                    sleep(10);
                } else if (gamepad1.dpad_up) {
                    arm.moveElbow(5);

                    sleep(10);
                }

                if (gamepad1.right_bumper) {
                    // Move wrist up (adjust the position as needed)
                    wristPosition = wrist1.getPosition();
                    wristPosition += .05;
                    wrist1.setPosition(wristPosition);
                    wrist2.setPosition(wristPosition);
                    sleep(10);
                } else if (gamepad1.left_bumper) {
                    // Move wrist down (adjust the position as needed)
                    wristPosition = wrist1.getPosition();
                    wristPosition -= .05;
                    wrist1.setPosition(wristPosition);
                    wrist2.setPosition(wristPosition);
                    sleep(10);
                }
                arm.update();

                // Add a short delay to prevent multiple toggles from one press
                sleep(50);

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Distance Travelled (in)", distanceTravelled);
                telemetry.addData("Current Position (X,Y)", String.format("X: %.2f, Y: %.2f", posX, posY));
                telemetry.addData("Orientation", String.format("Angle: %.2f", angle));
                telemetry.addData("Arm Position Motor1", armMotor1.getCurrentPosition());
                telemetry.addData("Arm Position Motor2", armMotor2.getCurrentPosition());
                telemetry.addData("Target Position", Arm.target_position);
                telemetry.addData("Scissor Lift Position SC1", SC1.getCurrentPosition());
                telemetry.addData("Scissor Lift Position SC2", SC2.getCurrentPosition());
                telemetry.addData("Wrist1 Position", wrist1.getPosition());
                telemetry.addData("Wrist2 Position", wrist2.getPosition());
                telemetry.addData("Wrist3 Power", wristPower);
                telemetry.update();
                arm.update();
            }

            telemetry.update();
        }

        // Stop the odometry thread when OpMode ends
        odometryRunning = false;
        arm.update();
    }

    private void resetEncoders() {
        FrontLeftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        FrontRightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BackLeftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BackRightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armMotor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armMotor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        SC1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        SC2.setMode(DcMotor.RunMode.RESET_ENCODERS);

        sleep(100); // Allow time for the reset to take effect
        setMotorRunWithoutEncoder();
    }

    private void setMotorRunWithoutEncoder() {
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private class OdometerTask implements Runnable {
        @Override
        public void run() {
            int previousFrontLeftCounts = FrontLeftMotor.getCurrentPosition();
            int previousFrontRightCounts = FrontRightMotor.getCurrentPosition();
            int previousBackLeftCounts = BackLeftMotor.getCurrentPosition();
            int previousBackRightCounts = BackRightMotor.getCurrentPosition();

            while (odometryRunning) {
                // Get current encoder counts
                int currentFrontLeftCounts = FrontLeftMotor.getCurrentPosition();
                int currentFrontRightCounts = FrontRightMotor.getCurrentPosition();
                int currentBackLeftCounts = BackLeftMotor.getCurrentPosition();
                int currentBackRightCounts = BackRightMotor.getCurrentPosition();

                // Calculate the change in counts
                int deltaFrontLeftCounts = currentFrontLeftCounts - previousFrontLeftCounts;
                int deltaFrontRightCounts = currentFrontRightCounts - previousFrontRightCounts;
                int deltaBackLeftCounts = currentBackLeftCounts - previousBackLeftCounts;
                int deltaBackRightCounts = currentBackRightCounts - previousBackRightCounts;

                // Calculate distance traveled by each wheel
                double distanceFrontLeft = (deltaFrontLeftCounts / encoderCountsPerRevolution) * (Math.PI * wheelDiameter);
                double distanceFrontRight = (deltaFrontRightCounts / encoderCountsPerRevolution) * (Math.PI * wheelDiameter);
                double distanceBackLeft = (deltaBackLeftCounts / encoderCountsPerRevolution) * (Math.PI * wheelDiameter);
                double distanceBackRight = (deltaBackRightCounts / encoderCountsPerRevolution) * (Math.PI * wheelDiameter);

                // Average the distances
                double averageDistance = (distanceFrontLeft + distanceFrontRight + distanceBackLeft + distanceBackRight) / 4.0;

                // Update distance traveled
                distanceTravelled += averageDistance;

                // Calculate change in position
                double deltaX = averageDistance * Math.cos(Math.toRadians(angle));
                double deltaY = averageDistance * Math.sin(Math.toRadians(angle));

                // Update current position
                posX += deltaX;
                posY += deltaY;

                // Update previous counts
                previousFrontLeftCounts = currentFrontLeftCounts;
                previousFrontRightCounts = currentFrontRightCounts;
                previousBackLeftCounts = currentBackLeftCounts;
                previousBackRightCounts = currentBackRightCounts;

                // Sleep for a short period to avoid overwhelming the CPU
                try {
                    Thread.sleep(50); // Adjust this value as needed
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }
}
