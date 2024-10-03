package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoMode", group="Linear Opmode")
public class AutoMode extends LinearOpMode {

    // Declare motors
    private DcMotor FrontLeftMotor = null;
    private DcMotor BackLeftMotor = null;
    private DcMotor FrontRightMotor = null;
    private DcMotor BackRightMotor = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Odometry variables
    private final double wheelDiameter = 4.0; // Wheel diameter in inches
    private final double encoderCountsPerRevolution = 1120; // REV 20 motor encoder counts
    private double posX = 0.0; // X position in inches
    private double posY = 0.0; // Y position in inches
    private double angle = 0.0; // Robot's orientation in degrees

    private volatile boolean odometryRunning = true;

    @Override
    public void runOpMode() {

        // Initialize the hardware (motors)
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");

        // Set motor directions
        FrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        BackRightMotor.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Reset encoders
        resetEncoders();

        // Start the odometry thread
        new Thread(new OdometerTask()).start();

        // Wait for the start button to be pressed
        waitForStart();
        runtime.reset();

        // Example movements using the new system
        moveToPosition(24, 12, 0.5);  // Move to (24, 24) inches
        moveToPosition(0, 24, 0.5);   // Move to (0, 24) inches
        moveToPosition(0, 0, 0.5);    // Move back to the origin (0, 0)

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // Stop the odometry thread when OpMode ends
        odometryRunning = false;
    }

    // Odometry-related methods

    private void resetEncoders() {
        FrontLeftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        FrontRightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BackLeftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BackRightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
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

    // Unified movement method for mecanum drive
    private void moveInDirection(double power, double angleInDegrees, long time) {
        double angleInRadians = Math.toRadians(angleInDegrees);

        // Calculate motor powers
        double frontLeftPower = power * (Math.sin(angleInRadians) + Math.cos(angleInRadians));
        double backLeftPower = power * (Math.sin(angleInRadians) - Math.cos(angleInRadians));
        double frontRightPower = power * (Math.sin(angleInRadians) - Math.cos(angleInRadians));
        double backRightPower = power * (Math.sin(angleInRadians) + Math.cos(angleInRadians));

        // Apply motor powers
        FrontLeftMotor.setPower(frontLeftPower);
        BackLeftMotor.setPower(backLeftPower);
        FrontRightMotor.setPower(frontRightPower);
        BackRightMotor.setPower(backRightPower);

        sleep(time);
        stopMotors();
    }

    // Navigate to a specific (x, y) coordinate
    private void moveToPosition(double targetX, double targetY, double power) {
        double deltaX = targetX - posX;
        double deltaY = targetY - posY;

        // Calculate the angle and distance to the target
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Move in the direction of the target
        moveInDirection(power, angleToTarget, (long)(distanceToTarget * 1000)); // Adjust timing based on your robot's speed

        // Update position
        posX = targetX;
        posY = targetY;
    }

    private void stopMotors() {
        FrontLeftMotor.setPower(0);
        BackLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackRightMotor.setPower(0);
    }
}
