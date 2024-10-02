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
    private double distanceTravelled = 0.0; // in inches

    // Current position variables
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

        // Autonomous movements with telemetry updates
        moveForward(0.5, 1000);
        updateTelemetry();
        strafeRight(0.5, 1000);
        updateTelemetry();
        turnByDegrees(90, 0.7);  // Example of turning by 90 degrees
        updateTelemetry();
        moveBackward(0.5, 1000);
        updateTelemetry();

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // Stop the odometry thread when OpMode ends
        odometryRunning = false;
    }

    // Odometry-related methods

    private void resetEncoders() {
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    // Movement methods for mecanum drive

    private void moveForward(double power, long time) {
        FrontLeftMotor.setPower(power);
        BackLeftMotor.setPower(power);
        FrontRightMotor.setPower(power);
        BackRightMotor.setPower(power);
        sleep(time);
        stopMotors();
    }

    private void moveBackward(double power, long time) {
        FrontLeftMotor.setPower(-power);
        BackLeftMotor.setPower(-power);
        FrontRightMotor.setPower(-power);
        BackRightMotor.setPower(-power);
        sleep(time);
        stopMotors();
    }

    private void strafeRight(double power, long time) {
        FrontLeftMotor.setPower(power);
        BackLeftMotor.setPower(-power);
        FrontRightMotor.setPower(-power);
        BackRightMotor.setPower(power);
        sleep(time);
        stopMotors();
    }

    private void strafeLeft(double power, long time) {
        FrontLeftMotor.setPower(-power);
        BackLeftMotor.setPower(power);
        FrontRightMotor.setPower(power);
        BackRightMotor.setPower(-power);
        sleep(time);
        stopMotors();
    }

    // New method to turn the robot by a specific degree based on encoders
    private void turnByDegrees(double degrees, double power) {
        // Determine the direction of the turn
        boolean turnRight = degrees > 0;

        // Calculate the total angle change needed and the corresponding encoder counts
        double targetAngleChange = Math.abs(degrees);  // The absolute number of degrees to turn
        double initialAngle = angle;  // Store the initial angle
        double targetAngle = initialAngle + degrees;  // The angle we're turning towards

        // Set a tolerance for the turn
        double angleTolerance = 1;  // 1.5 degree tolerance for precision

        // Reset encoders to track turning more accurately
        resetEncoders();

        // Turning loop
        while (opModeIsActive() && Math.abs(angle - targetAngle) < angleTolerance) {
            // If turning right
            if (turnRight) {
                FrontLeftMotor.setPower(power);
                BackLeftMotor.setPower(power);
                FrontRightMotor.setPower(-power);
                BackRightMotor.setPower(-power);
            } else { // If turning left
                FrontLeftMotor.setPower(-power);
                BackLeftMotor.setPower(-power);
                FrontRightMotor.setPower(power);
                BackRightMotor.setPower(power);
            }

            // Update telemetry with current angle
            telemetry.addData("Turning", "Target Angle: %.2f, Current Angle: %.2f", targetAngle, angle);
            telemetry.update();
        }

        // Stop the motors after the turn is complete
        stopMotors();
    }



    private void stopMotors() {
        FrontLeftMotor.setPower(0);
        BackLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackRightMotor.setPower(0);
    }

    // Update telemetry with odometry data
    private void updateTelemetry() {
        telemetry.addData("X Position (inches)", posX);
        telemetry.addData("Y Position (inches)", posY);
        telemetry.addData("Orientation (degrees)", angle);
        telemetry.update();
    }
}
