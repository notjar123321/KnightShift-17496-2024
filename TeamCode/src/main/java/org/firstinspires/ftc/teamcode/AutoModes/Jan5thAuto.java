package org.firstinspires.ftc.teamcode.AutoModes;

import android.content.res.AssetManager;
import android.graphics.Bitmap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Classes.MecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


import java.util.ArrayList;
import java.util.List;


@Config
@Autonomous(name = "Jan5thAuto", group = "Autonomous")
public class Jan5thAuto extends LinearOpMode {
    private OpenCvWebcam webcam;
    private Servo OutputArmServo;
    private Servo OutputArmWrist;

    private void nonBlockingDelay(double milliseconds) {
        ElapsedTime delayTimer = new ElapsedTime();
        delayTimer.reset();
        while (opModeIsActive() && delayTimer.milliseconds() < milliseconds) {
            telemetry.update();
        }
    }
    public class Claw {
        // Servo instances
        private Servo leftServo;
        private Servo rightServo;

        // Positions for open and closed states
        private static final double LEFT_SERVO_OPEN = 0.0;
        private static final double LEFT_SERVO_CLOSED = 1.0;
        private static final double RIGHT_SERVO_OPEN = 1.0;
        private static final double RIGHT_SERVO_CLOSED = 0.0;

        // Constructor
        public Claw(HardwareMap hardwareMap, String leftServoName, String rightServoName) {
            leftServo = hardwareMap.get(Servo.class, leftServoName);
            rightServo = hardwareMap.get(Servo.class, rightServoName);
        }

        // Method to open the claw
        public void open() {
            leftServo.setPosition(LEFT_SERVO_OPEN);
            rightServo.setPosition(RIGHT_SERVO_OPEN);
        }

        // Method to close the claw
        public void close() {
            leftServo.setPosition(LEFT_SERVO_CLOSED);
            rightServo.setPosition(RIGHT_SERVO_CLOSED);
        }

        // Method to set custom positions for the servos
        public void setPositions(double leftPosition, double rightPosition) {
            leftServo.setPosition(leftPosition);
            rightServo.setPosition(rightPosition);
        }

        // Action to open the claw
        public class OpenAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftServo.setPosition(LEFT_SERVO_OPEN);
                rightServo.setPosition(RIGHT_SERVO_OPEN);
                return true; // Action completes immediately
            }
        }

        // Action to close the claw
        public class CloseAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftServo.setPosition(LEFT_SERVO_CLOSED);
                rightServo.setPosition(RIGHT_SERVO_CLOSED);
                return true; // Action completes immediately
            }
        }

        // Factory method to create an open action
        public Action openAction() {
            return new OpenAction();
        }

        // Factory method to create a close action
        public Action closeAction() {
            return new CloseAction();
        }
    }
    public class Arm2 {private DcMotor motor1;

        private Servo wrist1 = null; // First wrist servo
        private Servo wrist2 = null;


        private ElapsedTime timer;
        private Telemetry telemetry;

        public int target_position = 0; // Target position for the arm
        private double integralSum = 0; // Integral for PID control
        private double lastError = 0; // Last error value for PID
        private double lastTime = 0; // Last time update was called
        private double gravityCompensation = 0.0005; // Gravity compensation factor (adjust as needed)
        private double target_velocity = .05; // Target velocity for constant velocity control

        // PID Constants
        private double kP = 0.01; // Proportional constant
        private double kI = 0.001; // Integral constant
        private double kD = 0.001; // Derivative constant


        private double maxIntegral = .05; // Limit integral to prevent windup
        private double maxDerivative = 0.0003; // Limit derivative changes


        public Arm2(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
            motor1 = hardwareMap.get(DcMotor.class, "INTAKE");

            timer = elapsedTime;
            telemetry = telemetryIn;

            motor1.setDirection(DcMotor.Direction.REVERSE);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            wrist1 = hardwareMap.get(Servo.class, "INPUTLEFT");
            wrist2 = hardwareMap.get(Servo.class, "INPUTRIGHT");
            wrist2.setDirection(Servo.Direction.REVERSE);
        }

        public void moveToPosition(int ticks) {
            target_position = Range.clip(ticks, 0, 10000);
            motor1.setTargetPosition(target_position);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double liftPower = 1.0;
            motor1.setPower(liftPower);

            while (motor1.isBusy()) {
                // Optional: Add telemetry or additional logic here
            }

            motor1.setPower(0);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class MoveToPositionAction implements Action {
            private int targetPosition;

            public MoveToPositionAction(int position) {
                this.targetPosition = Range.clip(position, 0, 10000);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Define a small threshold for acceptable error
                final int positionalThreshold = 10;
                final double liftPower = 1.0;


                motor1.setTargetPosition(targetPosition);
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor1.setPower(liftPower);

                while (motor1.isBusy()) {
                    // Check if within the acceptable threshold
                    telemetry.addData("repeating", "repeating");
                    telemetry.update();

                    // Optional: PID adjustments or other logic can go here if needed
                    // You can add telemetry or further conditions here to monitor progress
                }

                motor1.setPower(0);  // Stop the motor
                motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Revert to encoder mode
                return true;  // Action is complete
            }

        }

                // Factory method to create a MoveToPositionAction
        public Action moveToPositionAction(int position) {
            return new MoveToPositionAction(position);
        }
        public void update() {

            // Get the current position of the arm
            double pos = (motor1.getCurrentPosition());
            double error = target_position - pos;
            if(pos<0)
            {
                target_position=0;
            }
            if(pos<200){
                wrist1.setPosition(0.1);
                wrist2.setPosition(0.1);
            }

            // Time elapsed for PID calculation
            double currentTime = timer.seconds();
            double deltaTime = currentTime - lastTime;

            // Proportional term
            double pTerm = kP * error;

            // Integral term with limit
            integralSum = Range.clip(integralSum + error * deltaTime, -maxIntegral, maxIntegral);
            double iTerm = kI * integralSum;

            // Derivative term with limit
            double deltaError = error - lastError;
            double dTerm = (deltaTime > 0) ? kD * Range.clip(deltaError / deltaTime, -maxDerivative, maxDerivative) : 0;

            // PID output with gravity compensation
            double pidOutput = pTerm + iTerm + dTerm;
            double motorPower = Range.clip(pidOutput + gravityCompensation * Math.signum(error), -0.5, 0.5);

            // Apply power to motors
            motor1.setPower(motorPower);


            // Update previous values for next loop
            lastError = error;
            lastTime = currentTime;
            sleep(5);
            telemetry.update();
        }
    }
    public class LinearSlide {
        private DcMotor LS1;
        private DcMotor LS2;
        private ElapsedTime timer;
        private Telemetry telemetry;

        public int target_position = 0;

        public LinearSlide(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
            target_position = 0;
            LS1 = hardwareMap.get(DcMotor.class, "LS1");
            LS2 = hardwareMap.get(DcMotor.class, "LS2");

            timer = elapsedTime;
            telemetry = telemetryIn;

            LS1.setDirection(DcMotor.Direction.FORWARD);
            LS2.setDirection(DcMotorSimple.Direction.REVERSE);
            LS1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LS1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LS2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LS2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void moveLSTo(int ticks) {
            target_position = ticks + 10;
            LS1.setTargetPosition(target_position);
            target_position = ticks;
            LS2.setTargetPosition(target_position);

            LS1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LS2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double maxPower = 1;
            while (LS1.isBusy() || LS2.isBusy()) {
                LS1.setPower(maxPower);
                LS2.setPower(maxPower * 0.75);
                telemetry.update();
            }
        }

        public void moveElbow(int ticks) {
            target_position += ticks;
            LS1.setTargetPosition(target_position);
            LS2.setTargetPosition(target_position);

            LS1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LS2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double maxPower = 1;
            while (LS1.isBusy()) {
                int currentPos = (LS1.getCurrentPosition());
                int distanceToTarget = Math.abs(target_position - currentPos);

                LS1.setPower(maxPower);
                LS2.setPower(maxPower * 0.75);
                telemetry.update();
            }
        }
        public class MoveToPositionAction implements Action {
            private int targetPosition;

            public MoveToPositionAction(int position) {
                this.targetPosition = Range.clip(position, 0, 10000);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                LS1.setTargetPosition(targetPosition);
                LS2.setTargetPosition(targetPosition);
                LS1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LS2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                double liftPower = 1.0;
                LS1.setPower(liftPower);
                LS1.setPower(liftPower);

                if (!LS1.isBusy()) {
                    LS1.setPower(0);
                    LS2.setPower(0);

                    return true; // Action is complete
                }

                return false; // Action is still running
            }
        }

        // Factory method to create a MoveToPositionAction
        public Action moveToPositionAction(int position) {
            return new MoveToPositionAction(position);
        }
    }
    static class CubeDetectionPipeline extends OpenCvPipeline {
        private Telemetry telemetry;
        private double focalLength = 460; // Replace with calibrated focal length
        private double realHeightTall = 8.9; // Real-world upright cube height in cm
        private double realHeightShort = 3.8; // Real-world sideways cube height in cm
        Mat hsv = new Mat();
        Mat maskYellow = new Mat();
        Mat maskBlue = new Mat();
        Mat maskRed = new Mat();
        Scalar yellowLower = new Scalar(10, 150, 100);
        Scalar yellowUpper = new Scalar(40, 255, 255);
        Scalar blueLower = new Scalar(90, 105, 50);
        Scalar blueUpper = new Scalar(140, 255, 255);
        Scalar redLower1 = new Scalar(0, 150, 120);
        Scalar redUpper1 = new Scalar(10, 255, 255);
        Scalar redLower2 = new Scalar(135, 135, 110);
        Scalar redUpper2 = new Scalar(180, 255, 255);
        Scalar avgColor = new Scalar(0, 0, 0);

        public CubeDetectionPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }
        public Scalar getAvgColor(){
            return avgColor;
        }

        @Override
        public Mat processFrame(Mat input) {
            // Convert to HSV for color detection
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Generate masks
            Core.inRange(hsv, yellowLower, yellowUpper, maskYellow);
            Core.inRange(hsv, blueLower, blueUpper, maskBlue);
            Core.inRange(hsv, redLower1, redUpper1, maskRed);
            Mat maskRed2 = new Mat();
            Core.inRange(hsv, redLower2, redUpper2, maskRed2);
            Core.bitwise_or(maskRed, maskRed2, maskRed);


            // Combine masks
            Mat combinedMask = new Mat();
            Core.bitwise_or(maskYellow, maskBlue, combinedMask);
            Core.bitwise_or(combinedMask, maskRed, combinedMask);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                // Filter out small boxes
                if (rotatedRect.size.width < 100 || rotatedRect.size.height < 100) continue;

                // Determine if upright or sideways
                boolean isTall = rotatedRect.size.height > rotatedRect.size.width;
                double realHeight = isTall ? realHeightTall : realHeightShort;

                // Calculate apparent height and distance
                double apparentHeight = Math.min(rotatedRect.size.width, rotatedRect.size.height);
                double distance = (focalLength * realHeight) / apparentHeight;

                // Check bounds before accessing submat
                Rect rect = rotatedRect.boundingRect();
                if (rect.x >= 0 && rect.y >= 0 && rect.x + rect.width <= hsv.cols() && rect.y + rect.height <= hsv.rows()) {
                    Mat cubeRegion = hsv.submat(rect);

                    // Determine average color
                    avgColor = Core.mean(cubeRegion);


                    telemetry.addData("Distance (cm)", distance);
                    telemetry.update();
                    String color = "Unknown";
                    if (avgColor.val[0] >= yellowLower.val[0] && avgColor.val[0] <= yellowUpper.val[0]) color = "Yellow";
                    else if (avgColor.val[0] >= blueLower.val[0] && avgColor.val[0] <= blueUpper.val[0]) color = "Blue";
                    else if ((avgColor.val[0] >= redLower1.val[0] && avgColor.val[0] <= redUpper1.val[0])
                            || (avgColor.val[0] >= redLower2.val[0] && avgColor.val[0] <= redUpper2.val[0])) color = "Red";

                    // Add telemetry


                    // Draw rotated rectangle
                    Point[] vertices = new Point[4];
                    rotatedRect.points(vertices);
                    for (int j = 0; j < 4; j++) {
                        Imgproc.line(input, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 255, 0), 2);
                    }

                    // Draw label
                    Imgproc.putText(input, color + String.format(": %.2f cm", distance, avgColor),
                            new Point(rotatedRect.center.x, rotatedRect.center.y - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 1.5, new Scalar(0, 255, 0), 2);


                    cubeRegion.release();
                }
            }

            // Release temporary Mats
            maskRed2.release();
            combinedMask.release();
            hierarchy.release();

            return input;
        }
    }
    // Helper method to create an Action for servo movement
    public class wrist{
        private Servo wrist1;
        public wrist(HardwareMap hardwareMap, String ServoName){
            wrist1 = hardwareMap.get(Servo.class, ServoName);
        }
        public class SetPositionAction implements Action {
            private double pos;
            public SetPositionAction(double position){
                pos = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist1.setPosition(pos);
                return true; // Action completes immediately
            }
        }
        public Action setPositionAction(double position){
            return new SetPositionAction(position);
        }
    }
    private Action moveServoAction(Servo servo, double targetPosition) {
        return packet -> {
            servo.setPosition(targetPosition);
            telemetry.addData("Servo", "Moved to position: " + targetPosition);
            telemetry.update();
            return true; // Immediate action completion
        };
    }

    private Action waitAction(long milliseconds) {
        return new Action() {
            private long startTime = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime == -1) startTime = System.currentTimeMillis();
                if (System.currentTimeMillis() - startTime >= milliseconds) {
                    telemetry.addData("Wait", "Completed after " + milliseconds + "ms");
                    telemetry.update();
                    return true; // Action completed
                }
                return false; // Keep waiting
            }
        };
    }







    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        LinearSlide LS = new LinearSlide(hardwareMap, new ElapsedTime(), telemetry);
        Arm2 intake = new Arm2(hardwareMap, new ElapsedTime(), telemetry);
        Claw intakeclaw = new Claw(hardwareMap, "CLAWLEFT", "CLAWRIGHT");
        Claw outputclaw = new Claw(hardwareMap, "OUTPUTCLAWLEFT", "OUTPUTCLAWRIGHT");
        OutputArmServo = hardwareMap.get(Servo.class, "OUTPUTARM");
        OutputArmWrist = hardwareMap.get(Servo.class, "OUTPUTWRIST");
        wrist OutputArm = new wrist(hardwareMap, "OUTPUTARM");
        wrist OutputArmWrist = new wrist(hardwareMap, "OUTPUTWRIST");
        wrist intakeWrist1 = new wrist(hardwareMap, "INPUTLEFT");
        wrist intakeWrist2 = new wrist(hardwareMap, "INPUTRIGHT");


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-32, 0));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-20, 2, Math.toRadians(0)))
                .strafeTo(new Vector2d(0, 0));
                //.turn(Math.toRadians(45));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-20, 2, Math.toRadians(45+180)))
                .strafeTo(new Vector2d(-15, 14))
                .turn(Math.toRadians(45));
        TrajectoryActionBuilder MoveToBucket = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .strafeTo(new Vector2d(-14, -43))
                .turnTo(Math.toRadians(-45));


        Action trajectoryActionCloseOut = MoveToBucket.fresh()
                .strafeTo(new Vector2d(-14, -43))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.

        intakeclaw.close();
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();
        intake.update();

        Actions.runBlocking(
                new SequentialAction(
                        MoveToBucket.build(), // Move robot to the bucket
                        intake.moveToPositionAction(400), // Extend the arm
                        OutputArm.setPositionAction(1), // Move the output arm
                        OutputArmWrist.setPositionAction(0.7), // Set wrist position
                        intake.moveToPositionAction(200),
                        intakeWrist1.setPositionAction(0.5), // Adjust wrist1
                        intakeWrist2.setPositionAction(0.5), // Adjust wrist2
                        intakeclaw.openAction(), // Open intake claw
                        intake.moveToPositionAction(200),
                        intakeWrist1.setPositionAction(0.4), // Adjust wrist1
                        intakeWrist2.setPositionAction(0.4), // Adjust wrist2

                        intake.moveToPositionAction(223),
                        intakeWrist1.setPositionAction(0.1), // Final wrist1 adjustment
                        intakeWrist2.setPositionAction(0.1), // Final wrist2 adjustment

                        intake.moveToPositionAction(350),

                        LS.moveToPositionAction(3350),
                        intake.moveToPositionAction(223),
                        OutputArm.setPositionAction(0.85), // Rotate output arm servo
                        OutputArmWrist.setPositionAction(0.8), // Rotate wrist servo
                        OutputArmWrist.setPositionAction(0.0), // Reset wrist position
                        intakeclaw.openAction(), // Open the claw
                        OutputArm.setPositionAction(0.0), // Reset arm servo
                        LS.moveToPositionAction(0), // Reset linear slide
                        intake.moveToPositionAction(0), // Reset intake
                        intakeclaw.closeAction() // Close the claw
                )
        );

    }
}
