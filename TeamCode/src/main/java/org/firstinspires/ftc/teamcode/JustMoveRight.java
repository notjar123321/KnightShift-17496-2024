package org.firstinspires.ftc.teamcode;

import android.content.res.AssetManager;
import android.graphics.Bitmap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
import org.openftc.easyopencv.OpenCvWebcam;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.support.image.ImageProcessor;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.support.image.ops.ResizeOp;


@Config
@Autonomous(name = "JustMoveRight", group = "Autonomous")
public class JustMoveRight extends LinearOpMode {
    private OpenCvWebcam webcam;

    public class Claw {
        private DcMotorSimple wrist3;

        public Claw(HardwareMap hardwareMap) {
            wrist3 = hardwareMap.get(DcMotorSimple.class, "wrist2");
        }

        public void CloseClaw(){
            wrist3.setPower(-.5);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sleep(200);

                wrist3.setPower(.2);
                sleep(340);
                wrist3.setPower(0);
                // Set motor power based on joystick input
                sleep(40);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
        public void OpenClaw() {
            sleep(200);

            wrist3.setPower(-.2); // Set motor power based on joystick input
            sleep(340);
            wrist3.setPower(-.1);
            sleep(40);
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sleep(200);

                wrist3.setPower(-.2); // Set motor power based on joystick input
                sleep(340);
                wrist3.setPower(-.1);
                sleep(40);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }
    public class SCLift {
        private DcMotor SC1 = null;
        private DcMotor SC2 = null;
        int TARGET_POSITION;
        public SCLift(HardwareMap hardwareMap) {
            SC1 = hardwareMap.get(DcMotor.class, "Scissor1");
            SC2 = hardwareMap.get(DcMotor.class, "Scissor2");
            SC1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SC1.setDirection(DcMotorSimple.Direction.FORWARD);
            SC2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SC2.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public void moveToPosition(int position) {
            TARGET_POSITION=Range.clip(position, 0, 3100);
            SC1.setTargetPosition(TARGET_POSITION);
            SC2.setTargetPosition(TARGET_POSITION);

            SC1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SC2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set power for lift movement
            double liftPower = 1; // Adjust as necessary for your setup
            SC1.setPower(liftPower);
            SC2.setPower(liftPower);

            // Wait until both motors reach the target
            while (SC1.isBusy() && SC2.isBusy()) {
                // Optionally add telemetry here for debugging
            }

            // Stop motors after reaching position
            SC1.setPower(0);
            SC2.setPower(0);

            // Switch back to RUN_USING_ENCODER to maintain the position
            SC1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SC2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public class MoveToPositionAction implements Action {
            private int targetPosition;

            public MoveToPositionAction(int position) {
                this.targetPosition = Range.clip(position, 0, 3100);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SC1.setTargetPosition(targetPosition);
                SC2.setTargetPosition(targetPosition);

                SC1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SC2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                double liftPower = 1.0;
                SC1.setPower(liftPower);
                SC2.setPower(liftPower);

                // Stop if both motors have reached the target
                if (!SC1.isBusy() && !SC2.isBusy()) {
                    SC1.setPower(0);
                    SC2.setPower(0);

                    SC1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    SC2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    return true; // Action is complete
                }

                return false; // Action is still running
            }
        }

        // Method to get a MoveToPositionAction with the specified target position
        public Action moveToPositionAction(int position) {
            return new MoveToPositionAction(position);
        }

    }
    public class Arm2 {
        private DcMotor motor1;

        private Servo wrist1 = null; // First wrist servo


        private DcMotorSimple wrist3 = null;
        private ElapsedTime timer;
        private Telemetry telemetry;

        public int target_position = 0;
        private double integralSum = 0;
        private double lastError = 0;
        private double lastTime = 0;
        private double gravityCompensation = 0.04; // Tune as needed

        // PID Constants
        private double kP = 0.1;
        private double kI = 0.01;
        private double kD = 0.01;
        private double maxIntegral = 10; // Limit integral to prevent windup
        private double maxDerivative = 0.1; // Limit derivative changes

        public Arm2(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
            motor1 = hardwareMap.get(DcMotor.class, RobotConstants.arm1);

            wrist1 = hardwareMap.get(Servo.class, "wrist1");

            wrist3 = hardwareMap.get(DcMotorSimple.class, "wrist3");
            timer = elapsedTime;
            telemetry = telemetryIn;

            motor1.setDirection(DcMotor.Direction.FORWARD);

        }

        public void update() {
            if(motor1.getCurrentPosition() < 300) {
                wrist1.setPosition(.95);

            }
            // Get the current position of the arm
            double pos = (motor1.getCurrentPosition());
            double error = target_position - pos;

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
        }

        public void moveElbowTo(int ticks) {
            target_position = ticks;
            motor1.setTargetPosition(target_position);


            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            double currentPower = 0.1; // Initial low power
            double maxPower = 0.5; // Maximum allowable power

            while (motor1.isBusy()) {
                int currentPos = (motor1.getCurrentPosition());
                int distanceToTarget = Math.abs(target_position - currentPos);

                // Gradual power increase
                currentPower = Range.clip(currentPower + 0.01, 0.1, maxPower);
                if (distanceToTarget < 50) {
                    currentPower *= 0.5; // Slow down near target
                }

                motor1.setPower(currentPower);

            }
        }
        public void moveElbow(int ticks) {

            target_position += ticks;
            motor1.setTargetPosition(target_position);


            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            double currentPower = 0.2; // Initial low power
            double maxPower = 0.5; // Maximum allowable power

            while (motor1.isBusy() ) {
                int currentPos = (motor1.getCurrentPosition());
                int distanceToTarget = Math.abs(target_position - currentPos);

                // Gradual power increase
                currentPower = Range.clip(currentPower + 0.01, 0.1, maxPower);
                if (distanceToTarget < 50) {
                    currentPower *= 0.5; // Slow down near target
                }

                motor1.setPower(currentPower);

            }
        }

        public void moveElbowSmoothly(int targetPosition) {
            target_position = targetPosition;

            int error = targetPosition - targetPosition;

            // Calculate the number of steps for smooth movement
            int steps = 20;
            int stepSize = error / steps;
            stepSize = (stepSize == 0) ? (error > 0 ? 1 : -1) : stepSize;

            for (int i = 0; i < steps; i++) {
                int partialTarget = targetPosition + stepSize * (i + 1);

                // Move arm gradually towards the target
                motor1.setTargetPosition(partialTarget);


                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                motor1.setPower(0.3);


                // Delay to control smoothness
                sleep(10);
            }

            // Final adjustment at lower power for precision
            motor1.setPower(0.5);
        }


        public void setTargetPosition(int position) {
            target_position = position;
        }
    }
    public class Bucket {
        private Servo bucketServo;      //may be DCmotorSimple
        private static final double UP_POSITION = 1.0; // Max position for the servo (may have to flip depending on how built
        private static final double DOWN_POSITION = 0.0;
        public Bucket(HardwareMap hardwareMap) {
            bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        }

        public void tiltBucketToMax() {
            bucketServo.setPosition(DOWN_POSITION);
        }

        // Action version for SequentialAction
        public class TiltBucketToMaxAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketServo.setPosition(DOWN_POSITION);
                return true; // Return true when the action is complete
            }
        }

        public Action tiltBucketToMaxAction() {
            return new TiltBucketToMaxAction();
        }

        // Regular method to untlit the bucket
        public void UntiltBucket() {
            bucketServo.setPosition(UP_POSITION);
        }

        // Action version for SequentialAction
        public class UntiltBucketAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketServo.setPosition(UP_POSITION);
                return true; // Return true when the action is complete
            }
        }

        public Action unTiltBucketAction() {
            return new UntiltBucketAction();
        }

    }
    public class SampleDetection {
        private Interpreter tflite;

        // Load the model in the constructor or initialization function
        public SampleDetection(AssetManager assetManager, String modelPath) {
            SampleDetection detection = new SampleDetection(assetManager, "model.tflite");

        }
        public TensorImage preprocessImage(Bitmap bitmap) {
            TensorImage tensorImage = new TensorImage(DataType.UINT8);

            // Load image into TensorImage
            tensorImage.load(bitmap);

            // Resize to model input size, e.g., 300x300
            ImageProcessor imageProcessor = new ImageProcessor.Builder()
                    .add(new ResizeOp(300, 300, ResizeOp.ResizeMethod.BILINEAR))
                    .build();
            return imageProcessor.process(tensorImage);
        }


        // Close the interpreter when done
        public void close() {
            if (tflite != null) {
                tflite.close();
                tflite = null;
            }
        }
    }
    public TensorImage preprocessImage(Bitmap bitmap) {
        TensorImage tensorImage = new TensorImage(DataType.UINT8);

        // Load image into TensorImage
        tensorImage.load(bitmap);

        // Resize to model input size, e.g., 300x300
        ImageProcessor imageProcessor = new ImageProcessor.Builder()
                .add(new ResizeOp(300, 300, ResizeOp.ResizeMethod.BILINEAR))
                .build();
        return imageProcessor.process(tensorImage);
    }





    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Claw claw = new Claw(hardwareMap);
        SCLift scLift = new SCLift(hardwareMap);
        Arm2 arm = new Arm2(hardwareMap, new ElapsedTime(), telemetry);



        //Bucket bucket = new Bucket(hardwareMap);







        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(45, 0));

//        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-20, 2, Math.toRadians(0)))
//                .strafeTo(new Vector2d(0, 0));
//                //.turn(Math.toRadians(45));
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-20, 2, Math.toRadians(45+180)))
//                .strafeTo(new Vector2d(-15, 14))
//                .turn(Math.toRadians(45));







        Action trajectoryActionCloseOut = tab1.fresh()
                .build();




        // actions that need to happen on init; for instance, a claw tightening.
        claw.CloseClaw();
        //bucket.UntiltBucket();

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                        //bucket.tiltBucketToMaxAction(),
                        //bucket.unTiltBucketAction(),
                        //put the arm in the right positon
                        //wrist3.closeClaw(),

                )
        );
    }
}