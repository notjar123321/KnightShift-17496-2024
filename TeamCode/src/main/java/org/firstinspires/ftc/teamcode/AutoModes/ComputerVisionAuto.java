package org.firstinspires.ftc.teamcode.AutoModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.google.android.gms.common.api.Result;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Classes.MecanumDrive;
import org.firstinspires.ftc.teamcode.Classes.RobotConstants;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import android.content.res.AssetManager;
import android.graphics.Bitmap;

import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.support.*;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.support.image.ImageProcessor;
import org.tensorflow.lite.support.image.ops.ResizeOp;

@Config
@Autonomous(name = "Notworkingcameravision", group = "Autonomous")
public class ComputerVisionAuto extends LinearOpMode {
    private OpenCvWebcam webcam;

    public class Claw {
        private DcMotorSimple wrist3;

        public Claw(HardwareMap hardwareMap) {
            wrist3 = hardwareMap.get(DcMotorSimple.class, "wrist3");
        }

        public void CloseClaw(){
            wrist3.setPower(-.5);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist3.setPower(0.5);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
        public void OpenClaw() {
            wrist3.setPower(.5);
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist3.setPower(-0.5);
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
        private Servo wrist2 = null;

        private ElapsedTime timer;
        private Telemetry telemetry;

        public int target_position = 0; // Target position for the arm
        private double integralSum = 0; // Integral for PID control
        private double lastError = 0; // Last error value for PID
        private double lastTime = 0; // Last time update was called
        private double gravityCompensation = 0.00005; // Gravity compensation factor (adjust as needed)
        private double target_velocity = .05; // Target velocity for constant velocity control

        // PID Constants
        private double kP = 0.01; // Proportional constant
        private double kI = 0.001; // Integral constant
        private double kD = 0.001; // Derivative constant

        private double maxIntegral = .05; // Limit integral to prevent windup
        private double maxDerivative = 0.0003; // Limit derivative changes

        public Arm2(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
            target_position = 0;
            motor1 = hardwareMap.get(DcMotor.class, "INTAKE");

            timer = elapsedTime;
            telemetry = telemetryIn;

            motor1.setDirection(DcMotor.Direction.REVERSE);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use the encoder for power control
            wrist1 = hardwareMap.get(Servo.class, "INPUTLEFT");
            wrist2 = hardwareMap.get(Servo.class, "INPUTRIGHT"); // remember to change in config
            wrist2.setDirection(Servo.Direction.REVERSE);
        }

        public void update() {
            double pos = (motor1.getCurrentPosition());
            double error = target_position - pos;

            if (pos < 0) {
                target_position = 0;
            }
            if (pos < 200) {
                wrist1.setPosition(0.1);
                wrist2.setPosition(0.1);
            }

            double currentTime = timer.seconds();
            double deltaTime = currentTime - lastTime;

            double pTerm = kP * error;

            integralSum = Range.clip(integralSum + error * deltaTime, -maxIntegral, maxIntegral);
            double iTerm = kI * integralSum;

            double deltaError = error - lastError;
            double dTerm = (deltaTime > 0) ? kD * Range.clip(deltaError / deltaTime, -maxDerivative, maxDerivative) : 0;

            double pidOutput = pTerm + iTerm + dTerm;
            double motorPower = Range.clip(pidOutput + gravityCompensation * Math.signum(error), -0.5, 0.5);

            motor1.setPower(motorPower);

            lastError = error;
            lastTime = currentTime;
            sleep(5);
            telemetry.update();
        }

        public void moveElbowTo(int ticks) {
            target_position = ticks;
            target_position = Range.clip(target_position, 0, 10000);
            motor1.setTargetPosition(target_position);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int distanceToTarget = Math.abs(target_position - motor1.getCurrentPosition());
            double currentPower = 1;
            while (motor1.isBusy()) {
                motor1.setPower(currentPower);
            }
            telemetry.update();
        }

        public void moveElbow(int ticks) {
            target_position += ticks;
            target_position = Range.clip(target_position, 0, 10000);
            motor1.setTargetPosition(target_position);

            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            double currentPower = 1;
            double maxPower = 1;

            while (motor1.isBusy()) {
                int currentPos = (motor1.getCurrentPosition());
                int distanceToTarget = Math.abs(target_position - currentPos);
                motor1.setPower(currentPower);
            }
        }

        public void setTargetPosition(int position) {
            target_position = position;
        }

        // Action class implementation
        public class MoveToPositionAction implements Action {
            private int targetPosition;

            public MoveToPositionAction(int position) {
                this.targetPosition = Range.clip(position, 0, 10000);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                motor1.setTargetPosition(targetPosition);
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                double liftPower = 1.0;
                motor1.setPower(liftPower);

                if (!motor1.isBusy()) {
                    motor1.setPower(0);
                    motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    return true; // Action is complete
                }

                return false; // Action is still running
            }
        }

        public Action moveToPositionAction(int position) {
            return new MoveToPositionAction(position);
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
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);

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
       /** public int detectSampleLocation(SampleDetection detection) {
            // Capture an image from the webcam
            Bitmap bitmap = captureImageFromCamera();
            if (bitmap == null) {
                telemetry.addData("Detection", "Failed to capture image");
                telemetry.update();
                return -1; // Error code for failed capture
            }

            // Preprocess the image
            TensorImage inputImage = detection.preprocessImage(bitmap);

            // Model input/output setup (assumes single output with 3 locations)
            float[][] outputLocations = new float[1][3]; // Adjust based on model output

            // Run the inference
            detection.tflite.run(inputImage.getBuffer(), outputLocations);

            // Process output to determine sample location
            float left = outputLocations[0][0];
            float center = outputLocations[0][1];
            float right = outputLocations[0][2];

            if (left > center && left > right) {
                return 0; // Left position
            } else if (center > left && center > right) {
                return 1; // Center position
            } else {
                return 2; // Right position
            }
        }

        // Close the interpreter when done
        public void close() {
            if (tflite != null) {
                tflite.close();
                tflite = null;
            }
        }**/
    }




    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(24, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw wrist3 = new Claw(hardwareMap);
        SCLift scLift = new SCLift(hardwareMap);
        Arm2 arm = new Arm2(hardwareMap, new ElapsedTime(), telemetry);
        SampleDetection detection = new SampleDetection(hardwareMap.appContext.getAssets(), "model.tflite");


        //Bucket bucket = new Bucket(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        try {
            //tfLite = new Interpreter(loadModelFile());
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to load model");
            telemetry.update();
            return;
        }



        while (opModeIsActive()) {
            arm.update(); // consistently update PID control
            telemetry.update(); // update telemetry in each loop cycle
            //int sampleLocation = detectSampleLocation(detection);  // Detect sample position


            telemetry.update();
            sleep(10);
        }

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(4, 4))
                .turn(Math.toRadians(45+180));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-20, 2, Math.toRadians(45+180)))
                .strafeTo(new Vector2d(10, 14))
                .turn(Math.toRadians(45));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-20, 2, Math.toRadians(45+180)))
                .strafeTo(new Vector2d(-15, 14))
                .turn(Math.toRadians(45));







        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(48, 0))
                .build();




        // actions that need to happen on init; for instance, a claw tightening.
        wrist3.closeClaw();
        //bucket.UntiltBucket();

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();
        arm.update();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                scLift.moveToPositionAction(3100),
                                trajectoryActionChosen
                        )
                )
        );
    }
}



