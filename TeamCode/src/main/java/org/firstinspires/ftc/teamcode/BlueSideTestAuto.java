package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

@Config
@Autonomous(name = "Bucket_Side", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {
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
        public static Action openClaw() {
            return new OpenClaw();
        }
    }
    public class SCLift {
        private DcMotor SC1 = null;
        private DcMotor SC2 = null;
        public SCLift() {
            SC1 = hardwareMap.get(DcMotor.class, "Scissor1");
            SC2 = hardwareMap.get(DcMotor.class, "Scissor2");
            SC1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SC1.setDirection(DcMotorSimple.Direction.FORWARD);
            SC2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SC2.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
    public class Arm2 {
        private DcMotor motor1;
        private DcMotor motor2;
        private Servo wrist1 = null; // First wrist servo
        private Servo wrist2 = null;
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
            wrist2 = hardwareMap.get(Servo.class, "wrist2");
            wrist3 = hardwareMap.get(DcMotorSimple.class, "wrist3");
            timer = elapsedTime;
            telemetry = telemetryIn;

            motor1.setDirection(DcMotor.Direction.FORWARD);
            motor2.setDirection(DcMotor.Direction.REVERSE);

        }

        public void update() {
            if(motor1.getCurrentPosition() < 300) {
                wrist1.setPosition(.95);
                wrist2.setPosition(.95);
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


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw wrist3 = new Claw("wrist3");


        // vision here that outputs position


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-20, 2))
                .turn(Math.toRadians(45));



        Action trajectoryActionCloseOut = tab2.fresh()
                .strafeTo(new Vector2d(48, 0)) // go to park
                .build();


        // actions that need to happen on init; for instance, a claw tightening.



        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab2.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        wrist3.openClaw()
                        trajectoryActionCloseOut
                )
        );
    }
}