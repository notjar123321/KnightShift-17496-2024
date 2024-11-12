package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TwoDriver LinearOpMode", group = "Linear Opmode")
public class TwoDriver_LinearOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackRightMotor;
    private DcMotor armMotor1 = null; // First motor for arm rotation

    private DcMotor SC1;
    private DcMotor SC2;
    private Servo wrist1 = null; // First wrist servo
    private Servo wrist2 = null;
    private DcMotorSimple wrist3 = null;
    private Arm2 arm;

    private double sens = 0.7;
    private int scissorLiftPosition = 0;
    private int mode = 0; // 0: Drive, 1: Arm Control, 2: Scissor Lift
    public double wristPosition = 0;
    private double wristPower = 0;
    private boolean clawOpen = false;


    //odemetry variables
    // Odometry variables
    private double leftWheelDistance = 0;
    private double rightWheelDistance = 0;
    private double lastLeftEncoder = 0;
    private double lastRightEncoder = 0;
    private double robotAngle = 0;  // Robot's orientation in radians
    private double wheelBase = 12.6;  // Distance between left and right wheels (in inches)
    private double wheelRadius = 2; // Radius of the wheels (in inches)
    private double encoderTicksPerRevolution = 560*2;  // Encoder ticks per full wheel revolution
    private double distancePerTick = (2 * Math.PI * wheelRadius) / encoderTicksPerRevolution; // Distance per encoder tick
    private String clawposition = "Open";


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeHardware();

        waitForStart();
        runtime.reset();
        wristPosition = (wrist1.getPosition() + wrist2.getPosition()) / 2;
        while (opModeIsActive()) {
            double leftEncoder = FrontLeftMotor.getCurrentPosition();
            double rightEncoder = FrontRightMotor.getCurrentPosition();


            // Calculate how far each wheel has moved since the last loop
            double leftDistance = (leftEncoder - lastLeftEncoder) * distancePerTick;
            double rightDistance = (rightEncoder - lastRightEncoder) * distancePerTick;

            // Update last encoder positions for next iteration
            lastLeftEncoder = leftEncoder;
            lastRightEncoder = rightEncoder;
            double deltaTheta = (rightDistance - leftDistance) / wheelBase;
            robotAngle += deltaTheta;


            // Calculate robot's forward movement (average of left and right distance)
            double averageDistance = (leftDistance + rightDistance) / 2;

            // Update robot's position (x, y)
            double deltaX = averageDistance * Math.cos(robotAngle);
            double deltaY = averageDistance * Math.sin(robotAngle);

            // Update the robot's global position (x, y, theta)
            // Assuming initial position is (0, 0, 0)
            double robotX = deltaX;
            double robotY = deltaY;
            double cosAngle = Math.cos(robotAngle);
            double sinAngle = Math.sin(robotAngle);



            if (gamepad2.b && (clawOpen)) {
                clawOpen = false;
                wrist3.setPower(-.5); // Set motor power based on joystick input
                sleep(40);
            }
            if (gamepad2.b && (!clawOpen)) {
                clawOpen = true;
                wrist3.setPower(.5); // Set motor power based on joystick input
                sleep(40);
            }
            if (gamepad2.right_bumper) {
                // Move wrist up (adjust the position as needed)
                wristPosition = wrist1.getPosition();
                wristPosition += .05;
                wrist1.setPosition(wristPosition);
                wrist2.setPosition(wristPosition);
                sleep(50);
            } else if (gamepad2.left_bumper) {
                // Move wrist down (adjust the position as needed)
                wristPosition = wrist1.getPosition();
                wristPosition -= .05;
                wrist1.setPosition(wristPosition);
                wrist2.setPosition(wristPosition);
                sleep(50);
            }
            arm.update();
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double tempX = x * cosAngle + y * sinAngle;
            double tempY = -x * sinAngle + y * cosAngle;

            double denominator = Math.max(Math.abs(tempX) + Math.abs(tempY) + Math.abs(rx), 1);
            double frontLeftPower = (tempY + tempX + rx) / denominator;
            double backLeftPower = (tempY - tempX + rx) / denominator;
            double frontRightPower = (tempY - tempX - rx) / denominator;
            double backRightPower = (tempY + tempX - rx) / denominator;

            FrontLeftMotor.setPower(frontLeftPower * sens);
            BackLeftMotor.setPower(backLeftPower * sens);
            FrontRightMotor.setPower(frontRightPower * sens);
            BackRightMotor.setPower(backRightPower * sens);
            if (gamepad2.dpad_up) {
                scissorLiftPosition += 100;

                scissorLiftPosition = Math.max(0, Math.min(scissorLiftPosition, 3100));

                SC1.setTargetPosition(scissorLiftPosition);
                SC2.setTargetPosition(scissorLiftPosition);
                SC1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SC2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SC1.setPower(1);
                SC2.setPower(1);
                sleep(10);
            }
            if (gamepad2.dpad_down) {
                scissorLiftPosition -= 100;

                scissorLiftPosition = Math.max(0, Math.min(scissorLiftPosition, 3100));

                SC1.setTargetPosition(scissorLiftPosition);
                SC2.setTargetPosition(scissorLiftPosition);
                SC1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SC2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SC1.setPower(1);
                SC2.setPower(1);
                sleep(10);
            }
            robotAngle = (robotAngle + 360) % 360;
            if(wrist3.getPower()==.5)
                clawposition="Open";
            if(wrist3.getPower()==-.5)
                clawposition="Closed";
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm Position Motor1", armMotor1.getCurrentPosition());

            telemetry.addData("Target Position", Arm.target_position);
            telemetry.addData("Scissor Lift Position SC1", SC1.getCurrentPosition());
            telemetry.addData("Scissor Lift Position SC2", SC2.getCurrentPosition());
            telemetry.addData("Wrist1 Position", wrist1.getPosition());
            telemetry.addData("Wrist2 Position", wrist2.getPosition());
            telemetry.addData("Wrist3 Position", clawposition);
            telemetry.addData("Left Encoder", leftEncoder);
            telemetry.addData("Right Encoder", rightEncoder);
            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Robot Angle", (robotAngle*180)/Math.PI);
            telemetry.update();

        }
    }

    private void initializeHardware() {
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");
        armMotor1 = hardwareMap.get(DcMotor.class, "CLAW1"); // First arm motor
        SC1 = hardwareMap.get(DcMotor.class, "Scissor1");
        SC2 = hardwareMap.get(DcMotor.class, "Scissor2");
        wrist1 = hardwareMap.get(Servo.class, "wrist1");
        wrist2 = hardwareMap.get(Servo.class, "wrist2");
        wrist3 = hardwareMap.get(DcMotorSimple.class, "wrist3");

        // Initialize motors
        FrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        BackRightMotor.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor1.setDirection(DcMotor.Direction.FORWARD);

        wrist1.setDirection(Servo.Direction.REVERSE);
        wrist2.setDirection(Servo.Direction.FORWARD);
        SC1.setDirection(DcMotor.Direction.FORWARD);
        SC2.setDirection(DcMotor.Direction.REVERSE);

        // Set up scissor lift motors with encoders
        SC1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SC2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SC1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SC2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SC1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SC2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = new Arm2(hardwareMap, runtime, telemetry);
        robotAngle=0;
    }
}