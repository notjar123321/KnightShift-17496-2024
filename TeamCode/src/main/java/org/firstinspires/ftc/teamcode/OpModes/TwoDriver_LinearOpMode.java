package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Classes.Arm;
import org.firstinspires.ftc.teamcode.Classes.Arm2;

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
    private Servo bucket = null;

    private Servo wrist3 = null;
    private Arm2 arm;
    BNO055IMU               imu;


    private double sens = 0.7;
    private int scissorLiftPosition = 0;
    private int mode = 0; // 0: Drive, 1: Arm Control, 2: Scissor Lift
    public double wristPosition = 0;
    private double wristPower = 0;
    private boolean clawOpen = false;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;


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
    private boolean isFieldCentric = false;

    double y = 0;
    double x = 0;
    double rx = 0;
    double power = 0;
    double theta = 0;
    double sin = 0;
    double cos = 0;
    double max = 0;









    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeHardware();

        waitForStart();
        runtime.reset();
        wristPosition = (wrist1.getPosition()) ;
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
            double cosAngle = Math.cos(currAngle);
            double sinAngle = Math.sin(currAngle);
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currAngle = lastAngles.firstAngle; // The yaw (Z-axis) is usually the robot's heading.



            if (gamepad2.b && (clawOpen)) {
                clawOpen = false;
                wrist3.setPosition(1); // Set motor power based on joystick input
                sleep(40);
            }
            if (gamepad2.b && (!clawOpen)) {
                clawOpen = true;
                wrist3.setPosition(0); // Set motor power based on joystick input
                sleep(40);
            }
            if(gamepad1.y){
                isFieldCentric=!isFieldCentric;
                sleep(200);
            }
            if (gamepad2.right_bumper) {
                // Move wrist up (adjust the position as needed)
                wristPosition = wrist1.getPosition();
                wristPosition += .05;
                wrist1.setPosition(wristPosition);

                sleep(50);
            } else if (gamepad2.left_bumper) {
                // Move wrist down (adjust the position as needed)
                wristPosition = wrist1.getPosition();
                wristPosition -= .05;
                wrist1.setPosition(wristPosition);

                sleep(50);
            }
            arm.update();
            if(!isFieldCentric){
                 y = -gamepad1.left_stick_y;
                 x = gamepad1.left_stick_x;
                 rx = gamepad1.right_stick_x;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                FrontLeftMotor.setPower(frontLeftPower * sens);
                BackLeftMotor.setPower(backLeftPower * sens);
                FrontRightMotor.setPower(frontRightPower * sens);
                BackRightMotor.setPower(backRightPower * sens);

            }
            else{
                lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currAngle = lastAngles.firstAngle; // The yaw (Z-axis) is usually the robot's heading.

                double robotHeading = Math.toRadians(currAngle);

                // Transform joystick inputs for field-centric driving
                double tempX = x * Math.cos(robotHeading) + y * Math.sin(robotHeading);
                double tempY = -x * Math.sin(robotHeading) + y * Math.cos(robotHeading);

                // Apply the adjusted values to the motors
                double denominator = Math.max(Math.abs(tempY) + Math.abs(tempX) + Math.abs(rx), 1);
                double frontLeftPower = (tempY + tempX + rx) / denominator;
                double backLeftPower = (tempY - tempX + rx) / denominator;
                double frontRightPower = (tempY - tempX - rx) / denominator;
                double backRightPower = (tempY + tempX - rx) / denominator;

                FrontLeftMotor.setPower(frontLeftPower * sens);
                BackLeftMotor.setPower(backLeftPower * sens);
                FrontRightMotor.setPower(frontRightPower * sens);
                BackRightMotor.setPower(backRightPower * sens);
            }
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
            if(wrist3.getPosition()==1)
                clawposition="Open";
            if(wrist3.getPosition()==0)
                clawposition="Closed";
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm Position Motor1", armMotor1.getCurrentPosition());

            telemetry.addData("Target Position", Arm.target_position);
            telemetry.addData("Scissor Lift Position SC1", SC1.getCurrentPosition());
            telemetry.addData("Scissor Lift Position SC2", SC2.getCurrentPosition());
            telemetry.addData("Wrist1 Position", wrist1.getPosition());

            telemetry.addData("Wrist3 Position", clawposition);
            telemetry.addData("Left Encoder", leftEncoder);
            telemetry.addData("Right Encoder", rightEncoder);
            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("IMU Heading (Yaw)/Angle", currAngle);  // Robot’s yaw (heading)
            telemetry.addData("IMU Pitch", lastAngles.secondAngle);  // Pitch
            telemetry.addData("IMU Roll", lastAngles.thirdAngle);   // Roll

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
        wrist3 = hardwareMap.get(Servo.class, "wrist1");
        bucket= hardwareMap.get(Servo.class, "bucket");
        wrist3 = hardwareMap.get(Servo.class, "wrist3");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        Orientation             lastAngles = new Orientation();
        double                  globalAngle, power = .30, correction;

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();




        // Initialize motors
        FrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        BackRightMotor.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor1.setDirection(DcMotor.Direction.FORWARD);

        wrist1.setDirection(Servo.Direction.REVERSE);

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