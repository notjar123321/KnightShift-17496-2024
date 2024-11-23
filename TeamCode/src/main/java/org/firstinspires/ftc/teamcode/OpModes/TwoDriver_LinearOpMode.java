package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;


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
    private DcMotorSimple bucket = null;

    private DcMotorSimple wrist3 = null;
    private Arm2 arm;
    BNO055IMU               imu;


    private double sens = 1;
    private int scissorLiftPosition = 0;
    private int mode = 0; // 0: Drive, 1: Arm Control, 2: Scissor Lift
    public double wristPosition = 0;
    private double wristPower = 0;
    private boolean clawOpen = false;
    private boolean bucketUp = false;


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
    //private boolean isFieldCentric = false;

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


            // Calculate robot's forward movement (average of left and right distance)
            double averageDistance = (leftDistance + rightDistance) / 2;


            if (gamepad2.b && (clawOpen)) {
                sleep(200);
                clawOpen = false;
                wrist3.setPower(.2);
                sleep(340);
                wrist3.setPower(0);
                // Set motor power based on joystick input
                sleep(40);
            }
            if (gamepad2.b && (!clawOpen)) {
                sleep(200);
                clawOpen = true;
                wrist3.setPower(-.2); // Set motor power based on joystick input
                sleep(340);
                wrist3.setPower(-.1);
                sleep(40);

            }


            if (gamepad2.right_bumper) {
                Range.clip(wrist1.getPosition(), 0, 1);
                // Move wrist up (adjust the position as needed)
                wristPosition = wrist1.getPosition();
                wristPosition += .1;
                wrist1.setPosition(wristPosition);

                sleep(50);
            } else if (gamepad2.left_bumper) {
                // Move wrist down (adjust the position as needed)
                wristPosition = Range.clip(wrist1.getPosition(), 0, 1);
                wristPosition -= .1;
                wrist1.setPosition(wristPosition);

                sleep(50);
            }
            if (gamepad1.y) {
                bucketUp = true;
                // Move wrist up (adjust the position as needed)
                bucket.setPower(-1);
                sleep(300);
                bucket.setPower(0);
                sleep(10);
            } else if (gamepad1.x) {
                bucketUp = false;
                // Move wrist down (adjust the position as needed)
                bucket.setPower(1);
                sleep(300);
                bucket.setPower(0);
                sleep(10);
            }
            arm.update();
            sleep(10);

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
            if (gamepad2.x){
                arm.moveElbowTo(450);
            }


            if (gamepad2.left_stick_y!=0) {
                SC1.setPower(gamepad2.left_stick_y);
                sleep(10);

            }
            if (gamepad2.right_stick_y!=0) {
                SC2.setPower(gamepad2.right_stick_x);
            }
            if (gamepad2.dpad_right) {
                arm.moveElbow(30);
                sleep(5);
            }
            if (gamepad2.dpad_left) {
                arm.moveElbow(-30);
                sleep(5);
            }
            robotAngle = (robotAngle + 360) % 360;

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm Position Motor1", armMotor1.getCurrentPosition());

            telemetry.addData("Target Position", Arm.target_position);
            telemetry.addData("Scissor Lift Position SC1", SC1.getCurrentPosition());
            telemetry.addData("Scissor Lift Position SC2", SC2.getCurrentPosition());
            telemetry.addData("Wrist1 Position", wrist1.getPosition());


            telemetry.addData("Left Encoder", leftEncoder);
            telemetry.addData("Right Encoder", rightEncoder);

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
        wrist1 = hardwareMap.get(Servo.class, "wrist1"); //ACTUAL WRIST
        bucket= hardwareMap.get(DcMotorSimple.class, "wrist3");
        wrist3 = hardwareMap.get(DcMotorSimple.class, "wrist2"); // claw open close
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        //parameters.mode                = BNO055IMU.SensorMode.IMU;
        //parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.loggingEnabled      = false;
        //Orientation             lastAngles = new Orientation();
        //double                  globalAngle, power = .30, correction;

        //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //imu.initialize(parameters);



        // wait for start button.

        waitForStart();





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