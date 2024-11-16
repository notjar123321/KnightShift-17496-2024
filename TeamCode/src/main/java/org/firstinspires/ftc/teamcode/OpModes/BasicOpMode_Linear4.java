package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Classes.Arm;
import org.firstinspires.ftc.teamcode.Classes.Arm2;

@TeleOp(name = "Single Driver improved LinearOp", group = "Linear Opmode")
public class BasicOpMode_Linear4 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackRightMotor;
    private DcMotor armMotor1 = null; // First motor for arm rotation
    private DcMotor armMotor2 = null;
    private DcMotor SC1;
    private DcMotor SC2;
    private Servo wrist1 = null; // First wrist servo

    private DcMotorSimple wrist3 = null;
    private Arm2 arm;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();  // Orientation data from IMU
    private double initYaw;  // Initial yaw from IMU at the start
    private boolean isFieldCentric = true;  // Field-Centric control mode
    private double powerFL, powerFR, powerBL, powerBR;  // Motor powers


    private double sens = 0.7;
    private int scissorLiftPosition = 0;
    private int mode = 0; // 0: Drive, 1: Arm Control, 2: Scissor Lift
    public double wristPosition = 0;
    private double wristPower = 0;
    private boolean clawOpen = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeHardware();

        waitForStart();
        runtime.reset();
        wristPosition = (wrist1.getPosition() ) ;
        while (opModeIsActive()) {
            // Toggle between modes with gamepad1.y
            if (gamepad1.y) {
                mode = (mode + 1) % 2; // Cycle through 0 and 1
                sleep(200); // Debounce delay
                telemetry.addData("Mode", mode == 0 ? "Drive" : "Arm Control");
                telemetry.update();
            }
            if (gamepad1.b && (clawOpen)) {
                clawOpen=false;
                wrist3.setPower(-.5); // Set motor power based on joystick input
                sleep(40);
            }
            if (gamepad1.b && (!clawOpen)) {
                clawOpen=true;
                wrist3.setPower(.5); // Set motor power based on joystick input
                sleep(40);
            }
            if (gamepad1.right_bumper) {
                // Move wrist up (adjust the position as needed)
                wristPosition = wrist1.getPosition();
                wristPosition += .05;
                wrist1.setPosition(wristPosition);
                sleep(50);
            } else if (gamepad1.left_bumper) {
                // Move wrist down (adjust the position as needed)
                wristPosition = wrist1.getPosition();
                wristPosition -= .05;
                wrist1.setPosition(wristPosition);

                sleep(50);
            }

            switch (mode) {
                case 0: // Drive mode
                    driveMode();
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("Arm Position Motor1", armMotor1.getCurrentPosition());
                    telemetry.addData("Arm Position Motor2", armMotor2.getCurrentPosition());
                    telemetry.addData("Target Position", Arm.target_position);
                    telemetry.addData("Scissor Lift Position SC1", SC1.getCurrentPosition());
                    telemetry.addData("Scissor Lift Position SC2", SC2.getCurrentPosition());
                    telemetry.addData("Wrist1 Position", wrist1.getPosition());

                    telemetry.update();
                    break;
                case 1: // Arm Control mode (includes scissor lift and wrist control)
                    armControlMode();
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("Arm Position Motor1", armMotor1.getCurrentPosition());
                    telemetry.addData("Arm Position Motor2", armMotor2.getCurrentPosition());
                    telemetry.addData("Target Position", Arm.target_position);
                    telemetry.addData("Scissor Lift Position SC1", SC1.getCurrentPosition());
                    telemetry.addData("Scissor Lift Position SC2", SC2.getCurrentPosition());
                    telemetry.addData("Wrist1 Position", wrist1.getPosition());

                    telemetry.update();
                    break;
            }


            telemetry.update();
        }
    }

    private void initializeHardware() {
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");
        armMotor1 = hardwareMap.get(DcMotor.class, "CLAW1"); // First arm motor
        armMotor2 = hardwareMap.get(DcMotor.class, "CLAW2"); // second arm motor
        SC1 = hardwareMap.get(DcMotor.class, "Scissor1");
        SC2 = hardwareMap.get(DcMotor.class, "Scissor2");
        wrist1 = hardwareMap.get(Servo.class, "wrist1");
        wrist3 = hardwareMap.get(DcMotorSimple.class, "wrist3");

        // Initialize motors
        FrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        BackRightMotor.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor1.setDirection(DcMotor.Direction.FORWARD);
        armMotor2.setDirection(DcMotor.Direction.REVERSE);
        wrist1.setDirection(Servo.Direction.FORWARD);

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

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        // Make sure the IMU is calibrated before proceeding
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        // Get the initial heading for field-centric calculations
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initYaw = lastAngles.firstAngle;
    }

    private void driveMode() {
        arm.update();
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        FrontLeftMotor.setPower(frontLeftPower * sens);
        BackLeftMotor.setPower(backLeftPower * sens);
        FrontRightMotor.setPower(frontRightPower * sens);
        BackRightMotor.setPower(backRightPower * sens);
        if (gamepad1.dpad_up) {
            scissorLiftPosition += 100;

            scissorLiftPosition = Math.max(-3100, Math.min(scissorLiftPosition, 3100));

            SC1.setTargetPosition(scissorLiftPosition);
            SC2.setTargetPosition(scissorLiftPosition);
            SC1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SC2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SC1.setPower(1);
            SC2.setPower(1);
            sleep(10);
        }
        if (gamepad1.dpad_down) {
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


    }

    private void armControlMode() {
        arm.update();
        // Example arm control code
        if (gamepad1.dpad_down) {
            arm.moveElbow(-5);
            sleep(10);
        }
        if (gamepad1.dpad_up) {
            arm.moveElbow(5);
            sleep(10);
        }
        if (gamepad1.dpad_left) {
            arm.moveElbowTo(570);
            sleep(10);
        }
        if (gamepad1.dpad_right) {
            arm.moveElbowTo(850);
            sleep(10);
        }


        if (gamepad1.left_stick_y != 0) {
            scissorLiftPosition += (int)(gamepad1.left_stick_y * 50);

            scissorLiftPosition = Math.max(0, Math.min(scissorLiftPosition, 3100));

            SC1.setTargetPosition(scissorLiftPosition);
            SC2.setTargetPosition(scissorLiftPosition);
            SC1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SC2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SC1.setPower(1);
            SC2.setPower(1);
            sleep(10);
        }

        arm.update();



    }

    private void scissorLiftMode() {
        arm.update();

        if (gamepad1.left_stick_y != 0) {
            scissorLiftPosition += (int)(gamepad1.left_stick_y * 50);
            sleep(10);
        }
        scissorLiftPosition = Math.max(0, Math.min(scissorLiftPosition, 3100));

        SC1.setTargetPosition(scissorLiftPosition);
        SC2.setTargetPosition(scissorLiftPosition);
        SC1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SC2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SC1.setPower(1);
        SC2.setPower(1);
    }
}
