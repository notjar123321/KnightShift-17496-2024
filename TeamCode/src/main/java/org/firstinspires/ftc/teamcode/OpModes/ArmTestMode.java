package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Classes.Arm2;

@TeleOp(name = "Repair Mode", group = "Linear Opmode")
public class ArmTestMode extends LinearOpMode {

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
    private Servo clawgrabber = null;
    private Arm2 arm;

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

        while (opModeIsActive()) {
            // Toggle between modes with gamepad1.y
            if (gamepad1.left_stick_y != 0) {
                scissorLiftPosition += (int)(gamepad1.left_stick_y * 50);

                scissorLiftPosition = Math.max(0, Math.min(scissorLiftPosition, 3100));

                SC1.setTargetPosition(scissorLiftPosition);
                SC2.setTargetPosition(scissorLiftPosition);
                SC1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SC2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SC1.setPower(1);
                SC2.setPower(1);
                sleep(100);
            }
            if(gamepad1.dpad_up)
                arm.moveElbow(10);
            if(gamepad1.dpad_down)
                arm.moveElbow(-10);
            arm.update();
            telemetry.addData("Arm Position Motor1", armMotor1.getCurrentPosition());
            telemetry.addData("SC1", SC1.getCurrentPosition());
            telemetry.addData("SC2", SC2.getCurrentPosition());
            telemetry.addData("wrist1", wrist1.getPosition());
            telemetry.addData("clawgrabber", clawgrabber.getPosition());
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
        //wrist1 = hardwareMap.get(Servo.class, "wrist1");
        //wrist2 = hardwareMap.get(Servo.class, "wrist2");
        //clawgrabber = hardwareMap.get(DcMotorSimple.class, "wrist3");

        // Initialize motors
        FrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        BackRightMotor.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor1.setDirection(DcMotor.Direction.REVERSE);
        armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        //wrist1.setDirection(Servo.Direction.REVERSE);
        //wrist2.setDirection(Servo.Direction.FORWARD);
        SC1.setDirection(DcMotor.Direction.FORWARD);
        SC2.setDirection(DcMotor.Direction.REVERSE);

        // Set up scissor lift motors with encoders
        SC1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SC2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SC1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SC2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SC1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SC2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = new Arm2(hardwareMap, runtime, telemetry);
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
