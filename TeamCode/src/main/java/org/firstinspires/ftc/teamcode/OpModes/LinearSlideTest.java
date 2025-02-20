package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Classes.Arm2;
import org.firstinspires.ftc.teamcode.Classes.IntakeClaw;
import org.firstinspires.ftc.teamcode.Classes.LinearSlide;

@Config
@TeleOp(name = "Linear Slide Test", group = "Linear Opmode")
public class LinearSlideTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private double sens = .8;
    private DcMotor FrontLeftMotor = null;
    private DcMotor FrontRightMotor = null;
    private DcMotor BackLeftMotor = null;
    private DcMotor BackRightMotor = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor LS1 = null;
    private DcMotor LS2 = null;
    private long lastPressedTimeBumper = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize hardware
        initializeHardware();

        // Now initialize Arm2 after hardware is set up
        Arm2 intake = new Arm2(hardwareMap, new ElapsedTime(), telemetry);
        IntakeClaw intakeclaw = new IntakeClaw(hardwareMap, "CLAW", "WRIST", "ROTATE");
        LinearSlide LS = new LinearSlide(hardwareMap, new ElapsedTime(), telemetry);

        runtime.reset();
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        // Start drivetrain control on a separate thread
        Thread drivetrainThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    driveControl();
                }
            }
        });

        // Start the drivetrain thread
        drivetrainThread.start();

        // Continue with other operations, for example, controlling other components
        while (opModeIsActive()) {
            if(Math.abs(gamepad2.right_stick_y)>.1 || Math.abs(gamepad2.left_stick_y)>.1){
                LS1.setPower(gamepad2.right_stick_y);
                LS2.setPower(gamepad2.left_stick_y);}
            LS1.setPower(0);
            LS2.setPower(0);


            // You can add additional logic here if needed

            telemetry.addData("LS1 Power", LS1.getPower());
            telemetry.addData("LS2 Power", LS2.getPower());
            telemetry.addData("LS1 Position", LS1.getCurrentPosition());
            telemetry.addData("LS2 Position", LS2.getCurrentPosition());


            telemetry.update();
        }

        // Wait for the drivetrain thread to finish before ending the opmode
        drivetrainThread.join();
    }

    private void driveControl() {
        double y = -gamepad1.left_stick_y; // Forward/backward (left stick vertical)
        double x = -gamepad1.left_stick_x;  // Left/right strafing (left stick horizontal)
        double rx = gamepad1.right_stick_x; // Rotation (right stick horizontal)

        // Calculate the largest possible input sum to scale the powers properly
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Calculate motor powers
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Apply power scaling factor
        FrontLeftMotor.setPower(-frontLeftPower * sens);
        BackLeftMotor.setPower(-backLeftPower * sens);
        FrontRightMotor.setPower(frontRightPower * sens);
        BackRightMotor.setPower(backRightPower * sens);
    }




    private void initializeHardware() {
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");
        motor1 = hardwareMap.get(DcMotor.class, "ArmR");
        motor2 = hardwareMap.get(DcMotor.class, "ArmL");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        LS1 = hardwareMap.get(DcMotor.class, "LS1");
        LS2 = hardwareMap.get(DcMotor.class, "LS2");



    }
    private static final long DEBOUNCE_DELAY = 200; // 200 milliseconds
    private boolean isButtonPressed(long currentTime, long lastPressedTime) {
        if (currentTime - lastPressedTime > DEBOUNCE_DELAY) {
            return true;
        }
        return false;
    }
}
