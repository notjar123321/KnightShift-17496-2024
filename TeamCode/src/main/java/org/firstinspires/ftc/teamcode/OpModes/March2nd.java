package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Classes.Arm2;

@Config
@TeleOp(name = "March2nd", group = "Linear Opmode")
public class March2nd extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private double sens = .8;
    private DcMotor FrontLeftMotor = null;
    private DcMotor FrontRightMotor = null;
    private DcMotor BackLeftMotor = null;
    private DcMotor BackRightMotor = null;

    Arm2 intake = new Arm2(hardwareMap, new ElapsedTime(), telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeHardware();

        runtime.reset();
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
            if(gamepad1.dpad_down)
            {
                intake.moveElbow(-10);
            }
            if(gamepad1.dpad_up)
            {
                intake.moveElbow(10);
            }
            // You can add additional logic here if needed
            telemetry.update();
        }

        // Wait for the drivetrain thread to finish before ending the opmode
        drivetrainThread.join();
    }

    private void driveControl() {
        // Drive control logic
        double y = -gamepad1.left_stick_y; // Forward/backward (left stick vertical)
        double x = -gamepad1.left_stick_x; // Left/right strafing (left stick horizontal)
        double rx = gamepad1.right_stick_x; // Rotation (right stick horizontal)

        // Calculate the largest possible input sum to scale the powers properly
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Calculate motor powers
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Apply power scaling factor
        FrontLeftMotor.setPower(-Math.sqrt(frontLeftPower)* sens);
        BackLeftMotor.setPower(-Math.sqrt(backLeftPower) * sens);
        FrontRightMotor.setPower(Math.sqrt(frontRightPower) * sens);
        BackRightMotor.setPower(Math.sqrt(backRightPower) * sens);
    }

    private void initializeHardware() {
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");
    }
}
