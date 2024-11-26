package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Linear Slide test", group = "Linear Opmode")
public class BasicOpMode_Linear2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor ls1 = null; // First motor for arm rotation


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeHardware();

        waitForStart();
        runtime.reset();
        LinearSlide LS1 = new LinearSlide(hardwareMap, new ElapsedTime(), telemetry);

        while (opModeIsActive()) {


            if (gamepad1.b) {
                sleep(10);
                LS1.moveLSTo(1500);
            }
            if (gamepad1.x) {
                sleep(10);
                LS1.moveLSTo(0);
            }
            if (gamepad1.y) {
                sleep(10);
                LS1.moveLSTo(3000);
            }

            telemetry.update();
            telemetry.addData("LS1 position", ls1.getCurrentPosition());
        }
    }

    private void initializeHardware() {

        ls1 = hardwareMap.get(DcMotor.class, "LS1");

        // Initialize motors
        ls1.setDirection(DcMotor.Direction.FORWARD);

        // Set up scissor lift motors with encoders
        ls1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ls1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearSlide LS1 = new LinearSlide(hardwareMap, new ElapsedTime(), telemetry);


    }
}