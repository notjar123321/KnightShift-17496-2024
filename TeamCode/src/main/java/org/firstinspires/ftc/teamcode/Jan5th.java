package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Linear Slide testFinal", group = "Linear Opmode")
public class Jan5th extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor ls1 = null; // First motor for arm rotation
    private DcMotor ls2 = null;
    private double y;
    private double x;
    private double rx;
    private double sens = 1;
    private DcMotor FrontLeftMotor=null;
    private DcMotor FrontRightMotor=null;
    private DcMotor BackLeftMotor=null;
    private DcMotor BackRightMotor=null;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeHardware();

        waitForStart();

        runtime.reset();
        LinearSlide LS = new LinearSlide(hardwareMap, new ElapsedTime(), telemetry);

        while (opModeIsActive()) {
            //drive with gamepad1
            /**if (gamepad1.right_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0){
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
        }**/

            //Linear Slide Commands(Dpad)
            if (gamepad2.dpad_left) {
                nonBlockingDelay(10);
                LS.moveLSTo(1500);
            }
            if (gamepad2.dpad_down) {
                nonBlockingDelay(10);
                LS.moveLSTo(0);
            }
            if (gamepad2.dpad_up) {
                nonBlockingDelay(10);
                LS.moveLSTo(3100);
            }


            //Claw Commands(
            


            telemetry.addData("LS1 position", ls1.getCurrentPosition());
            telemetry.addData("LS1 position", ls2.getCurrentPosition());
            telemetry.update();
        }
    }


    private void nonBlockingDelay(double milliseconds) {
        ElapsedTime delayTimer = new ElapsedTime();
        delayTimer.reset();
        while (opModeIsActive() && delayTimer.milliseconds() < milliseconds) {
            telemetry.update();
        }
    }
    private void initializeHardware() {

        ls1 = hardwareMap.get(DcMotor.class, "LS1");
        ls2 = hardwareMap.get(DcMotor.class, "LS2");
        // Initialize motors
        ls1.setDirection(DcMotor.Direction.REVERSE);
        ls2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set up scissor lift motors with encoders
        ls1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ls2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LinearSlide LS = new LinearSlide(hardwareMap, new ElapsedTime(), telemetry);
        /**FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");**/


    }
}