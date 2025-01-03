package org.firstinspires.ftc.teamcode.Classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class LinearSlide {
    private DcMotor LS1;
    private DcMotor LS2;

    private ElapsedTime timer;
    private Telemetry telemetry;

    public static int target_position = 0; // Target position for the arm


    public LinearSlide(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
        target_position = 0;
        LS1 = hardwareMap.get(DcMotor.class, "LS1");
        LS2 = hardwareMap.get(DcMotor.class, "LS2");


        timer = elapsedTime;
        telemetry = telemetryIn;

        LS1.setDirection(DcMotor.Direction.FORWARD);
        LS2.setDirection(DcMotorSimple.Direction.REVERSE);
        LS1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LS1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use the encoder for power control

        LS2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LS2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use the encoder for power control
        //LS1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //LS2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void moveLSTo(int ticks) {
        target_position = ticks+10;
        LS1.setTargetPosition(target_position);
        target_position = ticks;
        LS2.setTargetPosition(target_position);


        LS1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LS2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double maxPower = 1; // Maximum allowable power

        while (LS1.isBusy() || LS2.isBusy()) {
            LS1.setPower(maxPower);
            LS2.setPower(maxPower*.75);
            telemetry.update();
        }
    }

    public void moveElbow(int ticks) {

        target_position += ticks;
        LS1.setTargetPosition(target_position);
        LS2.setTargetPosition(target_position);
        LS1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LS2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double maxPower = 1; // Maximum allowable power


        while (LS1.isBusy()) {
            int currentPos = (LS1.getCurrentPosition());
            int distanceToTarget = Math.abs(target_position - currentPos);
            LS1.setPower(maxPower);
            LS2.setPower(maxPower*.75);
            telemetry.update();

        }
    }
}
