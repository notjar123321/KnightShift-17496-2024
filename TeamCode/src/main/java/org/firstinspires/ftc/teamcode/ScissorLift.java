package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ScissorLift {
    private DcMotor motorSC1;
    private DcMotor motorSC2;
    private ElapsedTime timer;
    private Telemetry telemetry;

    public static int target_position = 0; // Target position for the scissor lift


    public ScissorLift(HardwareMap hardwareMap, ElapsedTime elapsedTime, Telemetry telemetryIn) {
        motorSC1 = hardwareMap.get(DcMotor.class, "Scissor1");
        motorSC2 = hardwareMap.get(DcMotor.class, "Scissor2");
        timer = elapsedTime;
        telemetry = telemetryIn;

        motorSC1.setDirection(DcMotor.Direction.FORWARD);
        motorSC2.setDirection(DcMotor.Direction.REVERSE); // Reverse motorSC2 if needed


    }
    public void moveScissorLift(int ticks) {
        target_position += ticks;
        motorSC1.setTargetPosition(target_position);
        motorSC2.setTargetPosition(target_position);

        motorSC1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSC2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorSC1.setPower(1); // Adjust power as needed
        motorSC2.setPower(1);
    }

    public void resetTimer() {
        timer.reset();
    }

    public int getTargetPosition() {
        return target_position;
    }
}
