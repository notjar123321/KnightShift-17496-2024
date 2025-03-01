package org.firstinspires.ftc.teamcode.Classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm2 {
    private DcMotor motor1;
    private DcMotor motor2;  // Second motor (opposite direction)
    private Telemetry telemetry;

    public static int target_position = 0; // Target position for the arm (linked to FTC Dashboard)
    public static double powerFactor = -1; // Adjusts the tanh curve's strength
    public static int errorThreshold = 5; // Threshold for stopping movement
    public static double divider = 150;

    public Arm2(HardwareMap hardwareMap, Telemetry telemetryIn) {
        target_position=0;
        motor1 = hardwareMap.get(DcMotor.class, "ArmR");
        motor2 = hardwareMap.get(DcMotor.class, "ArmL");

        telemetry = telemetryIn;

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE); // Opposite direction

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void update() {
        int currentPos = motor1.getCurrentPosition();  // Use motor1 for feedback
        double error = target_position - currentPos;

        // Smooth power curve using tanh function
        double pow = powerFactor * Math.tanh(error / divider);

        // Stop moving if within threshold
        if (Math.abs(error) <= errorThreshold) {
            pow = 0;
        }

        // Apply power symmetrically to both motors (motor2 in opposite direction)
        motor1.setPower(-pow);
        motor2.setPower(-pow);

        // Telemetry for debugging
        telemetry.addData("Target Position", target_position);
        telemetry.addData("Current Position", currentPos);
        telemetry.addData("Error", error);
        telemetry.addData("Motor Power", pow);
        telemetry.update();
    }

    public void moveArmBy(int position) {
        target_position += position;
    }
    public void moveArmTo(int position) {
        target_position = position;
    }
}