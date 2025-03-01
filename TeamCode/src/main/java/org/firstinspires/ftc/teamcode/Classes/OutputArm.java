package org.firstinspires.ftc.teamcode.Classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class OutputArm {
    // Servo instances
    private Servo OutputClaw;
    private Servo OutputWrist;

    // Positions for OutputClaw and closed states
    public static double OpenOutputClaw = 0.6;
    public static double ClosedOutputClaw = 0.35;
    //edit for special values Mr. guy
    public static double LowerOutputWrist = 0.9;
    public static double RaiseOutputWrist = 0.02;

    // Constructor
    public OutputArm(HardwareMap hardwareMap, String OutputClawName, String OutputWristName) {
        OutputClaw = hardwareMap.get(Servo.class, OutputClawName);
        OutputWrist = hardwareMap.get(Servo.class, OutputWristName);
        OutputWrist.setDirection(Servo.Direction.FORWARD);
    }

    // Method to OutputClaw the claw
    public void OpenOutputClaw() {
        OutputClaw.setPosition(OpenOutputClaw);
    }

    // Method to close the claw
    public void CloseOutputClaw() {
        OutputClaw.setPosition(ClosedOutputClaw);
    }

    // Method to set custom positions for the servos
    public void setClaw(double ClawPosition) {
        OutputClaw.setPosition(ClawPosition);
    }
    // 1 is lowering and 0 is up up and away
    public void setWrist(double WristPosition) {
        OutputWrist.setPosition(WristPosition);
    }
}