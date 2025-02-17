package org.firstinspires.ftc.teamcode.Classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class IntakeClaw {
    // Servo instances
    private Servo Claw;
    private Servo Wrist;
    private Servo Rotate;

    // Positions for open and closed states
    public static double Claw_Open = .25;
    public static double Claw_Closed = 0.95;


    // Constructor
    public IntakeClaw(HardwareMap hardwareMap, String ClawName, String WristName, String RotateName) {
        Claw = hardwareMap.get(Servo.class, ClawName);
        Wrist = hardwareMap.get(Servo.class, WristName);
        Rotate = hardwareMap.get(Servo.class, RotateName);
    }

    // Method to open the claw
    public void open() {
        Claw.setPosition(Claw_Open);
    }

    // Method to close the claw
    public void close() {
        Claw.setPosition(Claw_Closed);
    }

    // Method to set custom positions for the servos
    public void setClawPosition(double Position) {
        Claw.setPosition(Position);
        
    }
    
    public void setWristPosition(double Position){
        Wrist.setPosition(Position);
    }
    public void setRotatePosition(double Position){
        Rotate.setPosition(Position);
    }
    
    //Closer to 1 is Right and closer to 0 is Left 
    
    public void setRotation(double Position){
        Rotate.setPosition(Position);
    }
}