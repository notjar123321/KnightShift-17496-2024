package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Actions.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.util.*;
import org.firstinspires.ftc.teamcode.*;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Autonomous(name = "My Auto Mode")
public class AutoMode extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, 0); // Starting position
        drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();
        ElapsedTime time = new ElapsedTime();

        while(true);
            if (isStopRequested()) return;
            Actions.runBlocking(
                    drive.actionBuilder(startPose).strafeTo(new Vector2d(30, 61)).waitSeconds(2).build()
            );
            if(time.time() > 1){
                time.reset();

                startPose = new Pose2d(11,60,Math.toRadians(90));

            }



    }

    private TelemetryPacket getTelemetryPacket() {
        TelemetryPacket packet = new TelemetryPacket();
        // Add telemetry data if needed
        return packet;
    }
}
