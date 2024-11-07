
        package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;

        @TeleOp(name="ScissorLiftRepairMode", group="Linear OpMode")
        public class BasicOpMode_Linear2 extends LinearOpMode {

            private ElapsedTime runtime = new ElapsedTime();
            private DcMotor FrontLeftMotor = null;
            private DcMotor BackLeftMotor = null;
            private DcMotor FrontRightMotor = null;
            private DcMotor BackRightMotor = null;
            private DcMotor armMotor1 = null; // First motor for arm rotation
            private DcMotor armMotor2 = null;
            private DcMotor SC1 = null; // First motor for arm rotation
            private DcMotor SC2 = null;
            private Servo wrist1 = null; // First wrist servo
            private Servo wrist2 = null;
            private DcMotorSimple wrist3 = null;



            // Odometer variables
            private final double wheelDiameter = 4.0; // Wheel diameter in inches
            private final double encoderCountsPerRevolution = 1120; // REV 20 motor encoder counts
            private double distanceTravelled = 0.0; // in inches
            private double wristPower = 0.0;

            // Current position variables
            private double posX = 0.0; // X position in inches
            private double posY = 0.0; // Y position in inches
            private double angle = 0.0; // Robot's orientation in degrees

            //arm positions
            private final int arm_down = 0; //for motor 2 ecoder
            private final int arm_middle = 578;
            private final int arm_bottom = 1015;


            //Scissor Lift positions
            public final int upPosition = 3600;
            public final int downPosition = 0;

            //wrist position
            public double wristPosition = 0;
            public double clawPosition = 0;

            //Sensitivity
            private double sens = .7;

            private int currentPos;

            private volatile boolean odometryRunning = true;
            private boolean armControlMode = false;
            private boolean isSCup = false;

            @Override
            public void runOpMode() {
                telemetry.addData("Status", "Initialized");
                telemetry.update();

                // Initialize the hardware variables
                FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
                BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
                FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
                BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");
                armMotor1 = hardwareMap.get(DcMotor.class, "CLAW1"); // First arm motor
                armMotor2 = hardwareMap.get(DcMotor.class, "CLAW2"); //second arm motor
                SC1 = hardwareMap.get(DcMotor.class, "Scissor1");
                SC2 = hardwareMap.get(DcMotor.class, "Scissor2");
                wrist1= hardwareMap.get(Servo.class, "wrist1");
                wrist2= hardwareMap.get(Servo.class, "wrist2");
                wrist3= hardwareMap.get(DcMotorSimple.class, "wrist3");


                // Set motor directions
                FrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
                BackRightMotor.setDirection(DcMotor.Direction.FORWARD);
                FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                armMotor1.setDirection(DcMotor.Direction.FORWARD);
                armMotor2.setDirection(DcMotor.Direction.REVERSE);
                wrist1.setDirection(Servo.Direction.REVERSE);
                wrist2.setDirection(Servo.Direction.FORWARD);
                int groundPosition = 0;
                int midPosition = 580;
                int highPosition = 1000;

                // Default target position



                Arm arm = new Arm(hardwareMap, runtime, telemetry);
                ScissorLift scissor1 = new ScissorLift(hardwareMap, runtime, telemetry);
                ScissorLift scissor2 = new ScissorLift(hardwareMap, runtime, telemetry);
                // Start the odometry thread


                // Wait for the game to start
                waitForStart();
                runtime.reset();



                // Run until the end of the match
                while (opModeIsActive()) {
                    // Drive control variables
                    SC1.setPower(gamepad1.left_stick_y);
                    SC2.setPower(gamepad1.right_stick_y);


                    }




                }

                // Stop the odometry thread when OpMode ends


            }





        /*
         * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
         * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
         * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
         * class is instantiated on the Robot Controller and executed.
         *
         * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
         * It includes all the skeletal structure that all linear OpModes contain.
         *
         * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
         * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
         */
