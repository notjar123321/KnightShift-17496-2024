        /* Copyright (c) 2017 FIRST. All rights reserved.
         *
         * Redistribution and use in source and binary forms, with or without modification,
         * are permitted (subject to the limitations in the disclaimer below) provided that
         * the following conditions are met:
         *
         * Redistributions of source code must retain the above copyright notice, this list
         * of conditions and the following disclaimer.
         *
         * Redistributions in binary form must reproduce the above copyright notice, this
         * list of conditions and the following disclaimer in the documentation and/or
         * other materials provided with the distribution.
         *
         * Neither the name of FIRST nor the names of its contributors may be used to endorse or
         * promote products derived from this software without specific prior written permission.
         *
         * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
         * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
         * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
         * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
         * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
         * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
         * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
         * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
         * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
         * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
         * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
         */

        package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;


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

        @TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
        public class BasicOpMode_Linear extends LinearOpMode {

            private ElapsedTime runtime = new ElapsedTime();
            private DcMotor FrontLeftMotor = null;
            private DcMotor BackLeftMotor = null;
            private DcMotor FrontRightMotor = null;
            private DcMotor BackRightMotor = null;

            // Odometer variables
            private final double wheelDiameter = 4.0; // Wheel diameter in inches
            private final double encoderCountsPerRevolution = 1120; // REV 20 motor encoder counts
            private double distanceTravelled = 0.0; // in inches

            // Current position variables
            private double posX = 0.0; // X position in inches
            private double posY = 0.0; // Y position in inches
            private double angle = 0.0; // Robot's orientation in degrees

            private volatile boolean odometryRunning = true;

            @Override
            public void runOpMode() {
                telemetry.addData("Status", "Initialized");
                telemetry.update();

                // Initialize the hardware variables
                FrontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
                BackLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
                FrontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
                BackRightMotor = hardwareMap.get(DcMotor.class, "BRM");

                // Set motor directions
                FrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
                BackRightMotor.setDirection(DcMotor.Direction.FORWARD);
                FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);

                // Reset encoders
                resetEncoders();

                // Start the odometry thread
                new Thread(new OdometerTask()).start();

                // Wait for the game to start
                waitForStart();
                runtime.reset();

                // Run until the end of the match
                while (opModeIsActive()) {
                    // Drive control variables
                    double leftPower;
                    double rightPower;

                    // POV Mode control
                    double y = -gamepad1.left_stick_y; // Forward/backward
                    double x = gamepad1.left_stick_x; // Left/right
                    double rx = gamepad1.right_stick_x; // Rotation

                    leftPower = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    rightPower = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                    // Set motor powers
                    FrontLeftMotor.setPower((y + x + rx) / leftPower);
                    BackLeftMotor.setPower((y - x + rx) / leftPower);
                    FrontRightMotor.setPower((y - x - rx) / rightPower);
                    BackRightMotor.setPower((y + x - rx) / rightPower);

                    // Update angle based on rotation input, apply scaling factor
                    double angleChange = rx * 5; // Adjust sensitivity factor as needed
                    angle = (angle + angleChange) % 360; // Keep angle within 0-360 degrees

                    // Show the elapsed game time and odometer data
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("Distance Travelled (in)", distanceTravelled);
                    telemetry.addData("Current Position (X,Y)", "X: %.2f, Y: %.2f", posX, posY);
                    telemetry.addData("Orientation", "Angle: %.2f", angle);
                    telemetry.update();
                }

                // Stop the odometry thread when OpMode ends
                odometryRunning = false;
            }

            private void resetEncoders() {
                FrontLeftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
                FrontRightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
                BackLeftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
                BackRightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
                sleep(100); // Allow time for the reset to take effect
                setMotorRunWithoutEncoder();
            }

            private void setMotorRunWithoutEncoder() {
                FrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                FrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            private class OdometerTask implements Runnable {
                @Override
                public void run() {
                    int previousFrontLeftCounts = FrontLeftMotor.getCurrentPosition();
                    int previousFrontRightCounts = FrontRightMotor.getCurrentPosition();
                    int previousBackLeftCounts = BackLeftMotor.getCurrentPosition();
                    int previousBackRightCounts = BackRightMotor.getCurrentPosition();

                    while (odometryRunning) {
                        // Get current encoder counts
                        int currentFrontLeftCounts = FrontLeftMotor.getCurrentPosition();
                        int currentFrontRightCounts = FrontRightMotor.getCurrentPosition();
                        int currentBackLeftCounts = BackLeftMotor.getCurrentPosition();
                        int currentBackRightCounts = BackRightMotor.getCurrentPosition();

                        // Calculate the change in counts
                        int deltaFrontLeftCounts = currentFrontLeftCounts - previousFrontLeftCounts;
                        int deltaFrontRightCounts = currentFrontRightCounts - previousFrontRightCounts;
                        int deltaBackLeftCounts = currentBackLeftCounts - previousBackLeftCounts;
                        int deltaBackRightCounts = currentBackRightCounts - previousBackRightCounts;

                        // Calculate distance traveled by each wheel
                        double distanceFrontLeft = (deltaFrontLeftCounts / encoderCountsPerRevolution) * (Math.PI * wheelDiameter);
                        double distanceFrontRight = (deltaFrontRightCounts / encoderCountsPerRevolution) * (Math.PI * wheelDiameter);
                        double distanceBackLeft = (deltaBackLeftCounts / encoderCountsPerRevolution) * (Math.PI * wheelDiameter);
                        double distanceBackRight = (deltaBackRightCounts / encoderCountsPerRevolution) * (Math.PI * wheelDiameter);

                        // Average the distances
                        double averageDistance = (distanceFrontLeft + distanceFrontRight + distanceBackLeft + distanceBackRight) / 4.0;

                        // Update distance traveled
                        distanceTravelled += averageDistance;

                        // Calculate change in position
                        double deltaX = averageDistance * Math.cos(Math.toRadians(angle));
                        double deltaY = averageDistance * Math.sin(Math.toRadians(angle));

                        // Update current position
                        posX += deltaX;
                        posY += deltaY;

                        // Update previous counts
                        previousFrontLeftCounts = currentFrontLeftCounts;
                        previousFrontRightCounts = currentFrontRightCounts;
                        previousBackLeftCounts = currentBackLeftCounts;
                        previousBackRightCounts = currentBackRightCounts;

                        // Sleep for a short period to avoid overwhelming the CPU
                        try {
                            Thread.sleep(50); // Adjust this value as needed
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }
                }
            }
        }
