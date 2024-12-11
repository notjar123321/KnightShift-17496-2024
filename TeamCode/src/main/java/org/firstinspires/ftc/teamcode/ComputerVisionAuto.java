package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import android.os.SystemClock;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Cube Detection with Distance (Fixed)", group = "Autonomous")
public class ComputerVisionAuto extends LinearOpMode {

    private OpenCvCamera webcam;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the FTC Dashboard
        dashboard = FtcDashboard.getInstance();

        // Get the camera monitor view ID
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Initialize the webcam
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set the pipeline
        CubeDetectionPipeline pipeline = new CubeDetectionPipeline(telemetry);
        webcam.setPipeline(pipeline);


        // Open the camera asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 24);
                telemetry.addData("Status", "Camera Opened Successfully");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Code: " + errorCode);
                telemetry.update();
            }
        });

        // Wait for the driver to press play
        waitForStart();
        long startTime = SystemClock.uptimeMillis();
        // Main loop
        while (opModeIsActive()) {

            telemetry.addData("FPS", webcam.getFps());
            telemetry.addData("Pipeline Time (ms)", webcam.getPipelineTimeMs());
            telemetry.addData("Scalar[0]", pipeline.getAvgColor().val[0]);
            telemetry.addData("Scalar[1]", pipeline.getAvgColor().val[1]);
            telemetry.addData("Scalar[2]", pipeline.getAvgColor().val[2]);

            telemetry.update();

        }

        // Stop streaming when done
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        dashboard.stopCameraStream();
    }

    static class CubeDetectionPipeline extends OpenCvPipeline {
        private Telemetry telemetry;
        private double focalLength = 460; // Replace with calibrated focal length
        private double realHeightTall = 8.9; // Real-world upright cube height in cm
        private double realHeightShort = 3.8; // Real-world sideways cube height in cm
        Mat hsv = new Mat();
        Mat maskYellow = new Mat();
        Mat maskBlue = new Mat();
        Mat maskRed = new Mat();
        Scalar yellowLower = new Scalar(10, 150, 100);
        Scalar yellowUpper = new Scalar(40, 255, 255);
        Scalar blueLower = new Scalar(90, 105, 50);
        Scalar blueUpper = new Scalar(140, 255, 255);
        Scalar redLower1 = new Scalar(0, 150, 120);
        Scalar redUpper1 = new Scalar(10, 255, 255);
        Scalar redLower2 = new Scalar(135, 135, 110);
        Scalar redUpper2 = new Scalar(180, 255, 255);
        Scalar avgColor = new Scalar(0, 0, 0);

        public CubeDetectionPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }
        public Scalar getAvgColor(){
            return avgColor;
        }

        @Override
        public Mat processFrame(Mat input) {
            // Convert to HSV for color detection
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Generate masks
            Core.inRange(hsv, yellowLower, yellowUpper, maskYellow);
            Core.inRange(hsv, blueLower, blueUpper, maskBlue);
            Core.inRange(hsv, redLower1, redUpper1, maskRed);
            Mat maskRed2 = new Mat();
            Core.inRange(hsv, redLower2, redUpper2, maskRed2);
            Core.bitwise_or(maskRed, maskRed2, maskRed);


            // Combine masks
            Mat combinedMask = new Mat();
            Core.bitwise_or(maskYellow, maskBlue, combinedMask);
            Core.bitwise_or(combinedMask, maskRed, combinedMask);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                // Filter out small boxes
                if (rotatedRect.size.width < 100 || rotatedRect.size.height < 100) continue;

                // Determine if upright or sideways
                boolean isTall = rotatedRect.size.height > rotatedRect.size.width;
                double realHeight = isTall ? realHeightTall : realHeightShort;

                // Calculate apparent height and distance
                double apparentHeight = Math.min(rotatedRect.size.width, rotatedRect.size.height);
                double distance = (focalLength * realHeight) / apparentHeight;

                // Check bounds before accessing submat
                Rect rect = rotatedRect.boundingRect();
                if (rect.x >= 0 && rect.y >= 0 && rect.x + rect.width <= hsv.cols() && rect.y + rect.height <= hsv.rows()) {
                    Mat cubeRegion = hsv.submat(rect);

                    // Determine average color
                    avgColor = Core.mean(cubeRegion);


                    telemetry.addData("Distance (cm)", distance);
                    telemetry.update();
                    String color = "Unknown";
                    if (avgColor.val[0] >= yellowLower.val[0] && avgColor.val[0] <= yellowUpper.val[0]) color = "Yellow";
                    else if (avgColor.val[0] >= blueLower.val[0] && avgColor.val[0] <= blueUpper.val[0]) color = "Blue";
                    else if ((avgColor.val[0] >= redLower1.val[0] && avgColor.val[0] <= redUpper1.val[0])
                            || (avgColor.val[0] >= redLower2.val[0] && avgColor.val[0] <= redUpper2.val[0])) color = "Red";

                    // Add telemetry


                    // Draw rotated rectangle
                    Point[] vertices = new Point[4];
                    rotatedRect.points(vertices);
                    for (int j = 0; j < 4; j++) {
                        Imgproc.line(input, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 255, 0), 2);
                    }

                    // Draw label
                    Imgproc.putText(input, color + String.format(": %.2f cm", distance, avgColor),
                            new Point(rotatedRect.center.x, rotatedRect.center.y - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 1.5, new Scalar(0, 255, 0), 2);


                    cubeRegion.release();
                }
            }

            // Release temporary Mats
            maskRed2.release();
            combinedMask.release();
            hierarchy.release();

            return input;
        }
    }

}
