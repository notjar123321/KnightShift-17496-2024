package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Cube Detection with Distance", group = "Autonomous")
public class ComputerVisionAuto extends LinearOpMode {

    private OpenCvWebcam webcam;
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
                // Start streaming to the dashboard
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 30); // Stream at 30 FPS
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

        // Main loop
        while (opModeIsActive()) {
            telemetry.addData("FPS", webcam.getFps());
            telemetry.addData("Pipeline Time (ms)", webcam.getPipelineTimeMs());
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
        private double realHeight = 8.89; // Real-world cube height in cm

        public CubeDetectionPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        @Override
        public Mat processFrame(Mat input) {
            // Convert to HSV for color detection
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Refined HSV ranges
            Scalar yellowLower = new Scalar(22, 80, 100); // Looser range for yellow
            Scalar yellowUpper = new Scalar(32, 255, 255);
            Scalar blueLower = new Scalar(100, 150, 50);
            Scalar blueUpper = new Scalar(130, 255, 255);
            Scalar redLower1 = new Scalar(0, 150, 120);  // Darker, more saturated reds
            Scalar redUpper1 = new Scalar(10, 255, 255);
            Scalar redLower2 = new Scalar(170, 150, 120);
            Scalar redUpper2 = new Scalar(180, 255, 255);

            // Find contours for all cubes
            Mat maskYellow = new Mat();
            Mat maskBlue = new Mat();
            Mat maskRed = new Mat();

            Core.inRange(hsv, yellowLower, yellowUpper, maskYellow);
            Core.inRange(hsv, blueLower, blueUpper, maskBlue);
            Core.inRange(hsv, redLower1, redUpper1, maskRed);
            Mat maskRed2 = new Mat();
            Core.inRange(hsv, redLower2, redUpper2, maskRed2);
            Core.add(maskRed, maskRed2, maskRed);

            // Combine masks and find contours
            Mat combinedMask = new Mat();
            Core.add(maskYellow, maskBlue, combinedMask);
            Core.add(combinedMask, maskRed, combinedMask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                Rect boundingRect = Imgproc.boundingRect(contour);

                // Filter out small boxes
                if (boundingRect.width < 30 || boundingRect.height < 30) { // Skip tiny boxes
                    continue;
                }

                // Calculate apparent height in pixels
                double apparentHeight = boundingRect.height;

                // Calculate distance
                double distance = (focalLength * realHeight) / apparentHeight;

                // Determine the color
                String color = "Unknown";
                Mat cubeRegion = hsv.submat(boundingRect);
                Scalar avgColor = Core.mean(cubeRegion);

                if (avgColor.val[0] >= yellowLower.val[0] && avgColor.val[0] <= yellowUpper.val[0]) color = "Yellow";
                else if (avgColor.val[0] >= blueLower.val[0] && avgColor.val[0] <= blueUpper.val[0]) color = "Blue";
                else if ((avgColor.val[0] >= redLower1.val[0] && avgColor.val[0] <= redUpper1.val[0])
                        || (avgColor.val[0] >= redLower2.val[0] && avgColor.val[0] <= redUpper2.val[0])) color = "Red";

                // Add telemetry
                telemetry.addData("Cube Color", color);
                telemetry.addData("Distance (cm)", distance);

                // Draw bounding box and label
                Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2);
                Imgproc.putText(input, color + String.format(": %.2f cm", distance),
                        new Point(boundingRect.x, boundingRect.y - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
            }

            telemetry.update();
            return input; // Return the processed frame with annotations

        }
    }
}
