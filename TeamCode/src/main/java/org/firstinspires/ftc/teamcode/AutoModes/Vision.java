package org.firstinspires.ftc.teamcode.AutoModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.tensorflow.lite.Interpreter;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.io.InputStream;
import android.content.res.AssetManager;
import java.io.IOException;

@Config
@Autonomous(name = "CameraTestReal", group = "Autonomous")
public class Vision extends LinearOpMode {
    private OpenCvWebcam webcam;
    private Interpreter interpreter;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Step 1: Validate hardwareMap for Webcam
        try {
            WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            telemetry.addData("Webcam", "Found: " + (webcamName != null));
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Error", "Webcam not found in hardwareMap: " + e.getMessage());
            telemetry.update();
            return;
        }

        // Step 2: Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        if (webcam != null) {
            telemetry.addData("Camera", "Initialized Successfully");
        } else {
            telemetry.addData("Camera", "Initialization Failed");
            telemetry.update();
            return;
        }
        telemetry.update();

        // Step 3: Load TensorFlow Lite model
        telemetry.addData("Model", "Loading...");
        telemetry.update();
        AssetManager assetManager = hardwareMap.appContext.getAssets();
        try (InputStream is = assetManager.open("chunky.tflite")) {
            byte[] modelBytes = null;
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.TIRAMISU) {
                modelBytes = is.readAllBytes();
            }
            ByteBuffer modelBuffer = ByteBuffer.wrap(modelBytes);
            interpreter = new Interpreter(modelBuffer);
            telemetry.addData("Model", "Loaded Successfully");
        } catch (IOException e) {
            telemetry.addData("Model Error", "Failed to load model: " + e.getMessage());
            telemetry.update();
            return;
        }
        telemetry.update();

        // Step 4: Set the OpenCvPipeline for the webcam
        try {
            webcam.setPipeline(new CustomPipeline(interpreter));
            telemetry.addData("Pipeline", "Pipeline set successfully");
        } catch (Exception e) {
            telemetry.addData("Pipeline Error", "Failed to set pipeline: " + e.getMessage());
            telemetry.update();
            return;
        }
        telemetry.update();

        // Step 5: Open the camera and start streaming
        telemetry.addData("Camera", "Opening...");
        telemetry.update();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera", "Streaming Started");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Failed to open: Error code " + errorCode);
                telemetry.update();
            }
        });

        // Wait for the start of the OpMode
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        // Runtime loop
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            sleep(50); // Reduce CPU usage
        }
    }

    // Custom OpenCvPipeline for processing frames
    static class CustomPipeline extends OpenCvPipeline {
        private final Interpreter interpreter;

        public CustomPipeline(Interpreter interpreter) {
            this.interpreter = interpreter;
        }

        @Override
        public Mat processFrame(Mat input) {
            // Step 1: Preprocess the frame
            Mat resizedFrame = new Mat();
            Imgproc.resize(input, resizedFrame, new Size(640, 640)); // Resize to model input size
            Imgproc.cvtColor(resizedFrame, resizedFrame, Imgproc.COLOR_BGR2RGB); // Convert to RGB

            ByteBuffer inputTensor = ByteBuffer.allocateDirect(1 * 640 * 640 * 3 * 4); // Float32 buffer
            inputTensor.order(ByteOrder.nativeOrder());

            for (int y = 0; y < resizedFrame.rows(); y++) {
                for (int x = 0; x < resizedFrame.cols(); x++) {
                    double[] pixel = resizedFrame.get(y, x);
                    for (double channel : pixel) {
                        inputTensor.putFloat((float) (channel / 255.0)); // Normalize to [0, 1]
                    }
                }
            }

            // Step 2: Run inference
            float[][][] outputTensor = new float[1][7][8400]; // Shape of the model output
            interpreter.run(inputTensor, outputTensor);

            // Step 3: Postprocess the output
            for (int i = 0; i < outputTensor[0][0].length; i++) {
                float confidence = outputTensor[0][4][i]; // Confidence score
                if (confidence > 0.5) { // Filter by confidence threshold
                    float xCenter = outputTensor[0][0][i];
                    float yCenter = outputTensor[0][1][i];
                    float width = outputTensor[0][2][i];
                    float height = outputTensor[0][3][i];
                    int classId = (int) outputTensor[0][5][i];

                    // Convert normalized coordinates to pixel values
                    int x = (int) ((xCenter - width / 2) * input.width());
                    int y = (int) ((yCenter - height / 2) * input.height());
                    int boxWidth = (int) (width * input.width());
                    int boxHeight = (int) (height * input.height());

                    // Draw the bounding box and label on the frame
                    Imgproc.rectangle(input, new Point(x, y), new Point(x + boxWidth, y + boxHeight), new Scalar(0, 255, 0), 2);
                    Imgproc.putText(input, "Class " + classId + " (" + confidence + ")", new Point(x, y - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                }
            }

            // Return the frame with annotations
            return input;
        }


        // Draw bounding boxes on the frame
        private void drawBoundingBoxes(Mat frame, float[][] output) {
            for (float[] detection : output) {
                float x = detection[0];
                float y = detection[1];
                float width = detection[2];
                float height = detection[3];
                float confidence = detection[4];
                int classId = (int) detection[5];

                if (confidence > 0.5) { // Confidence threshold
                    Point topLeft = new Point(x - width / 2, y - height / 2);
                    Point bottomRight = new Point(x + width / 2, y + height / 2);

                    // Draw a rectangle around the detected object
                    Imgproc.rectangle(frame, topLeft, bottomRight, new Scalar(0, 255, 0), 2);
                    Imgproc.putText(frame, "Class " + classId, topLeft, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);
                }
            }
        }
    }
}
