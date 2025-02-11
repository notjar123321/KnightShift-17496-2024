package org.firstinspires.ftc.teamcode.AutoModes;

import android.content.res.AssetManager;

import com.acmerobotics.dashboard.FtcDashboard; // Import FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.tensorflow.lite.Interpreter;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

@Autonomous(name = "Rectangle Detection Auto Without TFOD", group = "Concept")
public class RectangleTest extends LinearOpMode {

    private static final String MODEL_FILE_NAME = "chunky.tflite";
    private Interpreter tflite;
    private OpenCvCamera camera;
    private FtcDashboard dashboard; // Dashboard instance

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance(); // Initialize the dashboard

        initCamera();
        initTFLite();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

        if (camera != null) {
            camera.stopStreaming();
        }
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                Mat grayscale = new Mat();
                Imgproc.cvtColor(input, grayscale, Imgproc.COLOR_RGB2GRAY);
                Imgproc.rectangle(input, new Rect(100, 100, 200, 200), new Scalar(0, 255, 0), 2);
                return input;
            }
        });

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

                // Stream camera output to the dashboard
                dashboard.startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error Code: " + errorCode);
                telemetry.update();
            }
        });
    }

    private void initTFLite() {
        try {
            ByteBuffer modelBuffer = loadModelFile(MODEL_FILE_NAME);
            tflite = new Interpreter(modelBuffer);
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to load TFLite model");
        }
    }

    private ByteBuffer loadModelFile(String modelName) throws Exception {
        AssetManager assetManager = hardwareMap.appContext.getAssets();
        InputStream inputStream = assetManager.open(modelName);
        byte[] model = new byte[inputStream.available()];
        inputStream.read(model);
        ByteBuffer buffer = ByteBuffer.allocateDirect(model.length).order(ByteOrder.nativeOrder());
        buffer.put(model);
        inputStream.close();
        return buffer;
    }
}
