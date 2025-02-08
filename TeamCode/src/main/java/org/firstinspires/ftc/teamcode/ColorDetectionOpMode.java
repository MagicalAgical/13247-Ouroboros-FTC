package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@TeleOp(name = "Color Detection with Telemetry", group = "OpenCV")
public class ColorDetectionOpMode extends LinearOpMode {
    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);

        // Set the OpenCV pipeline
        ColorDetectionPipeline colorPipeline = new ColorDetectionPipeline();
        webcam.setPipeline(colorPipeline);

        // Open the camera asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(130, 120, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error Code: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Detected Color", colorPipeline.getDetectedColor());
            telemetry.update();
        }

        // Stop the camera when the op mode stops
        webcam.stopStreaming();
    }

    static class ColorDetectionPipeline extends OpenCvPipeline {
        Mat hsvMat = new Mat();
        Mat mask = new Mat();
        String detectedColor = "None";

        @Override
        public Mat processFrame(Mat input) {
            // Convert the frame to HSV
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Define color ranges (HSV values)
            Scalar lowerRed = new Scalar(0, 100, 100);
            Scalar upperRed = new Scalar(10, 255, 255);

            Scalar lowerBlue = new Scalar(110, 135, 0);
            Scalar upperBlue = new Scalar(140, 255, 255);

            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);

            // Detect red
            Core.inRange(hsvMat, lowerRed, upperRed, mask);
            if (Core.countNonZero(mask) > 500) {
                detectedColor = "Red";
                return input;
            }

            // Detect blue
            Core.inRange(hsvMat, lowerBlue, upperBlue, mask);
            if (Core.countNonZero(mask) > 500) {
                detectedColor = "Blue";
                return input;
            }

            // Detect yellow
            Core.inRange(hsvMat, lowerYellow, upperYellow, mask);
            if (Core.countNonZero(mask) > 500) {
                detectedColor = "Yellow";
                return input;
            }

            // No color detected
            detectedColor = "None";
            return input;
        }

        public String getDetectedColor() {
            return detectedColor;
        }
    }
}
