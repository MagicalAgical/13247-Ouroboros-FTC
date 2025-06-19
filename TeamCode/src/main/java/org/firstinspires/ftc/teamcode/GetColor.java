package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@TeleOp(name = "Color Detection RGB Scalar", group = "OpenCV")
public class GetColor extends LinearOpMode {
    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);

        // Set the OpenCV pipeline
        ColorDetectionPipeline pipeline = new ColorDetectionPipeline();
        webcam.setPipeline(pipeline);

        // Open the camera asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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
            Scalar averageColor = pipeline.getAverageColor();

            telemetry.addData("Average RGB", averageColor.toString());
            telemetry.update();
        }

        // Stop the camera when the op mode stops
        webcam.stopStreaming();
    }

    static class ColorDetectionPipeline extends OpenCvPipeline {
        Mat rgbMat = new Mat();
        Scalar averageColor = new Scalar(0, 0, 0);

        @Override
        public Mat processFrame(Mat input) {
            // Process frame in RGB (input is already in RGB format)
            rgbMat = input;

            // Calculate the average RGB color
            Scalar mean = Core.mean(rgbMat);
            averageColor = new Scalar(mean.val[0], mean.val[1], mean.val[2]);

            return input; // Return the unmodified frame
        }

        public Scalar getAverageColor() {
            return averageColor;
        }
    }
}
