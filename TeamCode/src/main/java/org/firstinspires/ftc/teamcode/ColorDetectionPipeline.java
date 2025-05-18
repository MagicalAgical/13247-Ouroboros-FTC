package org.firstinspires.ftc.teamcode;

import androidx.appcompat.app.ActionBar;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
@Disabled
public class ColorDetectionPipeline extends OpenCvPipeline {
    private volatile boolean redDetected = false;
    private volatile boolean blueDetected = false;
    private volatile boolean yellowDetected = false;

    Mat hsvMat = new Mat();
    Mat mask = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // Convert the frame to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Define color ranges (HSV values)
        Scalar lowerRed = new Scalar(0, 100, 100);
        Scalar upperRed = new Scalar(10, 255, 255);

        Scalar lowerBlue = new Scalar(100, 150, 0);
        Scalar upperBlue = new Scalar(140, 255, 255);

        Scalar lowerYellow = new Scalar(20, 100, 100);  // Lower bound of yellow in HSV
        Scalar upperYellow = new Scalar(30, 255, 255);  // Upper bound of yellow in HSV

        // Detect red color
        Mat redMask = new Mat();
        Core.inRange(hsvMat, lowerRed, upperRed, redMask);
        redDetected = Core.countNonZero(redMask) > 500; // Adjust threshold as needed

        // Detect blue color
        Mat blueMask = new Mat();
        Core.inRange(hsvMat, lowerBlue, upperBlue, blueMask);
        blueDetected = Core.countNonZero(blueMask) > 500;

        // Detect yellow color (Fixed the variable name)
        Mat yellowMask = new Mat();
        Core.inRange(hsvMat, lowerYellow, upperYellow, yellowMask);
        yellowDetected = Core.countNonZero(yellowMask) > 500;

        return input; // Return the original frame (or processed output if needed)
    }

    public boolean isRedDetected() {
        return redDetected;
    }

    public boolean isBlueDetected() {
        return blueDetected;
    }

    public boolean isYellowDetected() {
        return yellowDetected;
    }
}
