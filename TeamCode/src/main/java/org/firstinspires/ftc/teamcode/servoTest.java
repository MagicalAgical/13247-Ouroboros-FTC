package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "LED Distance Sensor")
public class servoTest extends OpMode {

   // private RevBlinkinLedDriver light;
   // private TouchSensor touch;
    private DistanceSensor sensor;
    private boolean distanceMode = false;


    @Override
    public void init() {
      //  light = hardwareMap.get(RevBlinkinLedDriver.class, "light");
       sensor = hardwareMap.get(DistanceSensor.class, "sensor");


       // light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            distanceMode = true;
        } else if (gamepad1.b) {
            distanceMode = false;
            //light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }

        if (distanceMode) {
            double distance = sensor.getDistance(DistanceUnit.CM);

            if (distance > 10) {
                //light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                telemetry.addData("Mode", "Blue");
            } else {
               // light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                telemetry.addData("Mode", "Red");
            }
            telemetry.addData("Mode", "Distance-based");
            //telemetry.addData("Distance (cm)", distance);
        } else {
            telemetry.addData("Mode", "Rainbow");
        }


        telemetry.update();
    }
}
