package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
@Disabled
public class PIDLift extends OpMode {

    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;
    public static double ff2 = 0;

    private DcMotorEx leftLift;
    private DcMotorEx rightLift;

    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Hardware mapping
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");


        telemetry.addLine("Initialized. Ready to run.");
    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);

        int rightpos = rightLift.getCurrentPosition();

        double pid = controller.calculate(rightpos,target);
        double ff = ff2;

        double power = pid + ff;

        rightLift.setPower(power);
        leftLift.setPower(power);

        telemetry.addData("right pos ", rightpos);
        telemetry.addData("target ", target);
        telemetry.update();

    }

    @Override
    public void stop() {
        leftLift.setPower(0);
        rightLift.setPower(0);
    }
}
