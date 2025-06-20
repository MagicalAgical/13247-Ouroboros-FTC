package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.opencv.core.Mat;

import java.util.concurrent.CopyOnWriteArrayList;
@Autonomous(name = "Auto Left Low", group = "Autonomous")
public class autoLeftLow extends LinearOpMode {
    private DcMotor rightLift = null;
    private DcMotor leftLift = null;
    private Servo Claw = null;
    private CRServo hl = null;
    private DcMotor hangRight = null;
    private DcMotor hangLeft = null;
    //private RevBlinkinLedDriver light;

    @Override

    public void runOpMode() throws InterruptedException {
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        Claw = hardwareMap.get(Servo.class, "Claw");
        hl = hardwareMap.get(CRServo.class,"hl");

        rightLift.setDirection((DcMotorSimple.Direction.REVERSE));
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangLeft = hardwareMap.get(DcMotor.class,"hangL");
        hangRight = hardwareMap.get(DcMotor.class,"hangR");

        hangLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangRight.setDirection(DcMotorSimple.Direction.FORWARD);
        hangLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // light = hardwareMap.get(RevBlinkinLedDriver.class,"light");


        telemetry.addLine("Ready for start Auto Left Low");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw.setPosition(0.99);
        //light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        waitForStart();
        if (opModeIsActive()) {
            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .strafeRight(2)
                    .forward(9)
                    .turn(Math.toRadians(-40))
                    .strafeRight(3)
                    .addTemporalMarker(()->{
                        rightLift.setPower(0.9);
                        leftLift.setPower(0.9);
                    })
                    .addTemporalMarker(0.92,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .waitSeconds(1.7)
                    .build();

           TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                   .forward(11)
                   .build();


           TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                    .back(7)
                    .turn(Math.toRadians(170))
                    .addTemporalMarker(()->{
                        rightLift.setPower(-0.6);
                        leftLift.setPower(-0.6);
                        hl.setPower(0.1);
                    })
                    .addTemporalMarker(1,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .strafeRight(1.3)
                    //.forward(15)  // change if doing park auto
                    .build();

           TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                    .waitSeconds(1)
                    .turn(Math.toRadians(195))
                    .forward(9)
                    .build();

           TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d())
                   .strafeRight(3)
                   .addTemporalMarker(()->{
                       hangRight.setPower(1);
                       hangLeft.setPower(1);
                   })
                   .turn(Math.toRadians(90))
                   .forward(13)
                   .addTemporalMarker(2,()->{
                       hangLeft.setPower(0);
                       hangRight.setPower(0);
                   })
                    .build();


           drive.followTrajectorySequence(traj);
           rightLift.setPower(0);
           leftLift.setPower(0);
           drive.followTrajectorySequence(traj2);
           Claw.setPosition(0.3);
           drive.followTrajectorySequence(traj3);
           Claw.setPosition(0.9);
           drive.followTrajectorySequence(park);








           Pose2d finalPose = drive.getPoseEstimate();
           telemetry.addData("Final x", finalPose.getX());
           telemetry.addData("Final y", finalPose.getY());
           telemetry.addData("Final heading", finalPose.getHeading());
           telemetry.update();
        }
    }
}
