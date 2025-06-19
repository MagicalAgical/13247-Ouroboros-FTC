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
@Disabled
@Autonomous(name = "Auto Left Low", group = "Autonomous")
public class autoLeftLow extends LinearOpMode {
    private DcMotor rightLift = null;
    private DcMotor leftLift = null;
    private Servo Claw = null;
    private CRServo hl = null;
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
                    .turn(Math.toRadians(-39))
                    .strafeRight(3)
                    .addTemporalMarker(()->{
                        rightLift.setPower(0.85);
                        leftLift.setPower(0.85);
                    })
                    .addTemporalMarker(0.85,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .waitSeconds(2)
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
                    .forward(15)
                    .build();
            /*
            TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                    .addTemporalMarker(()->{
                        rightLift.setPower(0.7);
                        leftLift.setPower(0.7);
                    })
                    .addTemporalMarker(2,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .forward(2)
                            .build();

          */
            drive.followTrajectorySequence(traj);
            rightLift.setPower(0);
            leftLift.setPower(0);
            drive.followTrajectorySequence(traj2);
            Claw.setPosition(0.3);// Start trajectory execution
            drive.followTrajectorySequence(traj3);








            Pose2d finalPose = drive.getPoseEstimate(); // Final pose after trajectory execution
            telemetry.addData("Final x", finalPose.getX());
            telemetry.addData("Final y", finalPose.getY());
            telemetry.addData("Final heading", finalPose.getHeading());
            telemetry.update();
        }
    }
}
