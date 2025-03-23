package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Auto Right Cycle 2", group = "Autonomous")
public class autoRightCycle2 extends LinearOpMode {
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


        telemetry.addLine("Ready for start Auto Right 2");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw.setPosition(0.99);
        //light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        waitForStart();
        if (opModeIsActive()) {
            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .addTemporalMarker(()->{
                        rightLift.setPower(1);
                        leftLift.setPower(1);
                    })
                    .lineToLinearHeading(new Pose2d(25,0,Math.toRadians(0)))
                    .addTemporalMarker(1.8,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                        Claw.setPosition(0.3);
                    })
                    .waitSeconds(0.5)
                    .addTemporalMarker(1.9,()->{
                        rightLift.setPower(-0.5);
                        leftLift.setPower(-0.5);
                    })
                    .addTemporalMarker(2.15,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .addTemporalMarker(2.7,()->{
                        rightLift.setPower(-0.38);
                        leftLift.setPower(-0.38);
                    })
                    .lineToLinearHeading(new Pose2d(15,-10,Math.toRadians(180)))
                    .addTemporalMarker(2.8,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .forward(10.5)
                    .build();

           /* TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                    .waitSeconds(1)
                    .addTemporalMarker(()->{
                        rightLift.setPower(0.8);
                        leftLift.setPower(0.8);
                    })
                    .waitSeconds(1)
                    .back(10)
                    .turn(Math.toRadians(180))
                    .strafeLeft(11)
                    .addTemporalMarker(1.3,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .addTemporalMarker(1.5,()->{
                        rightLift.setPower(0.7);
                        leftLift.setPower(0.7);
                    })
                    .addTemporalMarker(1.8,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .build();

            TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                    .addTemporalMarker(()->{
                        rightLift.setPower(0.9);
                        leftLift.setPower(0.9);
                    })
                    .addTemporalMarker(1,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .forward(10)
                    .addTemporalMarker(1.3,()->{
                        rightLift.setPower(-0.5);
                        leftLift.setPower(-0.5);
                    })
                    .addTemporalMarker(1.45,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .back(10)
                    .build();

            */
            drive.followTrajectorySequence(traj);
            Claw.setPosition(0.99);
            //drive.followTrajectorySequence(traj2);
            //drive.followTrajectorySequence(traj3);






            Pose2d finalPose = drive.getPoseEstimate(); // Final pose after trajectory execution
            telemetry.addData("Final x", finalPose.getX());
            telemetry.addData("Final y", finalPose.getY());
            telemetry.addData("Final heading", finalPose.getHeading());
            telemetry.update();
        }
    }
}
