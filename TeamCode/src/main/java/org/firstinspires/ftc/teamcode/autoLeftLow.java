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

import java.util.concurrent.CopyOnWriteArrayList;

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
                    .strafeRight(2.5)
                    .forward(10)
                    .turn(Math.toRadians(-43))
                    .strafeRight(2.5)
                    .addTemporalMarker(()->{
                        rightLift.setPower(0.9);
                        leftLift.setPower(0.9);
                    })
                    .waitSeconds(0.5)
                    .addTemporalMarker(1.2,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .waitSeconds(2.5)
                    .build();

           TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                   .forward(10)
                    .build();


            TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                    .waitSeconds(3)
                    .back(7)
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
            Claw.setPosition(0.3);
          drive.followTrajectorySequence(traj2);// Start trajectory execution
            drive.followTrajectorySequence(traj3);







            Pose2d finalPose = drive.getPoseEstimate(); // Final pose after trajectory execution
            telemetry.addData("Final x", finalPose.getX());
            telemetry.addData("Final y", finalPose.getY());
            telemetry.addData("Final heading", finalPose.getHeading());
            telemetry.update();
        }
    }
}
