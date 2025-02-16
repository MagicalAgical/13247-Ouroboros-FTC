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

@Autonomous(name = "Auto Left Cycle", group = "Autonomous")
public class autoLeftCycle extends LinearOpMode {
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


        telemetry.addLine("Ready for start Auto Left Cycle");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw.setPosition(0.99);
        waitForStart();
        if (opModeIsActive()) {
            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .addTemporalMarker(()->{
                        //light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                        hl.setPower(0.75);
                        Claw.setPosition(0.95);
                        rightLift.setPower(.95);
                        leftLift.setPower(0.95);
                    })
                    .lineToLinearHeading(new Pose2d(21.5,-1.5))
                    .waitSeconds(0.2)
                    .addTemporalMarker(1.9,()->{
                        hl.setPower(0);
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .forward(6.5)
                    .waitSeconds(0.5)
                    .addTemporalMarker(4.5,()->{
                        rightLift.setPower(-0.4);
                        leftLift.setPower(-0.4);
                    })
                    .waitSeconds(0.9)
                    .addTemporalMarker(4.88,()->{
                        //light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        Claw.setPosition(0.3);
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .back(3.5)
                    .addDisplacementMarker(()->{
                        //light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                    })
                   /* .strafeLeft(8.5)
                    .forward(25)
                    .turn(Math.toRadians(90))
                    .forward(15)

                    */
                    .waitSeconds(1)
                    //.strafeLeft(18.5)
                    .lineToLinearHeading(new Pose2d(27.5,15))
                    .turn(Math.toRadians(15))
                    .waitSeconds(1)
                     .addTemporalMarker(12,()->{
                         rightLift.setPower(-0.55);
                         leftLift.setPower(-0.55);
                     })
                     .addTemporalMarker(13.7,()->{
                         rightLift.setPower(0);
                         leftLift.setPower(0);
                     })
                    .waitSeconds(3)
                    .forward(10)
                    .build();

            TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                    .waitSeconds(1.5)
                    .turn(Math.toRadians(190))
                    .addTemporalMarker(()->{
                        rightLift.setPower(0.8);
                        leftLift.setPower(0.8);
                    })
                    .addTemporalMarker(1.5,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .waitSeconds(1)
                    .forward(10)
                    .build();
            /*

            TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                    .addTemporalMarker(()->{
                        rightLift.setPower(0.7);
                        leftLift.setPower(0.7);
                    })
                    .addTemporalMarker(1,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .waitSeconds(1.5)
                    .back(10)
                    .turn(Math.toRadians(210))
                    .forward(5)
                    .build();
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
            Claw.setPosition(0.99);
           drive.followTrajectorySequence(traj2);
           rightLift.setPower(0);
           leftLift.setPower(0);
           Claw.setPosition(0.3);// Start trajectory execution
            //drive.followTrajectorySequence(traj3);
            //drive.followTrajectorySequence(traj4);





            Pose2d finalPose = drive.getPoseEstimate(); // Final pose after trajectory execution
            telemetry.addData("Final x", finalPose.getX());
            telemetry.addData("Final y", finalPose.getY());
            telemetry.addData("Final heading", finalPose.getHeading());
            telemetry.update();
        }
    }
}
