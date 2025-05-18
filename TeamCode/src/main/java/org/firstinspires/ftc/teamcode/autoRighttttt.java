package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto Right Yeah", group = "Autonomous")
public class autoRighttttt extends LinearOpMode {
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


        telemetry.addLine("Ready for start Auto Right Yeah");
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

                    .lineToLinearHeading(new Pose2d(26.2,0.7,Math.toRadians(0)))
                    .addTemporalMarker(1.44563,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .addTemporalMarker(1.6,()->{
                        rightLift.setPower(-0.45);
                        leftLift.setPower(-0.45);
                    })
                    .addTemporalMarker(1.945,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                        Claw.setPosition(0.3);
                    })
                    .back(10)
                    .build();

            TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d())
                    .addTemporalMarker(()->{
                        rightLift.setPower(-0.45);
                        leftLift.setPower(-0.45);
                    })
                    .lineToLinearHeading(new Pose2d(-13,-12.55242,Math.toRadians(0)))
                    .addTemporalMarker(0.39,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .turn(Math.toRadians(178))
                    .forward(14.5)
                    .build();
                    /*
                    .forward(5)
                    .addDisplacementMarker(()->{
                        Claw.setPosition(0.99);
                    })
                    .addTemporalMarker(2.5,()->{
                        rightLift.setPower(0.7);
                        leftLift.setPower(0.7);
                    })
                    .waitSeconds(2)
                    .addTemporalMarker(2.8,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .lineToLinearHeading(new Pose2d(0,0))

                     */
            TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d())
                    .waitSeconds(1.5)
                    .addTemporalMarker(()->{
                        rightLift.setPower(0.6);
                        leftLift.setPower(0.6);
                    })
                    .addTemporalMarker(0.5,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .waitSeconds(1)
                    .back(4)
                    .build();




/*
            TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                    .waitSeconds(1)
                    .addTemporalMarker(()->{
                        rightLift.setPower(0.8);
                        leftLift.setPower(0.8);
                    })
                    .waitSeconds(1)
                    .back(14)
                    .turn(Math.toRadians(175))
                    .strafeLeft(11)
                    .addTemporalMarker(1.15,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .addTemporalMarker(1.35,()->{
                        rightLift.setPower(0.7);
                        leftLift.setPower(0.7);
                    })
                    .addTemporalMarker(1.65,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .build();

            TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                    .addTemporalMarker(()->{
                        rightLift.setPower(0.905);
                        leftLift.setPower(0.905);
                    })
                    .addTemporalMarker(0.8,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .addTemporalMarker(1.2,()->{
                        rightLift.setPower(-0.55);
                        leftLift.setPower(-0.55);
                    })
                    .forward(13.2)
                    .addTemporalMarker(1.33,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                        Claw.setPosition(0.3);
                    })
                    .back(10)
                    .build();

 */
            drive.followTrajectorySequence(traj);
            drive.followTrajectorySequence(traj2);
            Claw.setPosition(0.99);
            drive.followTrajectorySequence(traj3);






            Pose2d finalPose = drive.getPoseEstimate(); // Final pose after trajectory execution
            telemetry.addData("Final x", finalPose.getX());
            telemetry.addData("Final y", finalPose.getY());
            telemetry.addData("Final heading", finalPose.getHeading());
            telemetry.update();
        }
    }
}
