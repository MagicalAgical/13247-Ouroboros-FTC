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
        //light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        waitForStart();
        if (opModeIsActive()) {
            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .addTemporalMarker(()->{
                        rightLift.setPower(0.98);
                        leftLift.setPower(0.98);
                    })

                    .lineToLinearHeading(new Pose2d(25,0.7,Math.toRadians(0)))
                    .addTemporalMarker(1.47,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .addTemporalMarker(1.6,()->{
                        rightLift.setPower(-0.45);
                        leftLift.setPower(-0.45);
                    })
                    .addTemporalMarker(1.9,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                        Claw.setPosition(0.3);
                    })
                    .back(10)
                    .build();

            TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d())
                    .addTemporalMarker(()->{
                        rightLift.setPower(-0.315);
                        leftLift.setPower(-0.315);
                    })
                    .lineToLinearHeading(new Pose2d(-13,-12.55242,Math.toRadians(0)))
                    .addTemporalMarker(0.185,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .turn(Math.toRadians(180))
                    .forward(13.4)
                    .build();
            TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d())
                    .waitSeconds(1)
                    .addTemporalMarker(1.2,()->{
                        rightLift.setPower(0.7);
                        leftLift.setPower(0.7);
                    })
                    .addTemporalMarker(1.6,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .waitSeconds(0.2)
                    .turn(Math.toRadians(180))
                    .forward(10)
                    .strafeLeft(23)
                    .addTemporalMarker(()->{
                        rightLift.setPower(0.61);
                        leftLift.setPower(0.61);
                    })
                    .addTemporalMarker(2,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .waitSeconds(2.5)
                    .forward(17)
                    .build();


            TrajectorySequence traj4 = drive.trajectorySequenceBuilder(new Pose2d())
                    .addTemporalMarker(()->{
                        rightLift.setPower(-0.4);
                        leftLift.setPower(-0.4);
                    })
                    .addTemporalMarker(0.27,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                        Claw.setPosition(0.3);
                    })
                    .waitSeconds(2.1)
                    .back(2)
                    .build();


            drive.followTrajectorySequence(traj);
            drive.followTrajectorySequence(traj2);
            Claw.setPosition(1);
            drive.followTrajectorySequence(traj3);
            drive.followTrajectorySequence(traj4);






            Pose2d finalPose = drive.getPoseEstimate(); // Final pose after trajectory execution
            telemetry.addData("Final x", finalPose.getX());
            telemetry.addData("Final y", finalPose.getY());
            telemetry.addData("Final heading", finalPose.getHeading());
            telemetry.update();
        }
    }
}