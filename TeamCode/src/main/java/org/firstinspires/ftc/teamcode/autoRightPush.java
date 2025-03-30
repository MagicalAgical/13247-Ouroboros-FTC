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

@Autonomous(name = "Auto Push", group = "Autonomous")
public class autoRightPush extends LinearOpMode {
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
                    .strafeRight(5.5)
                    .forward(35.5)
                    .strafeRight(2.8)
                    .turn(Math.toRadians(15))
                    .back(32.5)
                    .turn(Math.toRadians(-3))
                    .forward(32.5)
                    .strafeRight(1.8)
                    .turn(Math.toRadians(-15))
                    .back(32)
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
