package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "Auto Right Preload", group = "Autonomous")
public class autoRight extends LinearOpMode {
  private DcMotor rightLift = null;
  private DcMotor leftLift = null;
  private CRServo Claw = null;


    @Override
    public void runOpMode() throws InterruptedException {
        rightLift = hardwareMap.get(DcMotor.class,"rightLift");
        leftLift = hardwareMap.get(DcMotor.class,"leftLift");


        Claw = hardwareMap.get(CRServo.class, "Claw");

      /*  rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       */
        rightLift.setDirection((DcMotorSimple.Direction.FORWARD));
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //rightLift.setTargetPosition(0);
        //leftLift.setTargetPosition(0);




        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addLine("Ready for start Auto Right");
        telemetry.update();
        Claw.setPower(0.6);
        sleep(300);
        Claw.setPower(0);
        rightLift.setPower(-0.35);
        leftLift.setPower(0.35);
        sleep(400);
        rightLift.setPower(0);
        leftLift.setPower(0);//close
        waitForStart();
        if(opModeIsActive()) {
            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .addTemporalMarker(() -> {
                        rightLift.setPower(-1);
                        leftLift.setPower(1);
                    })
                    .lineToLinearHeading(new Pose2d(20,12))
                    .waitSeconds(3)
                    .addTemporalMarker(3.5,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .lineToLinearHeading(new Pose2d(37,12))
                    .addTemporalMarker(6,()->{
                        rightLift.setPower(1);
                        leftLift.setPower(-1);
                    })
                    .addTemporalMarker(6.9,()->{
                        Claw.setPower(-0.6);
                    })
                    .waitSeconds(1.5)
                    .addTemporalMarker(7.3,()->{
                        Claw.setPower(0);
                    })

                    .addTemporalMarker(9,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .back(28)
                    .strafeRight(60)
                    .build();
           /* TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(35,10))
                    .build();
            */
            drive.followTrajectorySequence(traj);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();


        }
    }
}