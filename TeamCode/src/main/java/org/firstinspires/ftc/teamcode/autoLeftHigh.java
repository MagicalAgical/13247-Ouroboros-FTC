package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto Left High", group = "Autonomous")
public class autoLeftHigh extends LinearOpMode {
    private DcMotor rightLift = null;
    private DcMotor leftLift = null;
    private Servo Claw = null;


    @Override
    public void runOpMode() throws InterruptedException {
        rightLift = hardwareMap.get(DcMotor.class,"rightLift");
        leftLift = hardwareMap.get(DcMotor.class,"leftLift");


        Claw = hardwareMap.get(Servo.class, "Claw");

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
        telemetry.addLine("Ready for start Auto Left");
        telemetry.update();
        Claw.setPosition(0.27);
        rightLift.setPower(-0.35);
        leftLift.setPower(0.35);
        sleep(400);
        rightLift.setPower(0);
        leftLift.setPower(0);//close
        waitForStart();
        if(opModeIsActive()) {
            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                    /*  .addTemporalMarker(() -> {
                          rightLift.setTargetPosition(-400);
                          leftLift.setTargetPosition(400);
                          rightLift.setPower(1);
                          leftLift.setPower(1);
                      })
                     */
                    .lineToLinearHeading(new Pose2d(28,-22,Math.toRadians(45)))
                    .addTemporalMarker(3,()->{
                        rightLift.setPower(-1);
                        leftLift.setPower(1);
                    })
                    .waitSeconds(9)
                    .addTemporalMarker(10.25,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .forward(15)
                    .waitSeconds(1)
                    .addDisplacementMarker(()->{
                        Claw.setPosition(0);
                    })
                    .back(18)
                    .addTemporalMarker(14,()->{
                        rightLift.setPower(1);
                        leftLift.setPower(-1);
                    })
                    .waitSeconds(4)
                    .lineToLinearHeading(new Pose2d(10,-75))
                    .addTemporalMarker(19,()->{
                        rightLift.setPower(0);
                        leftLift.setPower(0);
                    })
                    .back(17)
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
            telemetry.addData("Right Lift", rightLift.getCurrentPosition());
            telemetry.addData("Left Lift", leftLift.getCurrentPosition());
            telemetry.update();


        }
    }
}