package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto Right Preload", group = "Autonomous")
public class autoRight extends LinearOpMode {
    private DcMotor rightLift = null;
    private DcMotor leftLift = null;
    private CRServo Claw = null;

    @Override
    public void runOpMode() throws InterruptedException {
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        Claw = hardwareMap.get(CRServo.class, "Claw");

        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addLine("Ready for start Auto Right");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())

                    .build();

            drive.followTrajectorySequenceAsync(traj); // Start trajectory execution

            // Continuously update telemetry and trajectory execution
            while (opModeIsActive() && drive.isBusy()) {
                Pose2d poseEstimate = drive.getPoseEstimate(); // Get the robot's current pose
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.update();
                drive.update(); // Update trajectory logic
            }

            Pose2d finalPose = drive.getPoseEstimate(); // Final pose after trajectory execution
            telemetry.addData("Final x", finalPose.getX());
            telemetry.addData("Final y", finalPose.getY());
            telemetry.addData("Final heading", finalPose.getHeading());
            telemetry.update();
        }
    }
}
