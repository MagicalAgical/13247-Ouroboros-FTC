package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends LinearOpMode {
    private CRServo ClawRight = null;
    private CRServo ClawLeft = null;
    @Override
    public void runOpMode() throws InterruptedException {
       ClawRight = hardwareMap.get(CRServo.class,"hrLift");
       ClawLeft = hardwareMap.get(CRServo.class,"hlLift");


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x){
                ClawRight.setPower(0.6);
                ClawLeft.setPower(-0.6);
                sleep(50);
            }else if (gamepad1.y){
                ClawRight.setPower(-0.6);
                ClawLeft.setPower(0.6);
                sleep(0);
            }else{
                ClawRight.setPower(0);
                ClawLeft.setPower(0);
            }
        }
    }
}
