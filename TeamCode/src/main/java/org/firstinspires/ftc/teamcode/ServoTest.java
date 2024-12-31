package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends LinearOpMode {
    private CRServo Lift = null;
    @Override
    public void runOpMode() throws InterruptedException {
        Lift = hardwareMap.get(CRServo.class,"LiftS");


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x){
                Lift.setPower(0.6);
                sleep(50);
            }else if (gamepad1.y){
                Lift.setPower(-0.6);
                sleep(50);
            }else{
                Lift.setPower(0);
            }
        }
    }
}
