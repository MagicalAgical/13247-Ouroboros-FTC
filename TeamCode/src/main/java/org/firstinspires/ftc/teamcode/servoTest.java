package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class servoTest extends LinearOpMode {
    private CRServo intake;
    private CRServo hrLift;
   // private DcMotor rightLift = null;
   // private DcMotor leftLift = null;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(CRServo.class,"wheel");
     /*   leftLift = hardwareMap.get(DcMotor.class,"leftLift");
        rightLift = hardwareMap.get(DcMotor.class,"rightLift");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setDirection((DcMotorSimple.Direction.REVERSE));
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

      */



      //  hrLift = hardwareMap.get(CRServo.class,"hrLift");

       // intake = hardwareMap.get(Servo.class, "wheel");
       // double position = intake.getPosition();

        waitForStart();

        while (opModeIsActive()) {
         /*   if (gamepad1.right_trigger > 0.1) {
                position = Math.min(1.0, position + 0.01);
            } else if (gamepad1.left_trigger > 0.1) {
                position = Math.max(0.0, position - 0.01);
            }

            intake.setPosition(position);
            telemetry.addData("Servo Position", position);
            telemetry.update();

          */


           if(gamepad1.a){
                intake.setPower(0.4);
            }else if(gamepad1.b){
                intake.setPower(-0.4);
            }else{
               intake.setPower(0);
           }



            /*  if(gamepad1.x){
               hrLift.setPower(0.5);
           }else if (gamepad1.y){
               hrLift.setPower(-0.5);
           }else{
               hrLift.setPower(0);
           }

          */



           /* if(gamepad1.dpad_up){
                rightLift.setPower(1);
                leftLift.setPower(1);
            } else if (gamepad1.dpad_down) {
                rightLift.setPower(-1);
                leftLift.setPower(-1);
            }else{
                rightLift.setPower(0);
                leftLift.setPower(0);
            }

            */


        }
    }
}
