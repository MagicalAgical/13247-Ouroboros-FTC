package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class EncoderTeleOp extends LinearOpMode {
    private DcMotor leftUpper = null;
    private DcMotor leftLower = null;
    private DcMotor rightUpper = null;
    private DcMotor rightLower = null;

    private DcMotor rightLift = null;
    private DcMotor leftLift = null;

    // private CRServo leftClaw = null;
    //private CRServo rightClaw = null;

    //private CRServo Claw = null;

    private CRServo hlLift = null;
    private CRServo hrLift = null;

    private CRServo Claw = null;



    private static double MOTOR_ADJUST = 0.75;

    @Override
    public void runOpMode() {
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        leftUpper = hardwareMap.get(DcMotor.class, "lFront");
        rightUpper = hardwareMap.get(DcMotor.class, "rFront");
        leftLower = hardwareMap.get(DcMotor.class, "lBack");
        rightLower = hardwareMap.get(DcMotor.class, "rBack");

        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");

        //leftClaw = hardwareMap.get(CRServo.class, "rClaw");
        //rightClaw = hardwareMap.get(CRServo.class, "lClaw");

        Claw = hardwareMap.get(CRServo.class,"Claw");

        hrLift = hardwareMap.get(CRServo.class, "hrLift");
        hlLift = hardwareMap.get(CRServo.class, "hlLift");

        //Claw = hardwareMap.get(CRServo.class, "Claw");

        leftUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightUpper.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLower.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLower.setDirection(DcMotorSimple.Direction.REVERSE);
        leftUpper.setDirection(DcMotorSimple.Direction.REVERSE);

        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        rightLift.setDirection((DcMotorSimple.Direction.REVERSE));
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);


        rightLift.setTargetPosition(0);
        leftLift.setTargetPosition(0);


        waitForStart();
        double triggerPowerAdjust = 1;
        double speedAdjust = 1;

        while (opModeIsActive()) {
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x * 0.5;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            v1 = (v1 * triggerPowerAdjust * -1) * speedAdjust;
            v2 = (v2 * triggerPowerAdjust * -1) * speedAdjust;
            v3 = (v3 * triggerPowerAdjust * -1) * speedAdjust;
            v4 = (v4 * triggerPowerAdjust * -1) * speedAdjust;
            leftUpper.setPower(v1 * 1);
            rightUpper.setPower(v2 * 1);
            leftLower.setPower(v3 * 1);
            rightLower.setPower(v4 * 1);

            if (gamepad1.y) {
                speedAdjust = 1;
            }

            if (gamepad1.b) {
                speedAdjust = 0.75;
            }

            if (gamepad1.a) {
                speedAdjust = 0.50;
            }

            if (gamepad2.dpad_up) {
                rightLift.setTargetPosition(rightLift.getCurrentPosition() + 200);
                leftLift.setTargetPosition(leftLift.getCurrentPosition() + 200);
                rightLift.setPower(1);
                leftLift.setPower(1);
            }else if (gamepad2.dpad_down) {
                rightLift.setTargetPosition(rightLift.getCurrentPosition() - 200);
                leftLift.setTargetPosition(leftLift.getCurrentPosition() - 200);
                rightLift.setPower(1);
                leftLift.setPower(1);
            }else{
                rightLift.setPower(0);
                leftLift.setPower(0);
            }

            if(gamepad2.right_bumper){
                hrLift.setPower(0.9);
                hlLift.setPower(0.9);
                sleep(50);
            }
            else if (gamepad2.left_bumper){
                hrLift.setPower(-0.9);
                hlLift.setPower(-0.9);
                sleep(50);
            }
            else{
                hrLift.setPower(0);
                hlLift.setPower(0);
            }

            if (gamepad2.a){

            }else if (gamepad2.b){

            }else{

            }



            if(gamepad2.a){
                Claw.setPower(0.6);
            } else if (gamepad2.b){
                Claw.setPower(-0.6);
            }else{
                Claw.setPower(0);
            }



            //drive movements - joystick
            //speed adjustments - gamepad1: a, b, and y

            //moving lift up - gamepad2: dpad up and moving lift down - gamepad2: dpad down
            //moving lift out - gamepad1: a and moving lift in gamepad1: b
            //opening claw - gamepad2: right_bumper and closing claw - gamepad2: left_bumper



            telemetry.update();

        }
    }
    public void raiseLift(double tick){
        int value = (int) tick;
        rightLift.setTargetPosition(value);
        leftLift.setTargetPosition(value);
        rightLift.setPower(0.9);
        leftLift.setPower(0.9);
    }

}