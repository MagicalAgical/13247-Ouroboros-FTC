package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class PowerTeleOp extends LinearOpMode {
    private DcMotor leftUpper = null;
    private DcMotor leftLower = null;
    private DcMotor rightUpper = null;
    private DcMotor rightLower = null;

    private DcMotor rightLift = null;
    private DcMotor leftLift = null;

    private CRServo intake = null;

    private CRServo hlLift = null;

    private Servo Claw = null;

    private RevBlinkinLedDriver light;
    private TouchSensor touch;
    private DistanceSensor sensor;
    private boolean distanceMode = false;



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

        Claw = hardwareMap.get(Servo.class,"Claw");
       // intake = hardwareMap.get(CRServo.class, "intake");


        hlLift = hardwareMap.get(CRServo.class, "hl");

        light = hardwareMap.get(RevBlinkinLedDriver.class, "light");
        sensor = hardwareMap.get(DistanceSensor.class, "sensor");


        light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

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

        /*
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         */



        rightLift.setDirection((DcMotorSimple.Direction.REVERSE));
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

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


            if (gamepad1.right_bumper) {
                distanceMode = true;
            } else if (gamepad2.left_bumper) {
                distanceMode = false;
                light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
            }

            if (distanceMode) {
                double distance = sensor.getDistance(DistanceUnit.CM);
                double target = 10;

                if(gamepad1.dpad_up){
                    target = 30;
                }else if(gamepad1.dpad_down){
                    target = 40;
                }
                if (distance > target) {
                    light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    telemetry.addData("Met desired distance", "Blue");
                } else {
                    light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    telemetry.addData("Not at desired distance ", "Red");
                }
                telemetry.addData("Mode", "Distance-based");
                //telemetry.addData("Distance (cm)", distance);
            } else {
                telemetry.addData("Color", "Rainbow");
            }

            double motor_power = 0.8;

            if (gamepad2.left_stick_y > 0.3) {
                rightLift.setPower((motor_power));
                leftLift.setPower((motor_power));
            }
            else if (gamepad2.left_stick_y < -0.3) {
                rightLift.setPower((-motor_power));
                leftLift.setPower((-motor_power));
            }
            else {
                rightLift.setPower((0));
                leftLift.setPower((0));
            }

            if (gamepad2.dpad_down) {
                motor_power -= 0.1;
            }
            else if (gamepad2.dpad_up) {
                motor_power =+ 0.1;
            }else {
                motor_power = 0.8;
            }

           /* if(gamepad2.x){
                rightLift.setPower(0.35);
                leftLift.setPower(0.35);
            } else if (gamepad2.y) {
                rightLift.setPower(-0.35);
                leftLift.setPower(-0.35);
            }else {
                rightLift.setPower(0);
                leftLift.setPower(0);
            }

            */

            if(gamepad2.right_bumper){
                hlLift.setPower(-1);
            }
            else if (gamepad2.left_bumper){
                hlLift.setPower(1);
            }
            else{
                hlLift.setPower(0);
            }

            if(gamepad2.a){
                Claw.setPosition(0.9);
                //find position for complete close and open
                //add lift to raise up
            } else if (gamepad2.b){
                Claw.setPosition(0);
            }

          /*  if(gamepad2.right_trigger > 0.5){
                intake.setPower(0.35);
            }else if (gamepad2.left_trigger > 0.5){
                intake.setPower(-0.35);
            }else {
                intake.setPower(0);
            }

           */



            //drive movements - joystick
            //speed adjustments - gamepad1: a, b, and y

            //moving lift up - gamepad2: dpad up and moving lift down - gamepad2: dpad down
            //moving lift out - gamepad2: r bump and moving lift in gamepad2: left bumb
            //opening claw - gamepad2: a and closing claw - gamepad2: b
            //hang - gamepad1 - dpad up and down for up and down


            telemetry.addData("Claw",Claw.getPosition());
            telemetry.update();

        }
    }
    public void raiseLift(int value){
        rightLift.setTargetPosition(value);
        leftLift.setTargetPosition(value);
        rightLift.setPower(0.9);
        leftLift.setPower(0.9);
    }

}