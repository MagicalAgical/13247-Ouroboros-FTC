package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.arcrobotics.ftclib.controller.PIDController;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class PIDteleop extends LinearOpMode {
    private DcMotor leftUpper = null;
    private DcMotor leftLower = null;
    private DcMotor rightUpper = null;
    private DcMotor rightLower = null;

    private DcMotor rightLift = null;
    private DcMotor leftLift = null;

    private CRServo hlLift = null;

    private Servo Claw = null;

    private DcMotor hangRight = null;
    private DcMotor hangLeft = null;

    //private RevBlinkinLedDriver light;
    private DistanceSensor sensor;
    private boolean distanceMode = false;

    private boolean specimenMode = false;

    private static double MOTOR_ADJUST = 0.75;

    // PID controller variables
    private PIDController liftPID;
    public static double p = 0.015, i = 0.0001, d = 0.001;
    public static double f = 0.05;
    private final double ticks_in_degree = 700.0 / 180.0;
    private int liftTarget = 0;

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

        hangLeft = hardwareMap.get(DcMotor.class,"hangL");
        hangRight = hardwareMap.get(DcMotor.class,"hangR");

        Claw = hardwareMap.get(Servo.class,"Claw");

        hlLift = hardwareMap.get(CRServo.class, "hl");

        //light = hardwareMap.get(RevBlinkinLedDriver.class, "light");
        sensor = hardwareMap.get(DistanceSensor.class, "sensor");

        leftUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightUpper.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLower.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLower.setDirection(DcMotorSimple.Direction.FORWARD);
        leftUpper.setDirection(DcMotorSimple.Direction.FORWARD);

        rightLift.setDirection((DcMotorSimple.Direction.REVERSE));
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        hangRight.setDirection(DcMotorSimple.Direction.FORWARD);
        hangLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // PID setup
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftPID = new PIDController(p, i, d);

        waitForStart();
        double triggerPowerAdjust = 1;
        double speedAdjust = 1;
        double motor_power = 0.8;
        double specDist = 30;
        double highCham = 56;
        double highBasket = 819;
        double lowBasket = 85;

        while (opModeIsActive()) {
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x * 0.5;
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

            if (gamepad1.a) {
                speedAdjust = 1;
            } else if (gamepad1.x) {
                speedAdjust = 0.75;
            } else if (gamepad1.y) {
                speedAdjust = 0.5;
            }

            double distance = sensor.getDistance(DistanceUnit.CM);
            double target = highCham;
            if (gamepad2.a) {
                target = highCham;
                liftTarget = 1500;
                telemetry.addLine("High Chamber");
            } else if (gamepad2.x) {
                target = lowBasket;
                liftTarget = 400;
                telemetry.addLine("Low Basket");
            } else if (gamepad2.y) {
                target = specDist;
                liftTarget = 200;
                telemetry.addLine("Specimen Pickup");
            }

            if (distance >= target) {
                telemetry.addData("Met desired distance", "Green");
            } else {
                telemetry.addData("Not at desired distance ", "Aqua");
            }

            telemetry.addData("Distance", sensor.getDistance(DistanceUnit.CM));

            if (gamepad2.dpad_up) {
                motor_power = 0.9;
            } else if (gamepad2.dpad_down) {
                motor_power = 0.48;
            }
            telemetry.addData("Motor Power", motor_power);

            // PID lift logic
            int liftPos = rightLift.getCurrentPosition();
            liftPID.setPID(p, i, d);
            double pid = liftPID.calculate(liftPos, liftTarget);
            double ff = Math.cos(Math.toRadians(liftTarget / ticks_in_degree)) * f;
            double liftPower = pid + ff;
            rightLift.setPower(liftPower);
            leftLift.setPower(liftPower);

            if (gamepad2.right_bumper) {
                hlLift.setPower(1);
            } else if (gamepad2.left_bumper) {
                hlLift.setPower(-1);
            } else {
                hlLift.setPower(0);
            }

            if (gamepad2.right_trigger > 0.5) {
                Claw.setPosition(Claw.getPosition() + 0.05);
            } else if (gamepad2.left_trigger > 0.5) {
                Claw.setPosition(Claw.getPosition() - 0.05);
            }

            if (gamepad1.right_trigger > 0.5) {
                hangRight.setPower(0.7);
                hangLeft.setPower(0.7);
            } else if (gamepad1.left_trigger > 0.5) {
                hangRight.setPower(-0.7);
                hangLeft.setPower(-0.7);
            } else {
                hangRight.setPower(0);
                hangLeft.setPower(0);
            }

            telemetry.addData("Claw", Claw.getPosition());
            telemetry.addData("Lift Pos", liftPos);
            telemetry.addData("Lift Target", liftTarget);
            telemetry.addData("Lift Power", liftPower);
            telemetry.update();
        }

        if (isStopRequested()) {
            //light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
    }

    public void raiseLift(int value) {
        rightLift.setTargetPosition(value);
        leftLift.setTargetPosition(value);
        rightLift.setPower(0.9);
        leftLift.setPower(0.9);
    }
}
