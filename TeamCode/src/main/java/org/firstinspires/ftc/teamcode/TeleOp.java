package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    private DcMotor leftUpper = null;
    private DcMotor leftLower = null;
    private DcMotor rightUpper = null;
    private DcMotor rightLower = null;
    private DcMotor armMotor = null;
    private DcMotor liftMotor = null;

    private CRServo Claw = null;
    private Servo Turn = null;

    private static double MOTOR_ADJUST = 0.75;
    public static final int ARM_COUNTS_PER_MOTOR_REV = 1996;
    public static final double ARM_DEGREES_PER_COUNT = 360.0 / ARM_COUNTS_PER_MOTOR_REV;

    public double armTargetAngle = 66.0;

    private static final int LIFT_TICKS_PER_STEP = 1; // Adjust as needed
    private int liftMaxPosition = -350;

    @Override public void runOpMode() {
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        leftUpper = hardwareMap.get(DcMotor.class, "leftUpper");
        rightUpper = hardwareMap.get(DcMotor.class, "rightUpper");
        leftLower = hardwareMap.get(DcMotor.class, "leftLower");
        rightLower = hardwareMap.get(DcMotor.class,"rightLower");
        armMotor = hardwareMap.get(DcMotor.class, "Arm");
        liftMotor = hardwareMap.get(DcMotor.class, "Lift");

        Claw = hardwareMap.get(CRServo.class, "Claw");
        Turn = hardwareMap.get(Servo.class, "Turn");

        armMotor.setTargetPosition(360);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightUpper.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLower.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLower.setDirection(DcMotorSimple.Direction.REVERSE);
        leftUpper.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        setArmPosition(armTargetAngle);

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

            v1 = (v1 * triggerPowerAdjust * -0.8) * speedAdjust;
            v2 = (v2 * triggerPowerAdjust * -0.8) * speedAdjust;
            v3 = (v3 * triggerPowerAdjust * -0.8) * speedAdjust;
            v4 = (v4 * triggerPowerAdjust * -0.8) * speedAdjust;
            leftUpper.setPower(v1 * 0.8);
            rightUpper.setPower(v2 * 0.8);
            leftLower.setPower(v3 * 0.8);
            rightLower.setPower(v4 * 0.8);

            if (gamepad1.y) {
                speedAdjust = 1;
            }

            if (gamepad1.a) {
                speedAdjust = 0.75;
            }

            if (gamepad1.b) {
                speedAdjust = 0.25;
            }

            if(gamepad2.left_bumper){
                Turn.setPosition(Turn.getPosition() + 0.003);
            }
            if(gamepad2.right_bumper){
                Turn.setPosition(Turn.getPosition() - 0.003);
            }
            if(gamepad2.left_trigger > 0.5){
                Claw.setPower(.4); // open
                sleep(50);
            }else{
                Claw.setPower(0);
            }
            if(gamepad2.right_trigger > 0.5){
                Claw.setPower(-.4); // close
                sleep(50);
            }else{
                Claw.setPower(0);
            }

            // pick up blocks on battery side
            // Arm Movement
            if (gamepad2.a) {
                armTargetAngle = 75; // Set target angle to 75 degrees when 'a' is pressed
                setArmPosition(armTargetAngle);
                // Turn.setPosition( find placing on pole value )
            } else if (gamepad2.dpad_up && liftMotor.getCurrentPosition() > -350) {
                armTargetAngle += 0.25;
                setArmPosition(armTargetAngle); // decreases the arm angle by a value of 2 ticks
            } else if (gamepad2.dpad_down && liftMotor.getCurrentPosition() > -350) {
                armTargetAngle -= 0.25;
                setArmPosition(armTargetAngle);
            }// increases the arm angle by a value of 2 ticks
           /* }else if(gamepad2.y){
                armTargetAngle = 13.8;
                setArmPosition(armTargetAngle);
                // Turn.setPosition(find picking up value);
            }else if(gamepad2.dpad_up ){
                armTargetAngle += 0.15;
                setArmPosition(armTargetAngle);
            }else if(gamepad2.dpad_down && armTargetAngle < 180 && liftMotor.getCurrentPosition() > -200){
                armTargetAngle -= 0.15;
                setArmPosition(armTargetAngle);
            }else if(gamepad2.dpad_up && armTargetAngle < 180 && liftMotor.getCurrentPosition() >= -300 ){
                armTargetAngle -= 0.05;
                setArmPosition(armTargetAngle);
            }else if(gamepad2.dpad_down && armTargetAngle < 180 && liftMotor.getCurrentPosition() > -300){
                armTargetAngle -= 0.05;
                setArmPosition(armTargetAngle);
            }

            */

            if(gamepad1.x){
                // find values for under the pole
                // setArmPosition()
                // Turn.setPosition()
            }

            double currentArmAngle = getArmAngle();

            // Lift Movement

            if(currentArmAngle > 67 || currentArmAngle < 67 && liftMotor.getCurrentPosition() > liftMaxPosition ){
                if (gamepad2.x){
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setTargetPosition(liftMotor.getCurrentPosition() - 25); // UP
                    liftMotor.setPower(0.6);
                } else if (gamepad2.b) {
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setTargetPosition(liftMotor.getCurrentPosition() + 25); // DOWN
                    liftMotor.setPower(0.6);
                } else {
                    liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    liftMotor.setPower(0); // Stop the motor
                }
            }else{
                liftMotor.setPower(0);
            }


            /* if(gamepad2.x){
              claw.setPosition()
            }


             */

            telemetry.addData("Arm Angle", currentArmAngle);
            telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
            telemetry.addData("Turn Value", Turn.getPosition());
            telemetry.update();
        }
    }

    public void setArmPosition(double angle) {
        int targetPosition = (int) (angle / ARM_DEGREES_PER_COUNT);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setPower(0.3);
    }

    public double getArmAngle() {
        return armMotor.getCurrentPosition() * ARM_DEGREES_PER_COUNT;
    }

    public void setLiftPosition (double angle) {
        int motorTargetPos = (int) (angle / ARM_DEGREES_PER_COUNT);
        liftMotor.setTargetPosition(motorTargetPos);
        liftMotor.setPower(0.5);
    }
}

/*
   GAMEPAD INPUTS
   --------------
   GAMEPAD1
   a - SPEED ADJUST 0.75
   b - SPEED ADJUST 0.25
   x - NONE
   y - SPEED ADJUST 1

   left stick - FORWARD/BACKWARD/STRAFE
   right stick - TURNING

   right-trig - OPEN CLAW
   left-trig - CLOSE CLAW

   right-bump - NONE
   left-bump - NONE

   DPAD arrows - NONE

   ------------------
   GAMEPAD2

   a - 90 DEGREE ARM VALUE
   b - LIFT DOWN
   x - LIFT UP
   y - 175 DEGREE ARM VALUE

   left-stick - NONE
   right-stick - NONE

   right-trig - NONE
   left-trig - NONE

   right-bump - TURN UP
   left-bump - TURN DOWN

   dpad_up - ARM UP
   dpad_down - ARM DOWN
 */
