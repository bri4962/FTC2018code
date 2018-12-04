/**
 *
 * TeleOp using 4962 hardware
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 *
 */
@TeleOp(name = "TeleOp 4962onbot", group = "TeleOp")
//@Disabled
public class Teleop4962aOnbot extends LinearOpMode {

    Hardware4962 robot= new Hardware4962();        // this is our hardware class

    @Override
    public void runOpMode() throws InterruptedException {

        // use the Hardware4962 stuff that decribes the robot.
        robot.init(hardwareMap, telemetry);
        robot.armUp();
        robot.armMiddle();

        double intakePower = 0.0;

        // wait for the start button to be pressed.
        waitForStart();

        int encoderCountFront = 0;
        int encoderCountBack = 0;

        // intial servo positions

        double rightArmPosition = 0;
        double leftArmPosition = 0.85;
        robot.armUp();
        robot.armMiddle();

        robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.tiltCage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.tiltCage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.tiltCage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        while (opModeIsActive()) {
         /*
         * Gamepad 1 controls the motors via the left and right stick
         */

            float left = gamepad1.left_stick_y;
            float right = gamepad1.right_stick_y;
            float Rintake = gamepad2.right_stick_y;
            float Lintake = gamepad2.left_stick_y;
            boolean elevatorUp = gamepad2.right_bumper;
            boolean elevatorDown = gamepad2.left_bumper;
            boolean tiltforward = gamepad2.b;
            boolean tiltbackward = gamepad2.x;
            boolean tiltcenter = gamepad2.y;
            boolean elevatortop = gamepad2.a;
            boolean lineupServoDown = gamepad1.a;
            boolean lineupServoUp = gamepad1.b;
            boolean lockDown = gamepad1.x;
            boolean lockUp = gamepad1.y;

            // boolean shootme = gamepad2.right_bumper;
            //double shootoff = gamepad2.right_trigger;
            //boolean launchme = gamepad2.left_bumper;
            //double nudgeme = gamepad2.left_trigger;
            double turbo = gamepad1.left_trigger;
            //boolean shootlong = gamepad2.a;
            //boolean sidebutton = gamepad1.x;


            // clip the right/left values so that the values never exceed +/- 1

            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);


            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.
            right = -(float) scaleInput(right);
            left = -(float) scaleInput(left);
            if (turbo < 0.5) {  // half speed unless turbo is pushed

                right = right * (float) 0.5;
                left = left * (float) 0.5;
            }

            // clip the right/left values so that the values never exceed +/- 1
            Rintake = Range.clip(Rintake, -1, 1);
            Lintake = Range.clip(Lintake, -1, 1);


            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.
            Rintake = -(float) scaleInput(Rintake);
            Lintake = -(float) scaleInput(Lintake);


            /*if (intakeIn) {
                robot.rightintake.setPower(.7);
                robot.leftintake.setPower(-.7);
            }
            if (intakeOut) {
                robot.rightintake.setPower(-.7);
                robot.leftintake.setPower(.7);
            }
            if (!intakeIn && !intakeOut) {
                robot.rightintake.setPower(0);
                robot.leftintake.setPower(0);
            }
            if (rintakein) {
                rightArmPosition = rightArmPosition + 0.005;
                if (rightArmPosition > 1) { rightArmPosition = 1; }
                robot.intakeArmRight.setPosition(rightArmPosition);
                leftArmPosition = leftArmPosition - 0.005;
                if (leftArmPosition < 0) { leftArmPosition = 0; }
                robot.intakeArmLeft.setPosition(leftArmPosition);

            }
            if (rintakeout) {
                rightArmPosition = rightArmPosition - 0.005;
                if (rightArmPosition < 0) { rightArmPosition = 0; }
                robot.intakeArmRight.setPosition(rightArmPosition);
                leftArmPosition = leftArmPosition + 0.005;
                if (leftArmPosition > 1) { leftArmPosition = 1; }
                robot.intakeArmLeft.setPosition(leftArmPosition);
            }*/
            /*
            if (lintakein) {
                leftArmPosition = leftArmPosition - 0.005;
                if (leftArmPosition < 0) { leftArmPosition = 0; }
                robot.intakeArmLeft.setPosition(leftArmPosition);
            }
            if (lintakeout > 0.5) {
                leftArmPosition = leftArmPosition + 0.005;
                if (leftArmPosition > 1) { leftArmPosition = 1; }
                robot.intakeArmLeft.setPosition(leftArmPosition);
            }*/
            //if (intakearmsstraight) {
               // robot.ArmsStraight();
           // }
            /*if (intakearmsin) {
                robot.intakeArmLeft.setPosition(0.2);
                robot.intakeArmRight.setPosition(0.65);
           }
            if (intakearmsin) {
                robot.intakeArmLeft.setPosition(0.2);
                robot.intakeArmRight.setPosition(0.65);
            }*/
            /* if (beltforward) {
                robot.beltMotor.setPower(1);
            }
            if (beltbackward) {
                robot.beltMotor.setPower(-1);
            }
            if (!beltforward && !beltbackward) {
                robot.beltMotor.setPower(0);
            } */
            if (tiltforward) {
                robot.tiltCage.setTargetPosition(0);
                robot.tiltCage.setPower(.4);
                //robot.tiltCage.setPower(.5);
            }
            if (tiltbackward){
                //robot.tiltCage.setPower(-.8);
                robot.tiltCage.setTargetPosition(170);
                robot.tiltCage.setPower(1);
            }
            if (tiltcenter) {
                robot.tiltCage.setTargetPosition(60);
                robot.tiltCage.setPower(1);
                //robot.tiltCage.setPower(0);
            }
            if (elevatorUp){
                robot.elevatorMotor.setPower(1);
                robot.elevatorMotor.setTargetPosition(280);
            }
            if (elevatorDown){
                robot.elevatorMotor.setTargetPosition(0);
                robot.elevatorMotor.setPower(1);
            }
            if (!elevatorUp && !elevatorDown) {
                robot.elevatorMotor.setPower(0);
            }
            if (lineupServoDown) {
                robot.lineupServo.setPosition(.5);
            }
            if (lineupServoUp) {
                robot.lineupServo.setPosition(1);
            }
            if (lockDown) {
                robot.clampservo.setPosition(.5);
                robot.clampservo2.setPosition(.5);
            }
            if (lockUp) {
                robot.clampservo.setPosition(1);
                robot.clampservo2.setPosition(0);
            }
            if (elevatortop) {
                robot.elevatorMotor.setTargetPosition(-300);
                if (robot.elevatorMotor.getCurrentPosition() > -310 &&
                        robot.elevatorMotor.getCurrentPosition() < -290) {
                    robot.elevatorMotor.setPower(0);
                } else {
                    robot.elevatorMotor.setPower(-.5);
                }
            }

            // write the values to the motors
            robot.Drive(left,right);

            robot.leftintake.setPower(Lintake);
            robot.rightintake.setPower(-Rintake);

            //robot.intakeMotor.setPower(intakePower);
            //robot.elevatorMotor.setPower(elevatorPower);

            telemetry.addData("Text", "*** Robot Data***");
            //telemetry.addData("Right Servo","Right servo pos:" + String.format("%.2f", (float) rightArmPosition));
            //telemetry.addData("Left Servo","Leftt servo pos:" + String.format("%.2f", (float) leftArmPosition));
            telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
            telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
            telemetry.addData("enc elev: ", "%s", robot.elevatorMotor.getCurrentPosition());
            telemetry.addData("tilt:", "%s", robot.tiltCage.getCurrentPosition());
            //telemetry.addData("elevator", "e pwr = " + String.format("%.2f", elevatorPower));
            telemetry.update();
            idle();
        }
    }



    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
