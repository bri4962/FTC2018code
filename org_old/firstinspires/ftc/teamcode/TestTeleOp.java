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
@TeleOp(name = "TestTeleOp", group = "TeleOp")
//@Disabled
public class TestTeleOp extends LinearOpMode {

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
            boolean lockDown = gamepad1.x;
            boolean lockUp = gamepad1.y;
            double turbo = gamepad1.left_trigger;

            // Gamepad 2 controls the mechanisms


            float Rintake = gamepad2.right_stick_y;
            float Lintake = gamepad2.left_stick_y;
            boolean elevatorUp = gamepad2.right_bumper;
            boolean elevatorDown = gamepad2.left_bumper;
            boolean tiltforward = gamepad2.b;
            boolean tiltbackward = gamepad2.x;
            boolean tiltcenter = gamepad2.y;
            boolean resetTilt = gamepad2.a;


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


            if (tiltforward) {
                robot.tiltCage.setTargetPosition(0);
                robot.tiltCage.setPower(.4);
                //robot.tiltCage.setPower(.5);
            }
            if (tiltbackward){
                //robot.tiltCage.setPower(-.8);
                robot.tiltCage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.tiltCage.setTargetPosition(210);
                robot.tiltCage.setPower(1);
            }
            if (tiltcenter) {
                robot.tiltCage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.tiltCage.setTargetPosition(60);
                robot.tiltCage.setPower(1);
                //robot.tiltCage.setPower(0);
            }
            if (resetTilt) {
                robot.tiltCage.setPower(-0.4);
                robot.tiltCage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                robot.tiltCage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

            if (lockDown) {
                robot.clampservo.setPosition(.5);
                robot.clampservo2.setPosition(.5);
            }
            if (lockUp) {
                robot.clampservo.setPosition(1);
                robot.clampservo2.setPosition(0);
            }


            // write the values to the motors
            robot.Drive(left,right);

            robot.leftintake.setPower(Lintake);
            robot.rightintake.setPower(-Rintake);


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
