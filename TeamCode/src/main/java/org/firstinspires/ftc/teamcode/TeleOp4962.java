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
@TeleOp(name = "TeleOp 4962", group = "TeleOp")
//@Disabled
public class TeleOp4962 extends LinearOpMode {

    Hardware4962 robot= new Hardware4962();        // this is our hardware class

    @Override
    public void runOpMode() throws InterruptedException {

        // use the Hardware4962 stuff that decribes the robot.
        robot.init(hardwareMap, telemetry);


        // wait for the start button to be pressed.
        waitForStart();

        int encoderCountFront = 0;
        int encoderCountBack = 0;

        // intial servo positions


        //robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        boolean resetingElevator = false; //true is a reset is in progress

        while (opModeIsActive()) {
         /*
         * Gamepad 1 controls the motors via the left and right stick
         */

            float left = gamepad1.left_stick_y;
            float right = gamepad1.right_stick_y;
            double turbo = gamepad1.left_trigger;
            //boolean relicOut = gamepad1;


            // Gamepad 2 controls the mechanisms

            float shoulder = -gamepad2.left_stick_y;
            float hook = gamepad2.right_stick_y;

            // clip the right/left values so that the values never exceed +/- 1

            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);


            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.
            right = -(float) scaleInput(right);
            left = -(float) scaleInput(left);
            //if (turbo < 0.5) {  // half speed unless turbo is pushed

            //    right = right * (float) 0.5;
            //    left = left * (float) 0.5;
            //}

            robot.elevatorMotor.setPower(shoulder);
            //robot.hook.setPower(hook);
            // clip the right/left values so that the values never exceed +/- 1
           // Rintake = Range.clip(Rintake, -1, 1);
            //Lintake = Range.clip(Lintake, -1, 1);


            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.
           // Rintake = -(float) scaleInput(Rintake);
            //Lintake = -(float) scaleInput(Lintake);




            // write the values to the motors
            robot.Drive(left,right);



            telemetry.addData("Text", "*** Robot Data***");
            telemetry.addData("Lift power","Lift:" + String.format("%.2f",(float) shoulder));
            //telemetry.addData("Right Servo","Right servo pos:" + String.format("%.2f", (float) rightArmPosition));
            //telemetry.addData("Left Servo","Leftt servo pos:" + String.format("%.2f", (float) leftArmPosition));
            telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
            telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
            telemetry.addData("enc elev: ", "%s", robot.elevatorMotor.getCurrentPosition());
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