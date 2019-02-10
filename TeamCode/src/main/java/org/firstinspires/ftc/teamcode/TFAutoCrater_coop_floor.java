/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "TF Crater - Coop - FloorStart", group = "TFAuto")
//@Disabled
public class TFAutoCrater_coop_floor extends LinearOpMode {
    Hardware4962 robot= new Hardware4962();
//test
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AdHGGHX/////AAABmX2gi6RloEkjpoAvS1hvK7wk4Pm95ewF2MapIFXT2G8yLVz+v9yjsUkgmt7+bYxwCeEnXZKEIxucs7lcJIYV5jUwf+qY0BRr3vLQk9CcsNVjcGDrkDl9KPTouXRI7EZIayntD14AO5FhHxpOszlBEGJAqKPPx7eefDDkjNJQZwlyQdpQ/Hx7uu8V3+ieBVyMEFWNELNDM+hubLfomxtqiaQcqYi0rk3/z9HeO9ycwBrtYasDDpoMUflsZy9e+lq+CF5mkC4T6ICKCUW9y1GsO+fxDFBpYMZMw9cEydcMjbxxB7Oyk3UvAr0PzQscoa2PB3YeBD4gj05VkBmVeVYfj8LU6ZCWOmhCvBMiqu0Fzegl";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // use the Hardware4962 stuff that decribes the robot.
        robot.init(hardwareMap, telemetry);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        // Do not use waitForStart() if you have Motorola E4 phones.
        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        //waitForStart();

        // reset the encoders and get the starting angle
        robot.TurnOffEncoders();
        Orientation angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // start a timer
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            String position = "missing";

            while (opModeIsActive() && runtime.milliseconds() < 2000) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions) {
                            double ang = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                            telemetry.addData("obj:",recognition.getLabel() + ang);
                            if (recognition.getLabel()=="Gold Mineral"){
                                if (ang>15){
                                    position="right";
                                }else if (ang<-15){
                                    position="left";
                                }else {
                                    position="center";
                                }
                            }
                        }

                      telemetry.update();
                    }
                }
            }
// lower from lander
/*
            robot.ElevatorUp(false);
            while (robot.topLimit.getState() && opModeIsActive()){
                robot.ElevatorUp(false);
                angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.clear();
                telemetry.addData("heading: ", angles.firstAngle);
                telemetry.addData("switch: ", robot.topLimit.getState());
                telemetry.addData("Gold Mineral Position", position);
                telemetry.update();
            }
            robot.elevatorMotor.setPower(0);
*/
            // get starting angle

            angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float startAngle = angles.firstAngle;
            DriveOnHeading(startAngle, 10, .7, .06);
            robot.StopDriving();

            if (position=="left") {
                RotateToHeading(startAngle+55,.05);
                DriveOnHeading(startAngle+55, 20, .7, .06);
                robot.StopDriving();
                DriveOnHeadingBackwards(startAngle+55, 20, -.7, .06);
                robot.StopDriving();
            } else if (position=="right"){
                RotateToHeading(startAngle-55,.05);
                DriveOnHeading(startAngle-55, 20, .7, .06);
                robot.StopDriving();
                DriveOnHeadingBackwards(startAngle-55, 20, -.7, .06);
                robot.StopDriving();

            } else {

                DriveOnHeading(startAngle, 16, .7, .06);
                robot.StopDriving();
                DriveOnHeadingBackwards(startAngle, 16, -.7, .06);
                robot.StopDriving();
            }
            robot.StopDriving();
            RotateToHeading(startAngle+75,.05);
            DriveOnHeading(startAngle+75, 43, .7, .06);
            robot.StopDriving();
            //towards depot
            RotateToHeading(startAngle+130,.05);
            DriveOnHeading(startAngle+130, 44, .7, .06);
            robot.marker.setPosition(0.7);
            robot.StopDriving();
            sleep(1000);
            //DriveOnHeadingBackwards(startAngle+135, 4, -.2, .06);
            DriveOnHeadingBackwards(startAngle+133, 70, -.99, .06);
            robot.StopDriving();
            robot.marker.setPosition(0.2);

        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void DriveOnHeading( double heading, double distanceInches, double power, double gain) {

        // calculate encoder counts for distance
        float wheelDiameter = 4; // inches
        float wheelCirc = wheelDiameter * (float) 3.14159;
        float encoderTicksPerRotation = 1120;  // Neverest 40
        //float encoderTicksPerRotation = 560; // Neverest 20  FIXME
        float ticksPerInch = encoderTicksPerRotation / wheelCirc;
        int ticksToTravel = (int) (distanceInches * ticksPerInch);
        float MAX_ERROR = 1; // max number of degrees we can be off

        // check motor position
        int startEncCount=robot.leftfrontMotor.getCurrentPosition();

        double left = 0;
        double right = 0;
        while ((robot.leftfrontMotor.getCurrentPosition()-startEncCount) < ticksToTravel && opModeIsActive()) {
            Orientation angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double error = angles.firstAngle - heading;
            if (error > 180) { error = error - 360; }
            if (error < -180) { error = error + 360; }
            if (Math.abs(error) < MAX_ERROR) {
                robot.Drive(power,power);
            }  else {
                double correction = gain * error;
                //if (Math.abs(correction) > power) {
                //   correction = power;
                // }
                left = power + correction;
                right = power - correction;
                right = Range.clip(right, -1, 1);
                left = Range.clip(left, -1, 1);

                robot.Drive(left, right);
            }
            telemetry.clear();
            telemetry.addData("heading: ", angles.firstAngle);
            telemetry.addData("target:", heading);
            telemetry.addData("enc left: ", "%s", robot.leftfrontMotor.getCurrentPosition());
            telemetry.addData("target: ", "%s", -ticksToTravel);
            telemetry.addData("power L: ", left);
            telemetry.addData("power R: ", right);
            telemetry.update();
        }



    }


    public void DriveOnHeadingBackwards( double heading, double distanceInches, double power, double gain) {

        // calculate encoder counts for distance
        float wheelDiameter = 4; // inches
        float wheelCirc = wheelDiameter * (float) 3.14159;
        float encoderTicksPerRotation = 1120;  // Neverest 40
        //float encoderTicksPerRotation = 560; // Neverest 20  FIXME
        float ticksPerInch = encoderTicksPerRotation / wheelCirc;
        int ticksToTravel = (int) (distanceInches * ticksPerInch);
        float MAX_ERROR = 1; // max number of degrees we can be off

        // check motor position
        int startEncCount=robot.leftfrontMotor.getCurrentPosition();

        double left = 0;
        double right = 0;
        while ((robot.leftfrontMotor.getCurrentPosition()-startEncCount) > -ticksToTravel && opModeIsActive()) {
            Orientation angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double error = angles.firstAngle - heading;
            if (error > 180) { error = error - 360; }
            if (error < -180) { error = error + 360; }
            if (Math.abs(error) < MAX_ERROR) {
                robot.Drive(power,power);
            }  else {
                double correction = gain * error;
                //if (Math.abs(correction) > power) {
                //    correction = power;
                //}
                left = power + correction;
                right = power - correction;
                right = Range.clip(right, -1, 1);
                left = Range.clip(left, -1, 1);

                robot.Drive(left, right);
            }
            telemetry.clear();
            telemetry.addData("heading: ", angles.firstAngle);
            telemetry.addData("enc left: ", "%s", robot.leftfrontMotor.getCurrentPosition());
            telemetry.addData("target: ", "%s", ticksToTravel);
            telemetry.addData("power L: ", left);
            telemetry.addData("power R: ", right);
            telemetry.update();
        }



    }

    public void RotateToHeading( double heading, double gain){
        float MAX_ERROR = 1; // max number of degrees we can be off
        ElapsedTime runtime = new ElapsedTime();

        Orientation angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = angles.firstAngle - heading;
        if (error > 180) { error = error - 360; }
        if (error < -180) { error = error + 360; }
        runtime.reset();
        while (Math.abs(error) > MAX_ERROR && (runtime.seconds() < 3.0) && opModeIsActive()){
            angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = angles.firstAngle - heading;
            if (error > 180) { error = error - 360; }
            if (error < -180) { error = error + 360; }
            double power = error * gain;
            if (power > .4) { power = .4;}
            if (power > 0 && power < .15) { power = .15; }
            if (power < 0 && power > -.15) { power = -.15; }
            if (power < -.4) { power = -.4; }
            robot.Drive(power, -power);
            telemetry.clear();
            telemetry.addData("heading: ", angles.firstAngle);
            telemetry.addData("enc left: ", "%s", robot.leftfrontMotor.getCurrentPosition());
            telemetry.addData("power : ", power);
            telemetry.update();

        }
        robot.StopDriving();
    }


}
