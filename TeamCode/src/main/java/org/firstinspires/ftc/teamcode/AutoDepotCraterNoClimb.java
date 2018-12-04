package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//Vuforia stuff

/**
 * Created by bria.smith on 10/19/1=8.
 */

@Autonomous(name="AutoDepotCraterNoClimb", group = "Auto")
public class AutoDepotCraterNoClimb extends LinearOpMode {
    Hardware4962 robot= new Hardware4962();


    public void runOpMode() throws InterruptedException {

        // use the Hardware4962 stuff that decribes the robot.
        robot.init(hardwareMap, telemetry);

        waitForStart();



            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            boolean breakLoop = false;

            // 1) Lower and unlatch from lander
            // 2) drive forward towatds the depot
            // 3) drop the team marker
            // 4) rotate towards the crater (which one?) - maybe square up on the wall
            // 5) drive forward to park on the edge of the crater
            // stop


            robot.TurnOffEncoders();
            Orientation angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            // 1) Lower and unlatch from lander
            // 2) drive forward towatds the crater and park
            //
            // stop
/*
            robot.ElevatorUp(false);
            while (robot.topLimit.getState() && opModeIsActive()){
                robot.ElevatorUp(false);
                angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.clear();
                telemetry.addData("heading: ", angles.firstAngle);
                telemetry.addData("switch: ", robot.topLimit.getState());
                telemetry.update();
            }
            */

            angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float startAngle = angles.firstAngle;

            DriveOnHeading(startAngle,67,.7,.06);
            robot.StopDriving();
            robot.marker.setPosition(0.7);
            sleep(2000);

            DriveOnHeadingBackwards(startAngle+55, 98,-.7,.06);
        robot.StopDriving();

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
            telemetry.addData("target heading:", heading);
            telemetry.addData("enc left: ", "%s", robot.leftfrontMotor.getCurrentPosition());
            telemetry.addData("target ticks: ", "%s", ticksToTravel);
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

