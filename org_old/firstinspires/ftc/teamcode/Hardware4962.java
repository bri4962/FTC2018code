/**
 * Hardware initialization for 4962 robot
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 *
 */

/* adb setup.  Connect phone to USB. Connect computer wifi to Wifi Direct. In terminal type:
1) adb usb
2) adb tcpip 5555
3) adb connect 192.168.49.1
4) (unplug phone)
5) adb connect 192.168.49.1
 */
public class Hardware4962 {

    Telemetry telemetry;

    /* Public OpMode members. */
    public DcMotor rightfrontMotor = null;
    public DcMotor leftfrontMotor = null;
    public DcMotor rightbackMotor = null;
    public DcMotor leftbackMotor = null;
    public DcMotor elevatorMotor = null;
    public DcMotor tiltCage = null;
   // public DcMotor beltMotor = null;
   // public DcMotor relicMotor1 = null;
    //public DcMotor relicMotor2 = null;
    public DcMotor leftintake = null;
    public DcMotor rightintake = null;
    //public CRServo leftintake = null;
    //public CRServo rightintake = null;
    //public Servo intakeArmRight= null;
    //public Servo intakeArmLeft = null;
    public Servo lineupServo = null;
    //public CRServo tiltCage = null;
    public Servo clawOpen = null;
    public Servo clawForward = null;
    public Servo clawSide = null;
    public Servo colorArm = null;
    public Servo clampservo = null;
    public Servo clampservo2 = null;
    public Servo colorArmTilt = null;

    public ColorSensor sensorColor = null;
    public DistanceSensor sensorDistance = null;
    //public NavxMicroNavigationSensor navxMicro = null;
    //public IntegratingGyroscope gyro = null;
    // The IMU sensor object
    public BNO055IMU gyro = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware4962() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // we want to print teleetry in the Hardware class

        this.telemetry = telemetry;

        // Define and Initialize Motors

        rightfrontMotor = hwMap.dcMotor.get("motor right front");
        leftfrontMotor = hwMap.dcMotor.get("motor left front");
        rightbackMotor = hwMap.dcMotor.get("motor right back");
        leftbackMotor = hwMap.dcMotor.get("motor left back");
        elevatorMotor = hwMap.dcMotor.get("motor elevator");
        tiltCage = hwMap.dcMotor.get("tilt");
        //beltMotor = hwMap.dcMotor.get("motor belt");
        //relicMotor1 = hwMap.dcMotor.get("motor relic1");
        //relicMotor2 = hwMap.dcMotor.get("motor relic2");
        leftintake = hwMap.dcMotor.get("intake left");
        rightintake = hwMap.dcMotor.get("intake right");

        // define servos

        clawOpen = hwMap.servo.get("claw open");
        clawForward = hwMap.servo.get("claw forward");
        clawSide = hwMap.servo.get("claw side");
        colorArm = hwMap.servo.get("color arm");
        colorArmTilt = hwMap.servo.get("color arm tilt");
        lineupServo = hwMap.servo.get("line up");
        clampservo = hwMap.servo.get("clampservo");
        clampservo2 = hwMap.servo.get("clampservo2");

        // get a reference to the color sensor.
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");

        // NavX
        //navxMicro = hwMap.get(NavxMicroNavigationSensor.class, "navx");
        //gyro = (IntegratingGyroscope)navxMicro;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro = hwMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);

        leftfrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftbackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightfrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftfrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftbackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftintake.setPower(0);
        rightintake.setPower(0);
        tiltCage.setPower(0);
        clawOpen.setPosition(0.31);
        clawForward.setPosition(0);
        clawSide.setPosition(0.6);
        lineupServo.setPosition(1);
        clampservo.setPosition(1);
        clampservo2.setPosition(0);
        //intakeArmLeft.setPosition(0.8); // outward is decreasing
        //intakeArmRight.setPosition(.1); // outward is increasing
        colorArm.setPosition(0.5);
        colorArmTilt.setPosition(.1);

        // Set all motors to zero power

        StopDriving();



    }
    public void ResetEncoders() {
        leftfrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftbackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void TurnOffEncoders() {
        rightfrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftfrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftbackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void TurnOnEncoders() {
        rightfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double leftDistance() {
        // calculate encoder counts for distance
        float wheelDiameter = 4; // inches
        float wheelCirc = wheelDiameter * (float) 3.14159;
        //float encoderTicksPerRotation = 1120;  // Neverest 40
        float encoderTicksPerRotation = 560; // Neverest 20  FIXME
        float ticksPerInch = encoderTicksPerRotation / wheelCirc;
        return (leftfrontMotor.getCurrentPosition() / ticksPerInch);
    }


    public void StopDriving() {
        //telemetry.addData("Stopping, Enc:", motorLeft1.getCurrentPosition());
        Drive(0, 0);
    }

    // color arm control
    public void armMiddle() {
        colorArm.setPosition(.5);
    }

    public void armForward() {
        colorArm.setPosition(.8);
    }

    public void armBackward() {
        colorArm.setPosition(.2);
    }

    public void armDown() {
        colorArmTilt.setPosition(.8);
    }

    public void armUp() {
        colorArmTilt.setPosition(.3);
    }

    public void Drive(double left, double right) {
        leftfrontMotor.setPower(left);
        leftbackMotor.setPower(left);
        rightfrontMotor.setPower(right);
        rightbackMotor.setPower(right);
    }
    public void FloatMotors() {
        leftfrontMotor.setPowerFloat();
        leftbackMotor.setPowerFloat();
        rightfrontMotor.setPowerFloat();
        rightbackMotor.setPowerFloat();
    }
    public void BrakeMotors() {
        leftfrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightbackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void DriveOnHeading( double heading, double distanceInches, double power, double gain) {

        // calculate encoder counts for distance
        float wheelDiameter = 4; // inches
        float wheelCirc = wheelDiameter * (float) 3.14159;
        //float encoderTicksPerRotation = 1120;  // Neverest 40
        float encoderTicksPerRotation = 560; // Neverest 20  FIXME
        float ticksPerInch = encoderTicksPerRotation / wheelCirc;
        int ticksToTravel = (int) (distanceInches * ticksPerInch);
        float MAX_ERROR = 1; // max number of degrees we can be off

        // check motor position
        int startEncCount=leftfrontMotor.getCurrentPosition();

        double left = 0;
        double right = 0;
        while ((leftfrontMotor.getCurrentPosition()-startEncCount) > -ticksToTravel) {
            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double error = angles.firstAngle - heading;
            if (error > 180) { error = error - 360; }
            if (error < -180) { error = error + 360; }
            if (Math.abs(error) < MAX_ERROR) {
                Drive(power,power);
            }  else {
                double correction = gain * error;
                //if (Math.abs(correction) > power) {
                //   correction = power;
                // }
                left = power + correction;
                right = power - correction;
                right = Range.clip(right, -1, 1);
                left = Range.clip(left, -1, 1);

                Drive(left, right);
            }
            telemetry.clear();
            telemetry.addData("heading: ", angles.firstAngle);
            telemetry.addData("enc left: ", "%s", leftfrontMotor.getCurrentPosition());
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
        //float encoderTicksPerRotation = 1120;  // Neverest 40
        float encoderTicksPerRotation = 560; // Neverest 20  FIXME
        float ticksPerInch = encoderTicksPerRotation / wheelCirc;
        int ticksToTravel = (int) (distanceInches * ticksPerInch);
        float MAX_ERROR = 1; // max number of degrees we can be off

        // check motor position
        int startEncCount=leftfrontMotor.getCurrentPosition();

        double left = 0;
        double right = 0;
        while ((leftfrontMotor.getCurrentPosition()-startEncCount) < ticksToTravel) {
            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double error = angles.firstAngle - heading;
            if (error > 180) { error = error - 360; }
            if (error < -180) { error = error + 360; }
            if (Math.abs(error) < MAX_ERROR) {
                Drive(power,power);
            }  else {
                double correction = gain * error;
                //if (Math.abs(correction) > power) {
                //    correction = power;
                //}
                left = power + correction;
                right = power - correction;
                right = Range.clip(right, -1, 1);
                left = Range.clip(left, -1, 1);

                Drive(left, right);
            }
            telemetry.clear();
            telemetry.addData("heading: ", angles.firstAngle);
            telemetry.addData("enc left: ", "%s", leftfrontMotor.getCurrentPosition());
            telemetry.addData("target: ", "%s", ticksToTravel);
            telemetry.addData("power L: ", left);
            telemetry.addData("power R: ", right);
            telemetry.update();
        }



    }

    public void RotateToHeading( double heading, double gain){
        float MAX_ERROR = 1; // max number of degrees we can be off
        ElapsedTime runtime = new ElapsedTime();

        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = angles.firstAngle - heading;
        if (error > 180) { error = error - 360; }
        if (error < -180) { error = error + 360; }
        runtime.reset();
        while (Math.abs(error) > MAX_ERROR && (runtime.seconds() < 3.0)){
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = angles.firstAngle - heading;
            double power = error * gain;
            if (power > .4) { power = .4;}
            if (power > 0 && power < .15) { power = .15; }
            if (power < 0 && power > -.15) { power = -.15; }
            if (power < -.4) { power = -.4; }
            Drive(power, -power);
            telemetry.clear();
            telemetry.addData("heading: ", angles.firstAngle);
            telemetry.addData("enc left: ", "%s", leftfrontMotor.getCurrentPosition());
            telemetry.addData("power : ", power);
            telemetry.update();

        }
        StopDriving();
    }


}
