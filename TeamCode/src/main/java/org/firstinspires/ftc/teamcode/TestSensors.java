package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by FTC 4962 on 11/22/17.
 */
@TeleOp(name="Test Sensors", group = "Testing")
public class TestSensors extends LinearOpMode {
    Hardware4962 robot= new Hardware4962();
    Orientation angles;

    @Override public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);

        waitForStart();
        robot.init(hardwareMap, telemetry);
        robot.ResetEncoders();
        while (opModeIsActive()) {
            String bob = "false";
            if (robot.topLimit.getState()) {  bob = "true"; }
            angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading: ",angles.firstAngle );
            telemetry.addData("enc leftF: ", "%s", robot.leftfrontMotor.getCurrentPosition());
            telemetry.addData("left dist: ", robot.leftDistance() );
            telemetry.addData("enc rightF: ", "%s", robot.rightfrontMotor.getCurrentPosition());
            telemetry.addData("enc elev: ", "%s", robot.elevatorMotor.getCurrentPosition());
            telemetry.addData("top limit:","%s", bob);

            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

        }

    }

}
