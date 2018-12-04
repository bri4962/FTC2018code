package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//Vuforia stuff
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by maddie.smith on 11/5/17.
 */

@Autonomous(name="Auto_BlueSideKeyOnbot", group = "Auto")
public class Auto_BlueSideKeyOnbot extends LinearOpMode {
    Hardware4962 robot= new Hardware4962();
    
    public static final String TAG = "Vuforia VuMark Sample";
    VuforiaLocalizer vuforia;
    String whichColumn = "unknown";
    public void runOpMode() throws InterruptedException {

        // use the Hardware4962 stuff that decribes the robot.
        robot.init(hardwareMap, telemetry);
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AdGXL47/////AAAAGRMvzAzbRE9bnWB0RA6jU86JA0wqGKzdhRz+RiDSsQecZCB1doLVG4QgrEmVTBNvyrMFy+6Ba8kdJdZuXs3P+QHACNxK/Ig7SOfJ5zDJ8Y1DrPq5XqSWnjOwETMUeg6s92+tDRsOEbLBHphmaoXEQ7hJ0riwuamYOBtHiPzhK4p9qEbE7SDV+guXN0vAq1z4kmIRpAc3fBxkLGAziKH4nWBDvHfr2qMRIPvGJb6EAVuT7fQlFasFEIf9zgjzGn7rddD4ZUCEQLjwjijIPGYXywNDA2GE5XYjr2VPiJbm8ihYX6lWA6cUMxwQPkRhpmm6CeECIgfMZximnqwKgmQ+XXQpZyeoMFOwo4UXWcibIoJI";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        waitForStart();
        
        relicTrackables.activate();
        
        robot.tiltCage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.tiltCage.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive()){
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            boolean breakLoop = false;

            while (!breakLoop && runtime.seconds() < 5.0) {
              /**
               * See if any of the instances of {@link relicTemplate} are currently visible.
               * {@link RelicRecoveryVuMark} is an enum which can have the following values:
               * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
               * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
               */
              RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
              if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                  /* Found an instance of the template. In the actual game, you will probably
                   * loop until this condition occurs, then move on to act accordingly depending
                   * on which VuMark was visible. */
                  telemetry.addData("VuMark", "%s visible", vuMark);
                  breakLoop = true;
                  if (vuMark == RelicRecoveryVuMark.CENTER) {
                      whichColumn = "center";
                  } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                      whichColumn = "left";
                  } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                      whichColumn = "right";
                  }
              } 
              else {

                telemetry.addData("VuMark", "not visible");
              }
            }

            telemetry.update();

            robot.armDown();
            sleep(2000);

            float readRed = robot.sensorColor.red();
            float readBlue = robot.sensorColor.blue();
            telemetry.addData("Red  ", readRed);
            telemetry.addData("Blue ", readBlue);
            telemetry.addData("VuMark ", whichColumn);
            telemetry.update();

            if (readRed > readBlue ) {
               robot.armBackward();
                sleep(2000);
                robot.armMiddle();
                sleep(2000);
                robot.armUp();
                sleep(1500);


            } else {
                robot.armForward();
                sleep(2000);
                robot.armMiddle();
                sleep(2000);
                robot.armUp();
                sleep(1500);
            }
            
            if (whichColumn == "left" || whichColumn == "unknown") {
                robot.DriveOnHeading (0,23,.2,.03);
            }
            
            if (whichColumn == "center") {
                robot.DriveOnHeading (0,31,.2,.03);
            }
            
            if (whichColumn == "right") {
                robot.DriveOnHeading (0,39,.2,.03);
            }
            
            robot.RotateToHeading(-90,.02);
            robot.DriveOnHeadingBackwards(-90,6,-.2,.03);
            robot.StopDriving();
            robot.tiltCage.setTargetPosition(170);
            robot.tiltCage.setPower(1);
            sleep (5000);
            robot.DriveOnHeading (-90,10,.2,.03);
            robot.StopDriving();
            robot.tiltCage.setTargetPosition(0);
            robot.tiltCage.setPower(.4);
            robot.leftintake.setPower(1);
            robot.rightintake.setPower(-1);
            robot.DriveOnHeading (-90,26,.2,.03);
            robot.StopDriving();
            sleep (2000);
            robot.leftintake.setPower(0);
            robot.rightintake.setPower(-0);
            
            break;


        }

    }




}

