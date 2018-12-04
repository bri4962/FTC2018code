package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous

public class TestPrintAuto extends LinearOpMode {
    TestTelemetry robot;
    

   public void runOpMode() throws InterruptedException {
        robot = new TestTelemetry();
        robot.init(telemetry);
        
        waitForStart();
        //runtime.reset();


        while(opModeIsActive()){
            robot.printTelemetry();
            sleep(250);
        }
   }
}
   
