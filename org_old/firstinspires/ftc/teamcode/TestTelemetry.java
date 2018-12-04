package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestTelemetry {
    Telemetry telemetry;

    public void init(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public void printTelemetry() {

      //telemetry.clear();
      telemetry.addData("heading: ", 50);
      telemetry.addData("this is a test ","testing");
      telemetry.update();
    }
    
}
