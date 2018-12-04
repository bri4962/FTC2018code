

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import android.speech.tts.TextToSpeech;
import android.util.Log;

@Autonomous(name="Test Speech2", group="Linear Opmode")

public class TextToSpeechTest extends LinearOpMode implements TextToSpeech.OnInitListener {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    TextToSpeech ttobj=new TextToSpeech(AppUtil.getInstance().getActivity(), new TextToSpeech.OnInitListener() {
       @Override
       public void onInit(int status) {
       }
    });

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //text=new TextToSpeech(AppUtil.getInstance().getActivity(), new TextToSpeech.OnInitListener());

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        ttobj.setSpeechRate((float)0.8);
        


        ttobj.speak("Hello Riley, Bria, Chrissa, and Maddie!  My name is Falcon." +
        "I am here.  This is fun.  Robots are cool." + "I will now give a presentation"
        + "about my capabilities.", TextToSpeech.QUEUE_FLUSH, null);
            
        while (opModeIsActive() && ttobj.isSpeaking()) {
                idle();
        }
            

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        
    }
    @Override
    public void onInit(int status) {
        
    }
}
