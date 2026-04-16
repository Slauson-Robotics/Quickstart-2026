package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class exampleTelemetry extends LinearOpMode {
    int q=0;
    int d=0;
    int n=0;
    int f=0;
    int mode=0;
    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.update();
            switch (mode) {
                case 0:
                    telemetry.addData("How many quarters", q);
                    if (gamepad1.dpad_up) {
                        q+=1;
                    } else if (gamepad1.a){
                        mode=1;
                    }
                case 1:

            }
        }
    }


}
