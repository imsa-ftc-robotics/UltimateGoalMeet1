package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RingContourTestV2 extends RobotV2 {

    public void op_mode(){
        String height;
        while (!isStarted()){
            height = "[HEIGHT]" + " " + pipeline.getHeight();
            telemetry.addData("[Ring Stack] >>", height);
            telemetry.update();
            idle();
        }
        waitForStart();
        height = "[HEIGHT]" + " " + pipeline.getHeight();
        telemetry.addData("[Ring Stack] >>", height);
        telemetry.update();
        sleep(50000);

    }
}
