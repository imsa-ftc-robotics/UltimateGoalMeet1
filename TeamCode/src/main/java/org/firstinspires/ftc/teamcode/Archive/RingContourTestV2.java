package org.firstinspires.ftc.teamcode.Archive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
//@Disabled
public class RingContourTestV2 extends RobotV2 {

    public void op_mode(){
        String height;
        FtcDashboard.getInstance().startCameraStream(camera, 30);


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
