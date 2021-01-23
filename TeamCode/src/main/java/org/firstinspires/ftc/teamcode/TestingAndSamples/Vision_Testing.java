package org.firstinspires.ftc.teamcode.TestingAndSamples;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Archive.Robot;

@TeleOp
public class Vision_Testing extends Robot {

    @Override
    public void op_mode() {

        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Drop position", getDropPosition());
            telemetry.addData("value", pipeline.avg1);
            telemetry.update();
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }


    }
}
