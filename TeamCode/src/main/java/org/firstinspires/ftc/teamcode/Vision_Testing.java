package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Vision_Testing extends Robot{

    @Override
    public void op_mode() {
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Drop position", getDropPosition());
            telemetry.update();
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }


    }
}
