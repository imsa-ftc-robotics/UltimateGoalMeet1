package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TransferTest extends Robot {

    public void op_mode(){

        waitForStart();

        while (!isStopRequested()){
            transfer.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            intake.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
        }
    }
}
