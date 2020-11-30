package org.firstinspires.ftc.teamcode.TestingAndSamples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
@Disabled
public class TransferTest extends Robot {

    public void op_mode(){

        waitForStart();

        while (!isStopRequested()){
            transfer.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            intake.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
        }
    }
}
