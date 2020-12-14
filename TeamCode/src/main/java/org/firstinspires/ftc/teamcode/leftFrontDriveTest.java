package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class leftFrontDriveTest extends Robot {
    public void op_mode(){
        waitForStart();
        while(opModeIsActive()){
            leftFrontDrive.setPower(gamepad1.left_stick_y);
        }
    }

}
