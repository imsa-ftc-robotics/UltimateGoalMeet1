package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class DrivetrainTest extends Robot {

    public void op_mode(){
        waitForStart();
        while(!isStopRequested()) {
            leftBackDrive.setPower(gamepad1.left_stick_y);
            leftFrontDrive.setPower(gamepad1.left_trigger);
            rightBackDrive.setPower(gamepad1.right_stick_y);
            rightFrontDrive.setPower(gamepad1.right_trigger);
        }
    }
}
