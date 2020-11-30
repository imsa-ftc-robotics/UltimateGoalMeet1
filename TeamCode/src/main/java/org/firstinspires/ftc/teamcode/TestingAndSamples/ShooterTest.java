package org.firstinspires.ftc.teamcode.TestingAndSamples;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
@Disabled
public class ShooterTest extends Robot {
    public void op_mode(){
        waitForStart();
        while(opModeIsActive()){
            shooter.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
        }
    }
}
