package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ShooterTest extends Robot {
    public void op_mode(){
        waitForStart();
        while(opModeIsActive()){
            shooter.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
        }
    }
}
