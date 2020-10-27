package org.firstinspires.ftc.teamcode;

public class ServoTesting extends Robot {

    @Override
    public void op_mode() {
        waitForStart();
        while(!isStopRequested()){
            wobbleGoalServo.setPosition(gamepad1.left_trigger);
            //tray.setPosition(gamepad1.right_trigger);
        }
    }
}
