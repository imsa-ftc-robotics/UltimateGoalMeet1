package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
//@Disabled
public class ServoTesting extends Robot {

    @Override
    public void op_mode() {
        waitForStart();
        while(!isStopRequested()){
            //wobbleGoalServo.setPosition(gamepad1.left_trigger);
            tray.setPosition(gamepad1.right_trigger);
            //telemetry.addData("wobble", wobbleGoalServo.getPosition());
            telemetry.addData("tray", tray.getPosition());
            telemetry.update();

            /*tray
            down: 0.0
            normal: 0.41
            up: 0.64
             */

            /* WOBBLE
            completely open: 0.78
            closed: 0.03
            half open: 0.2
            * */
        }
    }
}
