package org.firstinspires.ftc.teamcode.TestingAndSamples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
//@Disabled
public class ServoTesting extends Robot {

    @Override
    public void op_mode() {
        waitForStart();
        while(!isStopRequested()){
            wobbleGoalServo.setPosition(gamepad1.left_trigger);
            telemetry.addData("wobble", wobbleGoalServo.getPosition());
            telemetry.update();

            /*tray
            down: 0.0
            normal: 0.41
            up: 0.64
             */

            /* WOBBLE
            completely open: 0
            closed: 0.85
            half open: 0.5
            * */
        }
    }
}
