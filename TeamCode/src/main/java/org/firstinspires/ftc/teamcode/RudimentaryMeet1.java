package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.DropPosition;

public class RudimentaryMeet1 extends Robot  {


    @Override
    public void op_mode() {

        waitForStart();
        //grab onto wobble goal
        wobbleGoalServo.setPosition(WOBBLE_CLOSED);
        switch (getDropPosition()){
            case Far:
                moveToPosition(0.7, TICKS_PER_INCH*12*9);
                sleep(300);
                strafeAngle(0.5, 0, 1200);
                sleep(300);

                wobbleGoalServo.setPosition(WOBBLE_OPEN);

                moveToPosition(0.7, -TICKS_PER_INCH*12*4);

        }
    }
}
