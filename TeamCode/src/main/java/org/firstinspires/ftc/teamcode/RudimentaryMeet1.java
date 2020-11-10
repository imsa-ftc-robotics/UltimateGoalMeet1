package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.DropPosition;

public class RudimentaryMeet1 extends Robot  {


    @Override
    public void op_mode() {

        waitForStart();
        strafe(0.4, 400);

        //grab onto wobble goal
        wobbleGoalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleGoalMotor.setPower(-0.5);
        sleep(500);
        wobbleGoalMotor.setPower(0);

        sleep(300);

        wobbleGoalServo.setPosition(WOBBLE_CLOSED);
        sleep(300);

        wobbleGoalMotor.setPower(-0.5);
        sleep(400);
        wobbleGoalMotor.setPower(0);

        switch (getDropPosition()){
            case Far:
                moveToPosition(0.7, TICKS_PER_INCH*12*10);
                sleep(300);
                strafeAngle(0.5, 0, 1200);
                sleep(300);

                wobbleGoalServo.setPosition(WOBBLE_OPEN);

                moveToPosition(0.7, -TICKS_PER_INCH*12*5);

            case Middle:
                moveToPosition(0.7, TICKS_PER_INCH*12*8);
                sleep(300);
                strafeAngle(0.5, 0, 700);
                sleep(300);
                wobbleGoalServo.setPosition(WOBBLE_OPEN);

                moveToPosition(0.7, TICKS_PER_INCH*12*4);

            case Near:
                moveToPosition(0.7, TICKS_PER_INCH*12*5);
                sleep(300);
                strafeAngle(0.5, 0, 1200);
                sleep(300);
                wobbleGoalServo.setPosition(WOBBLE_OPEN);
        }
    }
}
