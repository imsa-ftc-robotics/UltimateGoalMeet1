package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.DropPosition;

@Autonomous
public class RudimentaryMeet1 extends Robot  {


    @Override
    public void op_mode() {

        waitForStart();

        //grab onto wobble goal
        wobbleGoalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleGoalMotor.setPower(-0.3);
        sleep(400);
        wobbleGoalMotor.setPower(0);

        wobbleGoalServo.setPosition(WOBBLE_CLOSED);
        sleep(1000);

        wobbleGoalMotor.setPower(-0.4);
        sleep(400);
        wobbleGoalMotor.setPower(0);

        strafe(0.5, 400);

        int positionHolder = 0;
        switch (positionHolder){
            case 2:
                moveToPosition(0.7, TICKS_PER_INCH*12*10);
                sleep(300);
                strafeAngle(0.5, 0, 1200);
                sleep(300);

                wobbleGoalServo.setPosition(WOBBLE_OPEN);

                moveToPosition(0.7, -TICKS_PER_INCH*12*5);

            case 1:
                moveToPosition(0.7, TICKS_PER_INCH*12*8);
                sleep(300);
                strafeAngle(0.5, 180, 500);
                sleep(300);
                wobbleGoalServo.setPosition(WOBBLE_OPEN);

                moveToPosition(0.7, TICKS_PER_INCH*12*4);

            case 0:
                moveToPosition(0.7, TICKS_PER_INCH*12*6);
                sleep(300);
                strafeAngle(0.5, 180, 1400);
                sleep(300);
                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(400);
                strafeAngle(0.5, 0, 1400);

        }
    }
}
