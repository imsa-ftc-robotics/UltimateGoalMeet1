package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.DropPosition;

import java.util.ServiceLoader;

@Autonomous
public class RudimentaryMeet1 extends Robot  {


    @Override
    public void op_mode() {
        wobbleGoalMotor.setDirection(DcMotorEx.Direction.REVERSE);
        waitForStart();

        //grab onto wobble goal
        //wobbleGoalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleGoalMotor.setPower(-0.4);
        sleep(450);
        wobbleGoalMotor.setPower(0);

        wobbleGoalServo.setPosition(WOBBLE_CLOSED);
        sleep(1000);

        wobbleGoalMotor.setPower(-0.4);
        sleep(500);
        wobbleGoalMotor.setPower(0);

        strafe(0.5, 300);

        int positionHolder = 2;
        switch (positionHolder){
            case 2:
                strafe(0.5, 400);
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*10.1));
                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.001, 0);
                sleep(300);

                strafeAngle(0.5,   180, 1500);

                sleep(300);

                wobbleGoalServo.setPosition(WOBBLE_OPEN);

                moveToPosition(0.7, (int)(-TICKS_PER_INCH*12*5.4));
                wobbleGoalMotor.setPower(0.4);
                sleep(450);
                wobbleGoalMotor.setPower(0);

                strafe(0.5, 1500);
                moveToPosition(0.5, (int)(-TICKS_PER_INCH*12*3.1));
                reorientIMU(0, -0.5,0.5,0.5, 1.5, 0.001, 0);
                strafe(-0.5, 1200);
                sleep(500);

                wobbleGoalServo.setPosition(WOBBLE_CLOSED);
                sleep(500);

                wobbleGoalMotor.setPower(-0.4);
                sleep(400);
                wobbleGoalMotor.setPower(0);

                strafe(-0.5, 700);
                break;

            case 1:
                strafe(0.5, 400);
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*8.1));
                sleep(300);

                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.001, 0);
                strafeAngle(0.5, 180, 600);
                sleep(300);

                wobbleGoalMotor.setPower(0.4);
                sleep(400);
                wobbleGoalMotor.setPower(0);
                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(300);
                strafe(0.5, 500);
                sleep(200);
                moveToPosition(0.7, (int)(-TICKS_PER_INCH*12*6.65));
                wobbleGoalServo.setPosition(WOBBLE_HALF);

                strafe(-0.4, 1500);
                sleep(800);

                wobbleGoalServo.setPosition(WOBBLE_CLOSED);
                sleep(500);

                wobbleGoalMotor.setPower(-0.4);
                sleep(500);
                wobbleGoalMotor.setPower(0);
                sleep(200);


                strafe(0.4, 1300);
                sleep(200);
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*6.65));

                wobbleGoalMotor.setPower(0.4);
                sleep(400);
                wobbleGoalMotor.setPower(0);
                sleep(200);

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(200);

                strafeAngle(0.5,0, 500);
                sleep(200);

                moveToPosition(0.5, (int)(-TICKS_PER_INCH*12*2.25));

                break;

            case 0:
                moveToPosition(0.7, TICKS_PER_INCH*12*6);
                sleep(300);

                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.001, 0);
                strafeAngle(0.5, 180, 1400);
                sleep(300);

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(400);

                strafeAngle(0.5,0,450);

                wobbleGoalMotor.setPower(0.4);
                sleep(450);
                wobbleGoalMotor.setPower(0);

                moveToPosition(0.7, (int)(-TICKS_PER_INCH*12*2.2));
                sleep(300);

                reorientIMU(90, -0.5, 0.5, 1, 1.5, 0, 0);
                sleep(200);

                wobbleGoalServo.setPosition(WOBBLE_HALF);
                sleep(200);
                strafeAngle(0.5, 180, 650);
                sleep(1000);
                wobbleGoalServo.setPosition(WOBBLE_CLOSED);
                sleep(500);

                wobbleGoalMotor.setPower(-0.4);
                sleep(500);
                wobbleGoalMotor.setPower(0);

                reorientIMU(0, -0.5, 0.5, 1, 1.5, 0, 0);
                sleep(200);

                moveToPosition(0.6, (int)(TICKS_PER_INCH*12*2.7));
                sleep(1000);

                wobbleGoalMotor.setPower(0.4);
                sleep(400);
                wobbleGoalMotor.setPower(0);
                sleep(200);

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(200);

                strafeAngle(0.5,0, 500);
                break;
        }
    }
}
