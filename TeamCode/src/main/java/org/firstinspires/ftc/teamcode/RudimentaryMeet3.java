package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Archive.Robot;

@Autonomous
//extending the superclass
public class RudimentaryMeet3 extends RobotV2  {
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void op_mode() {
        //setting the direction of the wobble goal motor and creating an instance of the FTC dashboard
        wobbleGoalMotor.setDirection(DcMotorEx.Direction.REVERSE);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        UGContourRingPipeline.Height height = UGContourRingPipeline.Height.ONE;
        while (!isStarted() && !isStopRequested()){
            height = pipeline.getHeight();
            telemetry.addData("[Ring Stack] >>", height);
            telemetry.update();
            idle();
        }

        waitForStart();


        //grab onto wobble goal
        wobbleGoalMotor.setPower(-0.4);
        sleep(550);
        wobbleGoalMotor.setPower(0);

        wobbleGoalServo.setPosition(WOBBLE_CLOSED);
        sleep(1000);

        wobbleGoalMotor.setPower(-0.4);
        sleep(500);
        wobbleGoalMotor.setPower(0);



        switch (height){
            //if its far (4 rings), go to C
            case FOUR:
                strafe(0.5, 400);
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*10.1));
                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.003, 0);
                sleep(300);

                strafeAngle(0.5,   180, 1450);

                sleep(300);
                //releasing wobble goal
                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(500);
                //goes back for second wobble goal
                moveToPosition(0.7, (int)(-TICKS_PER_INCH*12*5.42));
                //wobbleGoalMotor.setPower(0.4);
                //sleep(400);
                //wobbleGoalMotor.setPower(0);

                strafe(0.5, 1700);
                moveToPosition(0.5, (int)(-TICKS_PER_INCH*12*3.5));
                wobbleGoalMotor.setPower(0.4);
                sleep(250);
                wobbleGoalMotor.setPower(0);
                wobbleGoalServo.setPosition(WOBBLE_HALF);
                reorientIMU(0.75, -0.5,0.5,0.5, 1.5, 0.001, 0);
                strafe(-0.4, 1650);
                reorientIMU(0, -0.7, 0.7, 0.5, 1.5, 0.005, 0);
                sleep(500);

                wobbleGoalServo.setPosition(WOBBLE_CLOSED);
                sleep(700);

                wobbleGoalMotor.setPower(-0.4);
                sleep(450);
                wobbleGoalMotor.setPower(0);
                //moving the robot to C
                strafe(-0.5, 600);
                reorientIMU(-1, -0.5,0.5,0.5, 1.5, 0.005, 0);
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*9));

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(500);

                strafe(0.4, 500);

                moveToPosition(0.5, (int)(-TICKS_PER_INCH*12*4));
                break;

             //if 1 rings, go to B
            case ONE:
                strafe(0.5, 400);
                //robot moves to B
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*8.1));
                sleep(300);

                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.005, 0);
                strafingPID(-0.5, 300,1,0,0);

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(300);
                wobbleGoalMotor.setPower(0.4);
                sleep(350);
                wobbleGoalMotor.setPower(0);
                strafingPID(0.5, 500,1,0,0);
                moveToPosition(0.7, (int)(-TICKS_PER_INCH*12*6.95));
                wobbleGoalServo.setPosition(WOBBLE_HALF);
                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.005, 0);
                strafingPID(-0.4, 1500, 1,0,0);
                sleep(100);
                wobbleGoalServo.setPosition(WOBBLE_CLOSED);
                sleep(500);
                wobbleGoalMotor.setPower(-0.4);
                sleep(500);
                wobbleGoalMotor.setPower(0);
                sleep(200);


                strafe(0.4, 1300);
                sleep(200);
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*6.65));


                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(200);
                wobbleGoalMotor.setPower(0.4);
                sleep(400);
                wobbleGoalMotor.setPower(0);



                moveToPosition(0.5, (int)(-TICKS_PER_INCH*12*2.75));


                shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooter.setVelocity(1470);
                wrapIMU(180, -0.5, 0.5, 0.5, 1, 0.001, 0);
                strafingPID(0.5, 1100,1,0,0);
                transferServo.setPosition(0);
                transfer.setPower(-0.7);
                wrapIMU(180, -0.5, 0.5, 0.5, 1, 0.005, 0);
                sleep(2200);
                transfer.setPower(0);
                shooter.setPower(0);
                moveWithEncoders(-0.5, 400);
                break;
             //if 0 rings, go to A
            case ZERO:
                //robot moves to A
                moveToPosition(0.7, TICKS_PER_INCH*12*6);
                sleep(300);

                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.001, 0);
                //strafing left
                strafingPID(-0.5, 1300, 1.2, 0, 0);
                sleep(300);

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(400);

                strafingPID(0.5,550,1.3, 0,0);

                wobbleGoalMotor.setPower(0.4);
                sleep(300);
                wobbleGoalMotor.setPower(0);

                moveToPosition(0.7, (int)(-TICKS_PER_INCH*12*2.4));
                sleep(300);

                reorientIMU(90, -0.5, 0.5, 1, 1.5, 0, 0);
                sleep(200);

                wobbleGoalServo.setPosition(WOBBLE_HALF);
                sleep(200);
                strafingPID(-0.4, 975, 1.2, 0, 0);
                sleep(1000);
                wobbleGoalServo.setPosition(WOBBLE_CLOSED);
                sleep(500);

                wobbleGoalMotor.setPower(-0.4);
                sleep(500);
                wobbleGoalMotor.setPower(0);

                reorientIMU(0, -0.5, 0.5, 1, 1.5, 0, 0);
                sleep(200);

                moveToPosition(0.6, (int)(TICKS_PER_INCH*12*3.02));
                sleep(500);


                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(200);

                moveToPosition(0.3, -TICKS_PER_INCH*4);

                shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooter.setVelocity(1470
                );
                wrapIMU(180, -0.5, 0.5, 0.7, 1.2, 0, 0);
                transferServo.setPosition(0);
                transfer.setPower(-0.7);
                sleep(3000);
                strafeAngle(-0.5,0, 500);
                transfer.setPower(0);
                shooter.setPower(0);
                moveWithEncoders(-0.3, 700);
                break;

        }
    }
}
