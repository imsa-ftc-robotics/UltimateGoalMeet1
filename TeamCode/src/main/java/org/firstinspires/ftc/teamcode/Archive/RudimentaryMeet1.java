package org.firstinspires.ftc.teamcode.Archive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Archive.Robot;
import org.firstinspires.ftc.teamcode.DropPosition;

@Autonomous
@Disabled
//extending the superclass

public class RudimentaryMeet1 extends Robot  {
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void op_mode() {
        //setting the direction of the wobble goal motor and creating an instance of the FTC dashboard
        wobbleGoalMotor.setDirection(DcMotorEx.Direction.REVERSE);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        runtime.reset();
        //getting the value of the orange
        double value = pipeline.avg1;
        //creates the drop position variable
        DropPosition position;
        //gets the drop position
        position=getDropPosition();
        //making sure the value for the orange is not zero
        while (value ==0 && !isStopRequested()) {
            value = pipeline.avg1;
            position = getDropPosition();
            telemetry.addData("position", position);
            telemetry.addData("value", value);
            telemetry.addData("still", "in loop");
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("position", position);
            packet.put("value", value);
            packet.put("loop?", "yes");
            dashboard.sendTelemetryPacket(packet);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (getRuntime() > 5000){
                position = DropPosition.Middle;
                break;
            }
        }
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("position", position);
        packet.put("value", value);
        packet.put("loop?", "no");
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("position", position);
        telemetry.addData("value", value);
        telemetry.addData("still", "no");
        telemetry.update();

        //grab onto wobble goal
        //wobbleGoalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleGoalMotor.setPower(-0.4);
        sleep(500);
        wobbleGoalMotor.setPower(0);

        wobbleGoalServo.setPosition(WOBBLE_CLOSED);
        sleep(1000);

        wobbleGoalMotor.setPower(-0.4);
        sleep(500);
        wobbleGoalMotor.setPower(0);

        strafe(0.5, 300);


        switch (position){
            //if its far (4 rings), go to C
            case Far:
                strafe(0.5, 400);
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*10.1));
                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.001, 0);
                sleep(300);

                strafeAngle(0.5,   180, 1700);

                sleep(300);
                //releasing wobble goal
                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(500);
                //goes back for second wobble goal
                moveToPosition(0.7, (int)(-TICKS_PER_INCH*12*5.4));
                //wobbleGoalMotor.setPower(0.4);
                //sleep(400);
                //wobbleGoalMotor.setPower(0);

                strafe(0.5, 1700);
                moveToPosition(0.5, (int)(-TICKS_PER_INCH*12*3.5));
                wobbleGoalMotor.setPower(0.4);
                sleep(250);
                wobbleGoalMotor.setPower(0);
                wobbleGoalServo.setPosition(WOBBLE_HALF);
                reorientIMU(0, -0.5,0.5,0.5, 1.5, 0.001, 0);
                strafe(-0.4, 1760);
                sleep(1000);

                wobbleGoalServo.setPosition(WOBBLE_CLOSED);
                sleep(700);

                wobbleGoalMotor.setPower(-0.4);
                sleep(450);
                wobbleGoalMotor.setPower(0);
                //moving the robot to C
                strafe(-0.5, 620);
                reorientIMU(0, -0.5,0.5,0.5, 1.5, 0.001, 0);
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*9.55));

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(500);

                strafe(0.4, 500);

                moveToPosition(0.5, (int)(-TICKS_PER_INCH*12*4.2));
                break;

             //if 1 rings, go to B
            case Middle:
                strafe(0.5, 400);
                //robot moves to B
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*8.1));
                sleep(300);

                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.001, 0);
                strafeAngle(0.5, 180, 600);
                sleep(300);

                //wobbleGoalMotor.setPower(0.4);
                //sleep(400);
                //wobbleGoalMotor.setPower(0);
                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(300);
                strafe(0.5, 500);
                sleep(200);
                moveToPosition(0.7, (int)(-TICKS_PER_INCH*12*6.77));
                wobbleGoalServo.setPosition(WOBBLE_HALF);

                strafe(-0.4, 1690);
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
             //if 0 rings, go to A
            case Near:
                //robot moves to A
                moveToPosition(0.7, TICKS_PER_INCH*12*6);
                sleep(300);

                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.001, 0);
                //strafing left
                strafeAngle(0.5, 180, 1450);
                sleep(300);

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(400);

                strafeAngle(0.5,0,200);

                wobbleGoalMotor.setPower(0.4);
                sleep(200);
                wobbleGoalMotor.setPower(0);

                moveToPosition(0.7, (int)(-TICKS_PER_INCH*12*2.2));
                sleep(300);

                reorientIMU(90, -0.5, 0.5, 1, 1.5, 0, 0);
                sleep(200);

                wobbleGoalServo.setPosition(WOBBLE_HALF);
                sleep(200);
                strafeAngle(0.4, 180, 975);
                sleep(1000);
                wobbleGoalServo.setPosition(WOBBLE_CLOSED);
                sleep(500);

                wobbleGoalMotor.setPower(-0.4);
                sleep(500);
                wobbleGoalMotor.setPower(0);

                reorientIMU(0, -0.5, 0.5, 1, 1.5, 0, 0);
                sleep(200);

                moveToPosition(0.6, (int)(TICKS_PER_INCH*12*3.02));
                sleep(1000);

                /*wobbleGoalMotor.setPower(0.4);
                sleep(400);
                wobbleGoalMotor.setPower(0);
                sleep(200);*/

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(200);

                strafeAngle(0.5,0, 500);
                break;

            default:
                //same thing as middle case
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
                moveToPosition(0.7, (int)(-TICKS_PER_INCH*12*6.77));
                wobbleGoalServo.setPosition(WOBBLE_HALF);

                strafe(-0.4, 1650);
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
        }
    }
}
