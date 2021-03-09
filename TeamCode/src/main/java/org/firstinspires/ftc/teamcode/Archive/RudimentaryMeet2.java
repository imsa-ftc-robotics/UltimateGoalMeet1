package org.firstinspires.ftc.teamcode.Archive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
//extending the superclass
@Disabled
public class RudimentaryMeet2 extends Robot  {
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
        wobbleGoalMotor.setPower(-0.4);
        sleep(550);
        wobbleGoalMotor.setPower(0);

        wobbleGoalServo.setPosition(WOBBLE_CLOSED);
        sleep(1000);

        wobbleGoalMotor.setPower(-0.4);
        sleep(500);
        wobbleGoalMotor.setPower(0);



        switch (position){
            //if its far (4 rings), go to C
            case Far:
                strafe(0.5, 400);
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*10.1));
                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.001, 0);
                sleep(300);

                strafeAngle(0.5,   180, 1650);

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
                strafe(-0.4, 1550);
                sleep(500);

                wobbleGoalServo.setPosition(WOBBLE_CLOSED);
                sleep(700);

                wobbleGoalMotor.setPower(-0.4);
                sleep(450);
                wobbleGoalMotor.setPower(0);
                //moving the robot to C
                strafe(-0.5, 500);
                reorientIMU(0, -0.5,0.5,0.5, 1.5, 0.001, 0);
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*9.4));

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(500);

                strafe(0.4, 500);

                moveToPosition(0.5, (int)(-TICKS_PER_INCH*12*4));
                break;

             //if 1 rings, go to B
            case Middle:
                strafe(0.5, 400);
                //robot moves to B
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*8.1));
                sleep(300);

                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.005, 0);
                strafingPID(-0.5, 500, 1,0,0);
                sleep(300);

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(300);
                wobbleGoalMotor.setPower(0.4);
                sleep(400);
                wobbleGoalMotor.setPower(0);
                strafingPID(0.5, 500,1,0,0);
                moveToPosition(0.7, (int)(-TICKS_PER_INCH*12*6.89));
                wobbleGoalServo.setPosition(WOBBLE_HALF);
                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.005, 0);
                strafe(-0.4, 1400);
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
                shooter.setVelocity(1630);
                wrapIMU(180, -0.5, 0.5, 1, 1.5, 0, 0);
                strafe(0.5, 1350);
                transferServo.setPosition(0.4);
                transfer.setPower(-0.7);
                sleep(3000);
                transfer.setPower(0);
                shooter.setPower(0);
                moveWithEncoders(-0.3, 400);
                break;
             //if 0 rings, go to A
            case Near:
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
                shooter.setVelocity(1630);
                wrapIMU(180, -0.5, 0.5, 1, 1.5, 0, 0);
                transferServo.setPosition(0.4);
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
