package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.DropPosition;

import java.util.ServiceLoader;

@Autonomous
public class RudimentaryMeet1 extends Robot  {
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void op_mode() {
        wobbleGoalMotor.setDirection(DcMotorEx.Direction.REVERSE);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        runtime.reset();
        double value = pipeline.avg1;
        DropPosition position;
        position=getDropPosition();
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
                break;
            }
        }
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("position", position);
        packet.put("value", value);
        packet.put("loop?", "no");
        dashboard.sendTelemetryPacket(packet);
        value = pipeline.avg1;
        position = getDropPosition();
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
            case Far:
                strafe(0.5, 400);
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*10.1));
                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.001, 0);
                sleep(300);

                strafeAngle(0.5,   180, 1700);

                sleep(300);

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(500);
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
                strafe(-0.4, 1600);
                sleep(700);

                wobbleGoalServo.setPosition(WOBBLE_CLOSED);
                sleep(700);

                wobbleGoalMotor.setPower(-0.4);
                sleep(450);
                wobbleGoalMotor.setPower(0);

                strafe(-0.5, 600);
                reorientIMU(0, -0.5,0.5,0.5, 1.5, 0.001, 0);
                moveToPosition(0.7, (int)(TICKS_PER_INCH*12*9.55));

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(500);

                strafe(0.4, 500);

                moveToPosition(0.5, (int)(-TICKS_PER_INCH*12*4.2));
                break;

            case Middle:
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
            case Near:
                moveToPosition(0.7, TICKS_PER_INCH*12*6);
                sleep(300);

                reorientIMU(0, -0.5, 0.5, 0.5, 1.5, 0.001, 0);
                strafeAngle(0.5, 180, 1400);
                sleep(300);

                wobbleGoalServo.setPosition(WOBBLE_OPEN);
                sleep(400);

                strafeAngle(0.5,0,250);

                wobbleGoalMotor.setPower(0.4);
                sleep(350);
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

            default:

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
