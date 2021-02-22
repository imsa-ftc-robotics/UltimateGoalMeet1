package org.firstinspires.ftc.teamcode.TestingAndSamples;

import android.graphics.Region;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Archive.Robot;
import org.firstinspires.ftc.teamcode.RegionalsBot;

@TeleOp
//@Disabled
public class ServoTesting extends LinearOpMode {

    RegionalsBot robot = new RegionalsBot(hardwareMap);

    @Override
    public void runOpMode() {
        waitForStart();
        while(!isStopRequested()){
            robot.transferServo.setPosition(gamepad1.left_trigger);
            telemetry.addData("position", robot.transferServo.getPosition());
            telemetry.update();

            /*tray
            down: 0.0
            normal: 0.41
            up: 0.64
             */

            /*
            0.04
            0.4
             */

            /* WOBBLE
            completely open: 0
            closed: 0.85
            half open: 0.5
            * */
        }
    }
}
