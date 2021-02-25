package org.firstinspires.ftc.teamcode;

import android.graphics.Region;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RollerTesting extends LinearOpMode {

    public void runOpMode(){
        RegionalsBot robot = new RegionalsBot(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            robot.bottomRoller.setPower(1);
            sleep(2000);
            robot.bottomRoller.setPower(-1);
            sleep(2000);
        }
    }
}
