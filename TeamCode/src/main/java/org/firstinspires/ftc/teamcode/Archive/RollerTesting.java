package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Archive.RegionalsBot;

@TeleOp
@Disabled
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
