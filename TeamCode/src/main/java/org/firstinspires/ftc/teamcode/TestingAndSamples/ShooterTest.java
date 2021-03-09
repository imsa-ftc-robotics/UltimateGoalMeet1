package org.firstinspires.ftc.teamcode.TestingAndSamples;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Archive.RegionalsBot;

@TeleOp
//@Disabled
public class ShooterTest extends LinearOpMode {
    public void runOpMode(){
        RegionalsBot robot = new RegionalsBot(hardwareMap);


        robot.shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()){
            robot.shooter1.setPower(1);
            robot.shooter2.setPower(1);

            telemetry.addData("shooter velo", robot.shooter1.getVelocity());
            telemetry.update();
        }
    }
}
