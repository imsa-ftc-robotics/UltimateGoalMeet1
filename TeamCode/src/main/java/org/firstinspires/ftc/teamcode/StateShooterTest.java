package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
//@Disabled
public class StateShooterTest extends LinearOpMode {

    public void runOpMode(){

        StateBot robot = new StateBot(hardwareMap);

        robot.shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        double shooterPower = 0;
        while(!isStopRequested()){
            shooterPower = gamepad1.right_trigger - gamepad1.left_trigger;

            robot.shooter1.setPower(shooterPower);
            robot.shooter2.setPower(shooterPower);
        }

    }
}
