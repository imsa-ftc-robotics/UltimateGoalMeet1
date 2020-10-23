package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Default_Teleop extends Robot{


    @Override
    public void op_mode() {
        double speedmultiplier = 1;
        waitForStart();
        while (opModeIsActive()){
            // POV Mode uses left stick for translation, and right stick to turn.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;


            double leftBackPower = Range.clip(((drive + turn - strafe)), -1.0, 1.0);
            double rightBackPower = Range.clip(((drive - turn + strafe)), -1.0, 1.0);
            double leftFrontPower = Range.clip(((drive + turn + strafe)), -1.0, 1.0);
            double rightFrontPower = Range.clip(((drive - turn - strafe)), -1.0, 1.0);


            leftBackPower *= speedmultiplier;
            leftFrontPower *= speedmultiplier;
            rightBackPower *= speedmultiplier;
            rightFrontPower *= speedmultiplier;


            leftBackDrive.setPower(leftBackPower);
            leftFrontDrive.setPower(leftFrontPower);
            rightBackDrive.setPower(rightBackPower);
            rightFrontDrive.setPower(rightFrontPower);


            telemetry.addData("Left Back Power", leftBackPower);
            telemetry.addData("Left Front Power", leftFrontPower);
            telemetry.addData("Right Back Power", rightBackPower);
            telemetry.addData("Right Front Power", rightFrontPower);
            telemetry.update();

        }
    }
}
