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

            double IntakePower = gamepad2.left_trigger-gamepad2.right_trigger;
            intake.setPower(IntakePower);

            if (gamepad2.a)
                wobbleGoalServo.setPosition(WOBBLE_CLOSED);
            if(gamepad2.b)
                wobbleGoalServo.setPosition(WOBBLE_HALF);
            if (gamepad2.x)
                wobbleGoalMotor.setPower(WOBBLE_OPEN);

            double wobbleGoalMotorPower = -gamepad2.right_stick_y;
            wobbleGoalMotor.setPower(wobbleGoalMotorPower);

            double armPower = gamepad2.left_stick_y;
            arm.setPower(armPower);

            if (gamepad2.dpad_up)
                tray.setPosition(TRAY_NORMAL);
            if(gamepad2.dpad_right)
                tray.setPosition(TRAY_UP);
            if (gamepad2.dpad_down)
                tray.setPosition(TRAY_DOWN);

            //intake winch
            double intakeWinchPower = gamepad1.right_trigger-gamepad1.left_trigger;
            intakeWinch.setPower(intakeWinchPower);
        }

    }
}
