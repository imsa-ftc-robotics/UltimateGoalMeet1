package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Nov16Teleop extends Robot{

    public void op_mode(){


        waitForStart();
        //shooter.setPower(-0.4);
        while (opModeIsActive()){
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;


            double leftBackPower = Range.clip(((drive + turn - strafe)*0.5), -1.0, 1.0);
            double rightBackPower = Range.clip(((drive - turn + strafe)*0.5), -1.0, 1.0);
            double leftFrontPower = Range.clip(((drive - turn - strafe)*0.5), -1.0, 1.0);
            double rightFrontPower = Range.clip(((drive + turn + strafe)*0.5), -1.0, 1.0);

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
                wobbleGoalServo.setPosition(WOBBLE_OPEN);

            double wobbleGoalMotorPower = -gamepad2.right_stick_y;
            wobbleGoalMotor.setPower(wobbleGoalMotorPower);
/*
            if(gamepad1.a){
                shooter.setPower(-1);
            }
            else if (gamepad1.b){
                shooter.setPower(-0.4);
            }
            else if (gamepad1.x){
                shooter.setPower(0);
            }
*/
            //intake winch
            if (gamepad1.right_bumper){
                intakeWinch.setPower(1);
            }
            else if (gamepad1.left_bumper){
                intakeWinch.setPower(-1);
            }
            else {intakeWinch.setPower(0);}

            transfer.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

        }
    }
}
