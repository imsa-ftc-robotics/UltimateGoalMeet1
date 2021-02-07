package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Archive.Robot;

@TeleOp
@Disabled
public class LM2Teleop extends Robot {

    public void op_mode(){


        waitForStart();
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        wobbleGoalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double shift;
        double reverse;
        while (opModeIsActive()){
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            if (gamepad1.right_bumper){
                shift = 0.5;
            }
            else {
                shift = 1;
            }

            if (gamepad1.left_bumper){
                reverse = -1;
            }
            else {
                reverse = 1;
            }


            double leftBackPower = Range.clip((((drive*reverse) + turn - (strafe*reverse))*shift), -1.0, 1.0);
            double rightBackPower = Range.clip((((drive*reverse) - turn + (strafe*reverse))*shift), -1.0, 1.0);
            double leftFrontPower = Range.clip((((drive*reverse) + turn + (strafe*reverse))*shift), -1.0, 1.0);
            double rightFrontPower = Range.clip((((drive*reverse) - turn - (strafe*reverse))*shift*reverse), -1.0, 1.0);

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

            double wobbleGoalMotorPower = -gamepad2.right_stick_y*0.5;
            wobbleGoalMotor.setPower(wobbleGoalMotorPower);

            if(gamepad1.a){
                shooter.setVelocity(-2800);
            }
            else if (gamepad1.b){
                shooter.setVelocity(-1400);
            }
            else if (gamepad1.x){
                shooter.setPower(0);
            }
            else if (gamepad2.y){
                shooter.setPower(0);
            }

            //intake winch
            if (gamepad2.right_bumper){
                intakeWinch.setPower(1);
            }
            else if (gamepad2.left_bumper){
                intakeWinch.setPower(-1);
            }
            else {intakeWinch.setPower(0);}

            transfer.setPower(gamepad2.left_stick_y);

            telemetry.addData("shooter encoder", shooter.getCurrentPosition());
            telemetry.addData("shooter velocity", shooter.getVelocity());
            telemetry.update();

        }
    }
}