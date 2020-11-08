package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Default_Teleop extends OpMode7573 {

    double speedmultiplier = 1;
    MotorRampToPower shooter_controller;

    public void init7573() {
        this.shooter.setPower(0);
    }

    @Override
    public void start7573() {
        this.shooter_controller = new MotorRampToPower(this.shooter, 0.667, -0.4);
        this.futures.spawn(shooter_controller);
    }

    @Override
    public void loop7573() {
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;


        double leftBackPower = Range.clip(((drive + turn - strafe)*0.5), -1.0, 1.0);
        double rightBackPower = Range.clip(((drive - turn + strafe)*0.5), -1.0, 1.0);
        double leftFrontPower = Range.clip(((drive + turn + strafe)*0.5), -1.0, 1.0);
        double rightFrontPower = Range.clip(((drive - turn - strafe)*0.5), -1.0, 1.0);


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

        if(gamepad1.a){
            shooter_controller.setTarget(-1);
        }
        else if (gamepad1.b){
            shooter_controller.setTarget(-0.4);
        }
        else if (gamepad1.x){
            shooter_controller.setTarget(0);
        }

        //intake winch
        double intakeWinchPower = gamepad1.right_trigger-gamepad1.left_trigger;
        intakeWinch.setPower(intakeWinchPower);
    }

    @Override
    public void end7573() {
        this.shooter_controller.cancel();
        this.shooter.setPower(0);
    }
}
