package org.firstinspires.ftc.teamcode;

import android.graphics.Region;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.geometry.euclidean.twod.Line;

@TeleOp
//@Disabled
public class RegionalsTeleop extends LinearOpMode {

    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    public void runOpMode(){
        RegionalsBot robot = new RegionalsBot(hardwareMap);

        VelocityPIDFController veloController = new VelocityPIDFController(robot.SHOOTER_VELO_PID, robot.kV, robot.kA, robot.kStatic);

        robot.shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double targetVelo = 0;



        waitForStart();

        double shift;
        double reverse = 1;
        while (opModeIsActive()){
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            if (gamepad1.right_bumper){
                shift = 0.4;
            }
            else if (gamepad1.left_bumper){
                shift = 0.3;
            }
            else {
                shift =1;
            }


            double leftBackPower = Range.clip((((drive*reverse) + turn - (strafe*reverse))*shift), -1.0, 1.0);
            double rightBackPower = Range.clip((((drive*reverse) - turn + (strafe*reverse))*shift), -1.0, 1.0);
            double leftFrontPower = Range.clip((((drive*reverse) + turn + (strafe*reverse))*shift), -1.0, 1.0);
            double rightFrontPower = Range.clip((((drive*reverse) - turn - (strafe*reverse))*shift*reverse), -1.0, 1.0);

            robot.leftRear.setPower(leftBackPower);
            robot.leftFront.setPower(leftFrontPower);
            robot.rightRear.setPower(rightBackPower);
            robot.rightFront.setPower(rightFrontPower);


            telemetry.addData("Left Back Power", leftBackPower);
            telemetry.addData("Left Front Power", leftFrontPower);
            telemetry.addData("Right Back Power", rightBackPower);
            telemetry.addData("Right Front Power", rightFrontPower);
            telemetry.update();

            double intakePower = gamepad2.left_trigger-gamepad2.right_trigger;
            robot.intake.setPower(intakePower);
            robot.bottomRoller.setPower(-intakePower);

            if (gamepad2.a)
                robot.wobbleGoalServo.setPosition(robot.WOBBLE_CLOSED);
            if (gamepad2.x)
                robot.wobbleGoalServo.setPosition(robot.WOBBLE_OPEN);

            double wobbleGoalMotorPower = -gamepad2.right_stick_y;
            robot.wobbleGoalMotor1.setPower(wobbleGoalMotorPower);
            robot.wobbleGoalMotor2.setPower(wobbleGoalMotorPower);


            ///// Run the velocity controller ////

            if(gamepad1.a){
                targetVelo = 1800;
            }
            else if (gamepad1.b){
                targetVelo = 1500;
            }
            else if (gamepad1.x){
                targetVelo = 0.4*2200;
            }
            else if (gamepad2.y){
                targetVelo = 0;
            }

            // Call necessary controller methods
            veloController.setTargetVelocity(targetVelo);
            veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
            veloTimer.reset();


            lastTargetVelo = targetVelo;

            // Get the velocity from the motor with the encoder
            double motorPos = robot.shooter1.getCurrentPosition();
            double motorVelo = robot.shooter1.getVelocity();

            // Update the controller and set the power for each motor
            double power = veloController.update(motorPos, motorVelo);
            robot.shooter1.setPower(power);
            robot.shooter2.setPower(power);


            if (gamepad2.dpad_up){
                robot.transferServo.setPosition(0);
            }
            else if (gamepad2.dpad_down){
                robot.transferServo.setPosition(0.16);
            }

            //intake winch
            if (gamepad2.right_bumper){
                robot.intakeWinch.setPower(1);
            }
            else if (gamepad2.left_bumper){
                robot.intakeWinch.setPower(-1);
            }
            else {robot.intakeWinch.setPower(0);}

            robot.transfer.setPower(gamepad2.left_stick_y);

            //telemetry.addData("shooter velocity", shooter.getPower());
            telemetry.addData("shooter encoder velo", robot.shooter1.getVelocity());
            telemetry.update();

        }
    }
}