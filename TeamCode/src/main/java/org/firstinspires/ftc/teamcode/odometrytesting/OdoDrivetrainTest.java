package org.firstinspires.ftc.teamcode.odometrytesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class OdoDrivetrainTest extends LinearOpMode {
    private DcMotorEx lf, rf, rb, lb;

    public void runOpMode(){
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lb = hardwareMap.get(DcMotorEx.class, "lb");

        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.FORWARD);
        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            rb.setPower(gamepad1.right_stick_y);
            rf.setPower(gamepad1.right_trigger);
            lf.setPower(gamepad1.left_trigger);
            lb.setPower(gamepad1.left_stick_y);
        }
    }
}
