package org.firstinspires.ftc.teamcode.TestingAndSamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Archive.Robot;

@Autonomous
public class wrapimutest extends Robot {
    public void op_mode(){
        waitForStart();
        wrapIMU(180, -0.5, 0.5, 1, 1.5, 0, 0);
    }
}
