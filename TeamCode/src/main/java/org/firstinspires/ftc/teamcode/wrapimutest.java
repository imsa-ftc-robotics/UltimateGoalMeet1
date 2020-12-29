package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class wrapimutest extends Robot {
    public void op_mode(){
        waitForStart();
        wrapIMU(180, -0.5, 0.5, 1, 1.5, 0, 0);
    }
}
