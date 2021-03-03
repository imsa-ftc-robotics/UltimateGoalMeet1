package org.firstinspires.ftc.teamcode.state_machine_sketch;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RegionalsBot;

//Shared State Here
public abstract class AutonomousDrive implements StateMachineIsh {
    LinearOpMode op;
    RegionalsBot bot;

    protected AutonomousDrive(AutonomousDrive old) {
        this.op = old.op;
        this.bot = old.bot;
    }
    protected AutonomousDrive(LinearOpMode op, RegionalsBot bot) {
        this.op = op;
        this.bot = bot;
    }

    public static StateMachineHolder initialize(LinearOpMode op, RegionalsBot bot) {
        return new StateMachineHolder(new WaitForStart(op, bot));
    }
}

class WaitForStart extends AutonomousDrive {
    protected WaitForStart(LinearOpMode op, RegionalsBot bot)  { super(op, bot); }
    public StateMachineIsh progress() {
        if (this.op.isStarted()) {
            //Autonomous just started
            UGContourRingPipeline.Height rings = this.bot.pipeline.getHeight();
            AutonomousDrive next = null;
            switch (rings) {
                case ZERO: next = new ZeroRingsPathOne(this); break;
                case ONE: next = new OneRingsPathOne(this); break;
                case FOUR: next = new FourRingsPathOne(this); break;
            }
            //Pick up wobble goal, and sleep for robble goal to complete
            return new SleepState(1000, next);
        } else {
            return this;
        }
    }
}