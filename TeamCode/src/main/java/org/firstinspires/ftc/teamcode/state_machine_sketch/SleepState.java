package org.firstinspires.ftc.teamcode.state_machine_sketch;

public class SleepState implements StateMachineIsh {
    long goal;
    StateMachineIsh next;
    public SleepState(long milliseconds, StateMachineIsh next) {
        this.goal = System.currentTimeMillis() + milliseconds;
        this.next = next;
    }
    public StateMachineIsh progress() {
        if (System.currentTimeMillis() > this.goal) return this.next;
        else return this;
    }
}
