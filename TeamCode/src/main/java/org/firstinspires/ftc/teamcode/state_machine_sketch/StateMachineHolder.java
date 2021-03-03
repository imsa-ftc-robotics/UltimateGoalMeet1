package org.firstinspires.ftc.teamcode.state_machine_sketch;

public class StateMachineHolder {
    StateMachineIsh state_machine;
    public StateMachineHolder(StateMachineIsh state_machine) { this.state_machine = state_machine; }
    public void progress() {
        this.state_machine = this.state_machine.progress();
    }
}
