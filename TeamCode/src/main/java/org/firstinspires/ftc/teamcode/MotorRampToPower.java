package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.time.Instant;

public class MotorRampToPower implements Future7573 {
    public final double delta_power_per_ms;
    public final DcMotorEx motor;
    private boolean cancelled = false;
    private double target;
    private long last_poll = Instant.now().toEpochMilli();

    public MotorRampToPower(DcMotorEx motor, double delta_power_per_second, double initial_target) {
        this.motor = motor;
        this.delta_power_per_ms = delta_power_per_second / 1000;
        this.target = initial_target;
    }

    public boolean getCancelled() { return this.cancelled; }
    public void cancel() { this.cancelled = true; }

    public double getTarget() { return this.target; }
    public void setTarget(double target) { this.target = target; }

    public MotorRampToPower poll() {
        if (this.cancelled) return null;

        long now = Instant.now().toEpochMilli();
        if (motor.getPower() < this.target) {
            double new_motor_power = motor.getPower() + (this.delta_power_per_ms * (now - this.last_poll));
            if (new_motor_power > this.target) new_motor_power = this.target;
            motor.setPower(new_motor_power);
        } else {
            double new_motor_power = motor.getPower() - (this.delta_power_per_ms * (now - this.last_poll));
            if (new_motor_power < this.target) new_motor_power = this.target;
            motor.setPower(new_motor_power);
        }
        this.last_poll = now;

        return this;
    }
}
