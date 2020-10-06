package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.time.Instant;
import java.time.Duration;

public interface Future7573 {
    public Future7573 poll();

    public static class Sleep implements Future7573 {
        Instant target_time;
        Future7573 next;
        public Sleep(long milliseconds, Future7573 next) {
            this.target_time = Instant.now().plus(Duration.ofMillis(milliseconds));
            this.next = next;
        }
        public Future7573 poll() {
            Instant now = Instant.now();
            if (now.isAfter(target_time)) return this.next;
            else return this;
        }
    }

    public static class WaitForMotor implements Future7573 {
        DcMotorEx motor;
        Future7573 next;
        public WaitForMotor(DcMotorEx motor, Future7573 next) {
            this.motor = motor;
            this.next = next;
        }
        public Future7573 poll() {
            if (this.motor.isBusy()) return this;
            else return next;
        }
    }
}