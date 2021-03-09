package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.time.Instant;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;

/**
 * Represents an ongoing task that will complete in the future
 * @see Future7573#poll()
 */
public interface Future7573 {
    /**
     * Does the work of the future, advancing it's state. This method returns it's
     * "updated state", AKA the next piece of work to be done. If the future is still doing
     * work, it will return itself. If it has finished, it can return another future and pass
     * it the flow, or it can return null to indicate this branch has no more work to be done.
     * @return the "updated state" of this future
     */
    public Future7573 poll();

    /**
     * Holds the current flow for a number of milliseconds
     */
    public static class Sleep implements Future7573 {
        Instant target_time;
        Future7573 next;

        /**
         * @param milliseconds - number of milliseconds to wait
         * @param next - the future to transfer flow to once complete
         */
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

    /**
     * Waits for a motor to become non-busy
     */
    public static class WaitForMotor implements Future7573 {
        DcMotorEx motor;
        Future7573 next;

        /**
         * @param motor - the motor to wait for
         * @param next - the future to transfer flow to once complete
         */
        public WaitForMotor(DcMotorEx motor, Future7573 next) {
            this.motor = motor;
            this.next = next;
        }
        public Future7573 poll() {
            if (this.motor.isBusy()) return this;
            else return next;
        }
    }

    /**
     * An indicator for chain futures to advance to the next future
     * in the chain. Do not call poll().
     */
    public static class Next implements Future7573 {
        public Future7573 poll() { throw new RuntimeException("poll() called on Next"); }
    }

    /**
     * A chain of futures to be executed consecutively. Advances are made when a future returns Future7573.Next.
     * Terminates when any future in the chain terminates without returning Next.
     */
    public static class FutureChain implements Future7573 {
        ArrayList<Future7573> chain = new ArrayList<>();

        /**
         * @param chain - the chain of futures.
         * @see FutureChain
         */
        public FutureChain(List<Future7573> chain) {
            if (chain.size() < 1)
                throw new IllegalArgumentException("Cannot create chain future with length of 0");
            this.chain.addAll(chain);
        }

        public Future7573 poll() {
            Future7573 result = this.chain.get(0).poll();
            if (result == null) {
                return null;
            } else if (result instanceof Next) {
                this.chain.remove(0);
                if (this.chain.size() < 1) return null;
                else return this;
            } else {
                this.chain.set(0, result);
                return this;
            }
        }
    }
}