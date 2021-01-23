package org.firstinspires.ftc.teamcode;

//haha yes I totally found an excuse to use this name again :P -Christian

import org.firstinspires.ftc.teamcode.Archive.Robot;

/**
 * An abstract op mode structured around Future7573. Methods named weirdly cause
 * they conflicted with stuff from LinearOpMode
 */
public abstract class OpMode7573 extends Robot {
    public final FutureManager futures = new FutureManager();

    /** Called as soon as the OP mode is initialized */
    public abstract void init7573();
    /** Called when the OP mode is started */
    public abstract void start7573();
    /** Called continuously after start until the OP mode is stopped */
    public abstract void loop7573();
    /** Called when the OP mode has stopped, even if it has not been started */
    public abstract void end7573();

    @Override public void op_mode() {
        this.init7573();
        while (!isStarted()) {
            this.futures.poll();
            try {
                Thread.sleep(60);
            } catch (InterruptedException e) { }
        }
        if (opModeIsActive()) {
            this.start7573();
            while (opModeIsActive()) {
                this.loop7573();
                this.futures.poll();
            }
        }
        this.end7573();
    }

    /**
     * Waits for the OP mode to start
     */
    public class WaitForStart implements Future7573 {
        Future7573 next;
        /**
         * @param next - the future to transfer flow to when finished
         */
        public WaitForStart(Future7573 next) { this.next = next; }
        public Future7573 poll() { return isStarted() ? next : this; }
    }
}
