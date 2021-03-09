package org.firstinspires.ftc.teamcode.Archive;

import org.firstinspires.ftc.teamcode.Archive.Future7573;

import java.util.ArrayList;
import java.util.concurrent.Callable;

public class FutureManager {
    private ArrayList<Future7573> futures = new ArrayList<>();

    /**
     * Spawns a single future for execution. FutureManager.poll() must be called for
     * progress to happen.
     * @param future - the future
     */
    public void spawn(Future7573 future) { futures.add(future); }

    /**
     * Spawns a starting future and a chain of actions behind it. The chain is advanced whenever
     * a future advances to Future7573.Next. FutureManager.poll() must be called for progress
     * to happen.
     * @param initial - the initial future
     * @param chain - additional actions that return more futures to be executed in order
     */
    public void spawnChain(Future7573 initial, Callable<Future7573> ... chain) {
        ArrayList<Future7573> actual_chain = new ArrayList<>();
        actual_chain.add(initial);
        for (Callable<Future7573> callable : chain) {
            actual_chain.add(new Future7573() {
                public Future7573 poll() {
                    try { return callable.call(); }
                    catch (Exception e) { throw new RuntimeException(e); }
                }
            });
        }
        spawn(new Future7573.FutureChain(actual_chain));
    }

    /**
     * Polls all futures once, allowing them to do work
     */
    public void poll() {
        for (int i = 0; i < futures.size(); i++) {
            Future7573 result = futures.get(i).poll();
            if (result == null) {
                futures.remove(i);
                i--;
            } else {
                futures.set(i, result);
            }
        }
    }
}
