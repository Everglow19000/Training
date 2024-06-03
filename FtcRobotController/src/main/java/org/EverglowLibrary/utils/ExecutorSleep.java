package org.EverglowLibrary.utils;

import org.EverglowLibrary.Systems.Executor;

public class ExecutorSleep extends Executor {
    public long time;

    public ExecutorSleep(long time){
        this.time = time;
    }
    @Override
    public void run() {
        try {
            Thread.sleep(time);
        }catch (Exception e){
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void stop() {

    }
}
