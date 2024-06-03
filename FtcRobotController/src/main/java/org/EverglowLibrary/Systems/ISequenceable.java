package org.EverglowLibrary.Systems;

public interface ISequenceable extends Runnable{
    boolean isFinished();
    void stop();
}
