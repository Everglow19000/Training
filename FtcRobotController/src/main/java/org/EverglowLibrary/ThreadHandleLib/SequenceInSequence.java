package org.EverglowLibrary.ThreadHandleLib;

import org.EverglowLibrary.Systems.Executor;

import java.nio.channels.AsynchronousCloseException;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.Queue;

public class SequenceInSequence{
    private Queue<Sequence> sequenceQueue = new LinkedList<>();
    private Thread m_Thread;
    private boolean isAsync;
    private Thread[] threads;

    public SequenceInSequence(boolean isAsync, Sequence... sequences){
        sequenceQueue.addAll(Arrays.asList(sequences));
        this.isAsync = isAsync;
        if(isAsync)
        {

        }
        else {
            m_Thread = new Thread(() -> {
                Sequence[] sequences1 = sequenceQueue.toArray(new Sequence[sequenceQueue.size()]);
                for (int i = 0; i < sequences1.length; i++) {
                    sequences1[i].startSequence();
                    while (!sequences1[i].isDone()) {

                    }
                }
            });
        }
    }

    public Sequence[] GetAllSequences(){
        return sequenceQueue.toArray(sequenceQueue.toArray(new Sequence[sequenceQueue.size()]));
    }

    public void add(Sequence sequence){
        sequenceQueue.add(sequence);
    }

    public void RunAll() {
        if(isAsync){
            threads = new Thread[sequenceQueue.size()];
            Sequence[] sequences1 = sequenceQueue.toArray(new Sequence[sequenceQueue.size()]);
            for (int i = 0; i < sequenceQueue.size(); i++) {
                threads[i] = new Thread(sequences1[i] :: startSequence);
                threads[i].start();
            }
        }
        else if(!m_Thread.isAlive()){
            m_Thread.start();
        }
    }

    public void stopAll(){
        for (Sequence seq:
             sequenceQueue) {
            seq.interruptSequence();
        }
    }
}
