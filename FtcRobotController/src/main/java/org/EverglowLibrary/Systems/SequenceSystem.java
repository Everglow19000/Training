package org.EverglowLibrary.Systems;

import org.EverglowLibrary.ExecuteMotor.Sequence;

import java.nio.channels.AsynchronousCloseException;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class SequenceSystem {
    private Queue<Sequence> m_Runs = new LinkedList<>();

    public SequenceSystem(Sequence... runs){
        m_Runs.addAll(Arrays.asList(runs));
    }

    public void ExecuteAll(){
        for (Sequence run : m_Runs) {
            Execute(run);
        }
    }

    public static void Execute(Sequence sequence){
        try {
            sequence.startSequence();
        }
        catch (Exception e){

        }
    }
}
