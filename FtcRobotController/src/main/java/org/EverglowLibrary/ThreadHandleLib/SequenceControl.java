package org.EverglowLibrary.ThreadHandleLib;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.Executor;
import org.EverglowLibrary.Systems.FourBarSystem;


public class SequenceControl {

//    private boolean seq1_toggle = false;
//    private boolean seq2_toggle = false;
//    private boolean seq3_toggle = false;
//    private boolean seq4_toggle = false;
//    private boolean upAndDown_toggle = false;

    private final Sequence getReadyToDropSeq;
    private final Sequence setUpAndUnderBlockSeq;
    private final Sequence dropAndRetreatSeq;
    private final Sequence getUpAndReadyToDrop;
    private final Sequence DropMiddle;

    public SequenceControl(ClawSystem clawSystem, FourBarSystem fourBarSystem
            , ElevatorSystem elevatorSystem){
        getReadyToDropSeq = new Sequence(false, clawSystem.getExecutor(false)
                ,elevatorSystem.getExecutor(ElevatorSystem.Level.UP)
                ,fourBarSystem.getExecutor(FourBarSystem.ServoAngel.DROP, true)
                , fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP));

        setUpAndUnderBlockSeq = new Sequence(false, clawSystem.getExecutor(false),
                elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.ServoAngel.DROP, true),
                fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN));

        dropAndRetreatSeq = new Sequence(false, clawSystem.getExecutor(true)
                ,elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.PICKUP, FourBarSystem.ServoAngel.DROP)
                        , elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN)
                , fourBarSystem.getExecutor(FourBarSystem.Level.PICKUP, FourBarSystem.ServoAngel.PICKUP));

        getUpAndReadyToDrop = new Sequence(true, elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP));

        DropMiddle = new Sequence(false, clawSystem.getExecutor(false),
                elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.ServoAngel.DROP, true),
                fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.MED));
    }

    public Sequence GetReadyToDropSeq(){
        return getReadyToDropSeq;
    }

    public Sequence SetUpAndUnderBlockSeq(){
        return setUpAndUnderBlockSeq;
    }

    public Sequence DropAndRetreatSeq(){
        return dropAndRetreatSeq;
    }

    public Sequence GetUpAndReadyToDrop(){
        return getUpAndReadyToDrop;
    }

    public Sequence GetMiddleDrop() {return DropMiddle;}
}
