package org.EverglowLibrary.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Calendar;


public class ClawSystem{
    private boolean open = false;
    private final double leftClosed = 0.7, leftOpen = 0.25, rightClosed = 0.4, rightOpen = 0.8;
    private final LinearOpMode opMode;

    Servo ClawR, ClawL;
    public ClawSystem(LinearOpMode opMode){
        this.opMode = opMode;
        ClawR = opMode.hardwareMap.get(Servo .class, "ClawR");
        ClawL = opMode.hardwareMap.get(Servo.class, "ClawL");
        toggle();
    }

    public void ChangePos(boolean toOpen){
        if(toOpen){
            ClawR.setPosition(rightOpen); //open
            ClawL.setPosition(leftOpen);
        }
        else{
            ClawR.setPosition(rightClosed); //closed
            ClawL.setPosition(leftClosed);
        }
        open = toOpen;
    }

    public void toggle() {
        ChangePos(!open);
    }

    public void MoveOneClaw(boolean isLeft, boolean toOpen){
        if(toOpen){
            if (isLeft){
                ClawL.setPosition(leftOpen);
            }
            else {
                ClawR.setPosition(rightOpen);
            }
        }
        else{
            if (isLeft){
                ClawL.setPosition(leftClosed);
            }
            else{
                ClawR.setPosition(rightClosed);
            }
        }

        if(ClawR.getPosition() == rightClosed && ClawL.getPosition() == leftClosed){
            open = false;
        }
        else if(ClawR.getPosition() == rightOpen && ClawL.getPosition() == leftOpen){
            open = true;
        }
    }

    public void MoveOneClaw(boolean isLeft){
        if(isLeft){
            if(ClawL.getPosition() == leftOpen){
                MoveOneClaw(isLeft,false);
            }
            else
            {
                MoveOneClaw(isLeft,true);
            }
        }
        else {
            if(ClawR.getPosition() == rightOpen){
                MoveOneClaw(isLeft,false);
            }
            else
            {
                MoveOneClaw(isLeft,true);
            }
        }
    }
    public Executor getExecutor(boolean toOpen){
        return new ClawExecutor(toOpen);
    }

    public class ClawExecutor extends Executor{
        private final boolean m_toOpen;
        private long startTime = -1;
        private boolean isWasOpenBefore = false;

        public ClawExecutor(boolean toClose){
            this.m_toOpen = toClose;
        }

        @Override
        public void run() {
            isWasOpenBefore = open;
            ChangePos(m_toOpen);
            startTime = Calendar.getInstance().getTimeInMillis();
        }

        @Override
        public boolean isFinished() {
            if(startTime == -1)
                opMode.sleep(10);

            //in milliseconds
            int waitTime = 400;

            if(Calendar.getInstance().getTimeInMillis() - startTime >= waitTime || isWasOpenBefore == m_toOpen) {
                startTime = -1;
                return true;
            }
            else
                return false;
        }

        @Override
        public void stop() {

        }
    }
}