package org.EverglowLibrary.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ElevatorSystem{

    public Executor getExecutor(Level level) {
        return new ElevatorExecutor(level);
    }


    public Executor getExecutor(Level level, double power) {
        return new ElevatorExecutor(level, power);
    }

    public enum Level {
        DOWN(0), UP(865), MED_AUTONOMOUS(100), MED(400), MAX_UP(1480), TLIA(750); //TLIA 918
        //Up 834

        public final int state;

        Level(int state) {
            this.state = state;
        }
    }

    public OpMode opMode;
    Level ElevatorCurrentLevel = Level.DOWN;
    /**
     * The motors which control the left and right sides of the elevator.
     */
    private final DcMotor left, right;


    public ElevatorSystem(OpMode opMode) {
        this.opMode = opMode;
        left = opMode.hardwareMap.get(DcMotorEx.class, "SlideL");
        right = opMode.hardwareMap.get(DcMotorEx.class, "SlideR");
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setDirection( DcMotorSimple.Direction.REVERSE);
        right.setTargetPosition(Level.DOWN.state);
        left.setTargetPosition(Level.DOWN.state);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Moves the elevator to the specified state.
     *
     * @param level The level to move the elevator to.
     */
    public void goTo(ElevatorSystem.Level level) {
        left.setTargetPosition(level.state);
        right.setTargetPosition(level.state);
    }

    public void goTo(int place){
        left.setTargetPosition(place);
        right.setTargetPosition(place);
    }

    public void setPower(double power){
        final double scale = 1;
        left.setPower(power * scale);
        right.setPower(power * scale);
    }

    public void setPower(double powerL, double powerR){
        final double scale = 1;
        left.setPower(powerL * scale);
        right.setPower(powerR * scale);
    }

    public void printPos(){
        opMode.telemetry.addData("right:", right.getCurrentPosition());
        opMode.telemetry.addData("left:", left.getCurrentPosition());
        opMode.telemetry.update();
    }

    public Level getCurrentPos(){
        return ElevatorCurrentLevel;
    }

    public void toggle(){
        final double downPower = -0.65;
        final double upPower = 0.65;
        if(this.ElevatorCurrentLevel == Level.DOWN) {
            ElevatorCurrentLevel = Level.UP;
            left.setPower(upPower);
            right.setPower(upPower);
        }
        else{
            ElevatorCurrentLevel = Level.DOWN;
            left.setPower(downPower);
            right.setPower(downPower);
        }

        goTo(ElevatorCurrentLevel);
    }

    public void toggleMax(){
        final double upPower = 0.9;
        if(ElevatorCurrentLevel != Level.MAX_UP)
        {
            ElevatorCurrentLevel = Level.MAX_UP;
            left.setPower(upPower);
            right.setPower(upPower);
        }
        else {
            ElevatorCurrentLevel = Level.TLIA;
            left.setPower(upPower);
            right.setPower(upPower);
        }
        goTo(ElevatorCurrentLevel);
    }


    public class ElevatorExecutor extends Executor{
        private Level toRun = null;
        private double Power = 0.75;

        public ElevatorExecutor(){
            //ElevatorCurrentLevel = Level.DOWN;
        }

        public ElevatorExecutor(Level level){
            toRun = level;
        }

        public ElevatorExecutor(Level level, double power){
            toRun = level;
            Power = power;
        }
        @Override
        public void run() {
            if(toRun == null) {
                toggle();
            }
            else{
                if(toRun == Level.UP) {
                    setPower(Power);
                    ElevatorCurrentLevel = Level.UP;
                }
                else {
                    setPower(-Power);
                    ElevatorCurrentLevel = Level.DOWN;
                }
                goTo(toRun);
            }
        }

        @Override
        public boolean isFinished() {
            final double epsilon = 15;
            boolean isFinished;

            isFinished =  left.getCurrentPosition() + epsilon >= toRun.state
                    && right.getCurrentPosition() + epsilon >= toRun.state
                    && left.getCurrentPosition() - epsilon <= toRun.state
                    && right.getCurrentPosition() - epsilon <= toRun.state;
//            if(ElevatorCurrentLevel == Level.UP)
//                return left.getCurrentPosition() + epsilon >= Level.UP.state
//                        && right.getCurrentPosition() + epsilon >= Level.UP.state;
//            else
//                return left.getCurrentPosition() - epsilon <= Level.DOWN.state
//                        && right.getCurrentPosition() - epsilon <= Level.DOWN.state;
            return isFinished;
        }

        @Override
        public void stop() {
            left.setPower(0);
            right.setPower(0);
        }
    }
}
