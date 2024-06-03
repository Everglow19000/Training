package org.EverglowLibrary.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class GWheelSystem{
    private final DcMotorEx motor;
    double currentPower = 0;
    OpMode opMode;
    public GWheelSystem(OpMode opMode){
        this.opMode = opMode;
        motor = opMode.hardwareMap.get(DcMotorEx .class, "GagazMot");
    }

    public void setPower(double Power) {
        motor.setPower(Power);
    }

    public void toggle(boolean inverted) {
        if(currentPower != 0) currentPower = 0;
        else if(inverted) currentPower = -1;
        else currentPower = 1;

        setPower(currentPower);
    }

    public double getCurrentPower(){
        return currentPower;
    }

    public Executor getExecutor(boolean isReverse){
        return new GWheelExecutor(isReverse);
    }

    public class GWheelExecutor extends Executor{

        private final boolean isInverted;

        public GWheelExecutor(boolean isInverted){
            this.isInverted = isInverted;
        }


        @Override
        public void run() {
            toggle(isInverted);
        }

        @Override
        public boolean isFinished() {
            return true; //there is no logic here
        }

        @Override
        public void stop() {
            setPower(0);
        }
    }

}