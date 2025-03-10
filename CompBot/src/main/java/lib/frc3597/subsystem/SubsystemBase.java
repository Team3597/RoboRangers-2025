package lib.frc3597.subsystem;

import lib.frc3597.looper.ILoop;

import java.util.UUID;

public abstract class SubsystemBase<T> extends SubsystemManagerBase implements ILoop {
    public String name = "Unnamed";
    UUID id = null;

    protected T subsystemManager;

    public abstract void readPeriodicInputs();
    public abstract void writePeriodicOutputs();
    public abstract void zeroSensors();

    public abstract void terminate();
    public abstract void outputData();

    public T getSubsystemManager() {
        return subsystemManager;
    }
}
