package lib.frc3597.looper;

import lib.frc3597.subsystem.SubsystemBase;
import lib.frc3597.subsystem.SubsystemHandler;

import java.util.List;

public class EnabledSuperSubsystemLooper implements ILoop {
    private final List<SubsystemBase> allSubsystems;

    public EnabledSuperSubsystemLooper(SubsystemHandler subsystemHandler) {
        this.allSubsystems = subsystemHandler.getSubsystems();
    }

    @Override
    public void onStart(double timestamp) {
        allSubsystems.forEach(s -> s.onStart(timestamp));
    }

    @Override
    public void onLoop(double timestamp) {
//        for (SubsystemBase allSubsystem : allSubsystems) {
//            allSubsystem.readPeriodicInputs();
//        }
        // lambda is equivalent to what is above lol

        allSubsystems.forEach(SubsystemBase::readPeriodicInputs);
        allSubsystems.forEach(s -> s.onLoop(timestamp));
        allSubsystems.forEach(SubsystemBase::writePeriodicOutputs);
        allSubsystems.forEach(SubsystemBase::outputData);
    }

    @Override
    public void onStop(double timestamp) {
        allSubsystems.forEach(s -> s.onStop(timestamp));
    }
}
