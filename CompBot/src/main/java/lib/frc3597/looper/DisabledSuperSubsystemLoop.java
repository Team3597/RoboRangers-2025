package lib.frc3597.looper;

import lib.frc3597.subsystem.SubsystemBase;
import lib.frc3597.subsystem.SubsystemHandler;

import java.util.List;

public class DisabledSuperSubsystemLoop implements ILoop {
    private final List<SubsystemBase> allSubsystems;

    public DisabledSuperSubsystemLoop(SubsystemHandler mSubsystemManger){
        this.allSubsystems = mSubsystemManger.getSubsystems();
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void onLoop(double timestamp) {
        allSubsystems.forEach(SubsystemBase::readPeriodicInputs);

        try {
            allSubsystems.forEach(SubsystemBase::outputData);
        } catch (Exception e) {
            System.out.println("Something is not initialized in disabled in outputData");
            e.printStackTrace();
        }
    }

    @Override
    public void onStop(double timestamp) {
    }
}
