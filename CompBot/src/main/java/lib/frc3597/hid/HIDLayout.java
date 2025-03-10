package lib.frc3597.hid;

import edu.wpi.first.wpilibj.XboxController;

public abstract class HIDLayout {
    protected XboxController driver;
    protected XboxController gunner;

    public HIDLayout(XboxController driver, XboxController gunner) {
        this.driver = driver;
        this.gunner = gunner;
    }

    public abstract void getSubsystemManagers();
}