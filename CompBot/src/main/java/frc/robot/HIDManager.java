package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import lib.frc3597.hid.HIDLayout;

public class HIDManager {
    private XboxController driver;
    private XboxController gunner;

    private static HIDManager instance;

    private HIDLayout layout;

    private HIDManager() {
        this.driver = new XboxController(0);
        this.gunner = new XboxController(1);
    }

    public static HIDManager getInstance() {
        if (instance == null) {
            instance = new HIDManager();
        }
        return instance;
    }
}
