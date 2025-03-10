package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoModeSelector {
    public enum DesiredMode {
        THREE_LVL_4,
        TWO_LVL_4_ALGAE,
        DO_NOTHING
    }

    private final RobotContainer m_robotContainer;

    public AutoModeSelector(RobotContainer robotContainer) {
        this.m_robotContainer = robotContainer;
    }

    public Command getAutoCommand() {
        return Commands.none();
    }
}
