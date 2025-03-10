package lib.frc3597.state;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class State<T> extends Command {
    private final String name;

    public State(String name) {
        this.name = name;
    }

    public abstract T getCurrentCondition();

    public String getName() {
        return name;
    }
}
