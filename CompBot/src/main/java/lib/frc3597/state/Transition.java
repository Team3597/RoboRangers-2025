package lib.frc3597.state;

public class Transition<T> {
    private final T endCondition;
    private final String nextState;

    public Transition(T endCondition, String nextState) {
        this.endCondition = endCondition;
        this.nextState = nextState;
    }

    public boolean isFinished(T current) {
        return endCondition == current;
    }

    public String getNextState() {
        return nextState;
    }
}
