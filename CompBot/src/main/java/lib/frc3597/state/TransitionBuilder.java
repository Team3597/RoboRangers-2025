package lib.frc3597.state;

public class TransitionBuilder<T> {
    private final StateMachine stateMachine;
    private final String currentState;

    private String nextState;

    public TransitionBuilder(StateMachine stateMachine, String currentState) {
        this.stateMachine = stateMachine;
        this.currentState = currentState;
    }

    public StateMachine when(T condition) {
        if (nextState == null) {
            throw new IllegalStateException("Condition must be set");
        }

        stateMachine.addCondition(currentState, new Transition<T>(condition, nextState));

        return stateMachine;
    }

    public TransitionBuilder to(String nextState) {
        this.nextState = nextState;

        return this;
    }
}
