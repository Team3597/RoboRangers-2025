package lib.frc3597.state;

public class TransitionBuilder<T> {
    private final StateMachine stateMachine;
    private final String currentState;

    private String nextState;

    public TransitionBuilder(StateMachine stateMachine, String currentState) {
        this.stateMachine = stateMachine;
        this.currentState = currentState;
    }

    // called on TransitionBuilder object returned by to("nextState")
    // returns stateMachine object
    public StateMachine when(T condition) {
        // checks that TransitionBuilder object has nextState set
        if (nextState == null) {
            throw new IllegalStateException("Condition must be set");
        }

        // calls addCondition from StateMachine class, passing back the state from StateMachine 
        // from("currentState") method and creating an object of Transition with nextState from
        // TransitionBuilder to("nextState") and condition from this method
        stateMachine.addCondition(currentState, new Transition<T>(condition, nextState));

        return stateMachine;
    }

    // called on TransitionBuilder object returned by StateMachine from("currentState") method
    // returns this instance of TransitionBuilder for when to be called on 
    public TransitionBuilder to(String nextState) {
        this.nextState = nextState;

        return this;
    }
}
