package lib.frc3597.state;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.HashMap;

public class StateMachine {
    //'map' of pairs of keys (string) and values (objects of State class)
    private final HashMap<String, State> states;
   
    private final HashMap<String, ArrayList<Transition>> conditions;
    private String currentState;

    public StateMachine() {
        states = new HashMap<>();
        conditions = new HashMap<>();
        currentState = null;
    }

    // add a transition from a given state
    // returns TransitionBuilder for this instance of state machine class and given state
    // to method can then be called on TransitionBuilder object returned from this method
    // use like StateMachine.from(currentState).to(nextState).when(condition) 
    public TransitionBuilder from(String currentState) {
        return new TransitionBuilder(this, currentState);
    }

    // add object of State class to states hash map
    // returns this instance of state machine class for further method calls
    // use like StateMachine.addState(new State("stateName"))
    public StateMachine addState(State state) {
        states.put(state.getName(), state);

        return this;
    }

    // called by TransitionBuilder in when(Condition) method, passes back the state from  
    // from("currentState") method and creates an object of Transition with nextState from
    // TransitionBuilder to("nextState") and condition from when(Condition) method
    void addCondition(String conditionName, Transition transition) {
        //ensure that conditions hash map has array of conditions for current state
        if (!conditions.containsKey(currentState)) {
            conditions.put(currentState, new ArrayList<>());
        }
        //add transition to array of conditions for current state
        conditions.get(currentState).add(transition); // Makili: seems like conditionName should be tied to the transition in some way...
    }

    // sets first state to passed "stateName"
    // returns this instance of StateMachine for chaining
    // use like StateMachine.getFirstState("StateName").start();
    public StateMachine getFirstState(String stateName) {
        currentState = stateName;
        // validates that passed state exists
        if (!states.containsKey(stateName)) {
            throw new IllegalArgumentException("State " + stateName + " not found");
        }

        return this;
    }

    // schedules first state as a command
    public void start() {
        // validates that first state is set with getFirstState
        if (currentState == null) {
            throw new IllegalStateException("No current state");
        }

        CommandScheduler.getInstance().schedule(states.get(currentState));
    }

    // call in robot periodic
    public void update() {
        // validate that currentState has been set by StateMachine.getFirstState("StateName").start();
        if (conditions.get(currentState) != null) { // Makili: huh? start() doesn't fill conditions for currentState...
            // create new state from currentState in states hash map
            State stateExecutor = states.get(currentState);
            // iterate through array of conditions for currentState
            // apply if statement on each transition object from conditions array using lambda
            conditions.get(currentState).forEach(transition -> {
                // if end condition of transition is met? kinda foggy how this actually works
                if (transition.isFinished(stateExecutor.getCurrentCondition())) {
                    // end current state
                    CommandScheduler.getInstance().cancel(states.get(currentState));
                    // set current state to nextState from transition
                    currentState = transition.getNextState();
                    // ensure that new currentState (was nextState) exists
                    if (!states.containsKey(currentState)) {
                        throw new IllegalArgumentException("State " + currentState + " not found");
                    }
                    // schedule new currentState
                    CommandScheduler.getInstance().schedule(states.get(currentState));
                }
            });
        }
    }
}
