package lib.frc3597.state;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.HashMap;

public class StateMachine {
    private final HashMap<String, State> states;
    private final HashMap<String, ArrayList<Transition>> conditions;
    private String currentState;

    public StateMachine() {
        states = new HashMap<>();
        conditions = new HashMap<>();
        currentState = null;
    }

    public TransitionBuilder from(String currentState) {
        return new TransitionBuilder(this, currentState);
    }

    public StateMachine addState(State state) {
        states.put(state.getName(), state);

        return this;
    }

    void addCondition(String conditionName, Transition transition) {
        if (!conditions.containsKey(currentState)) {
            conditions.put(currentState, new ArrayList<>());
        }

        conditions.get(currentState).add(transition);
    }

    public StateMachine getFirstState(String stateName) {
        currentState = stateName;
        if (!states.containsKey(stateName)) {
            throw new IllegalArgumentException("State " + stateName + " not found");
        }

        return this;
    }

    public void start() {
        if (currentState == null) {
            throw new IllegalStateException("No current state");
        }

        CommandScheduler.getInstance().schedule(states.get(currentState));
    }

    public void update() {
        if (conditions.get(currentState) != null) {
            State stateExecutor = states.get(currentState);
            conditions.get(currentState).forEach(transition -> {
                if (transition.isFinished(stateExecutor.getCurrentCondition())) {
                    CommandScheduler.getInstance().cancel(states.get(currentState));

                    currentState = transition.getNextState();

                    if (!states.containsKey(currentState)) {
                        throw new IllegalArgumentException("State " + currentState + " not found");
                    }

                    CommandScheduler.getInstance().schedule(states.get(currentState));
                }
            });
        }
    }
}
