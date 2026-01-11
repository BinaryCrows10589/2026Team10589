package binarycrows.robot.StateRequestGroup;

import binarycrows.robot.StateRequest;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.Enums.StateRequestStatus;

@SuppressWarnings("rawtypes")
public class SequentialGroup extends StateRequestGroup {

    public int currentStateRequestIndex = 0;

    public SequentialGroup(StateRequestPriority priority, long timeout, StateRequest... children) {
        super(priority, timeout, children);
    }

    public StateRequest getCurrentStateRequest() { return children[currentStateRequestIndex]; }

    public void increment() {
        currentStateRequestIndex++;
        if (currentStateRequestIndex >= getLength()) {
            updateStatus(StateRequestStatus.FULFILLED);
        }
    }
}
