package binarycrows.robot.StateRequestGroup;

import binarycrows.robot.StateRequest;
import binarycrows.robot.Enums.NullStateRequestType;
import binarycrows.robot.Enums.StateRequestPriority;

@SuppressWarnings("rawtypes")
public abstract class StateRequestGroup extends StateRequest<NullStateRequestType> {

    protected StateRequest[] children;
    @SuppressWarnings("unused")
    private StateRequest faultyStateRequest; // Records which state request caused this group to fail if it ever fails.

    public StateRequestGroup(StateRequestPriority priority, long requestTimeout, StateRequest... children) {
        super(null, priority);
        this.children = children;
        this.requestTimeout = requestTimeout;
    }

    public int getLength() { return children.length; }

    public void setFaultyStateRequest(StateRequest faultyStateRequest) {
        this.faultyStateRequest = faultyStateRequest;
    }


    

}
