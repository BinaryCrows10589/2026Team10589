package binarycrows.robot.StateRequestGroup;

import binarycrows.robot.StateRequest;
import binarycrows.robot.Enums.NullStateRequestType;
import binarycrows.robot.Enums.StateRequestPriority;

@SuppressWarnings("rawtypes")
public abstract class StateRequestGroup extends StateRequest<NullStateRequestType> {

    @SuppressWarnings("unused")
    protected StateRequest[] children;
    protected long requestTimeout;

    public StateRequestGroup(StateRequestPriority priority, long requestTimeout, StateRequest... children) {
        super(null, priority, true);
        this.children = children;
        this.requestTimeout = requestTimeout;
    }

    public void execute() {}
}
