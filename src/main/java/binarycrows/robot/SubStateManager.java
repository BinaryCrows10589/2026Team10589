package binarycrows.robot;

import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.Utils.StateRequestUtils;

public class SubStateManager<TYPE extends Enum<TYPE>> {

    @SuppressWarnings("rawtypes")
    public static SubStateManager instance = null;

    /**
     * The active state request for the sub-state manager. 
     * Note that this should be initialized to a default state by any extending classes.
     */
    protected StateRequest<TYPE> activeStateRequest;
    protected StateRequest<TYPE> defaultState;

    protected SubStateManager()
    {
        assert instance == null;
    }


    @SuppressWarnings("unused")
    public void periodic() {}

    /**
     * Accepts a state request and resolves whether it should replace the current state request.
     * @param request
     */
    public void recieveStateRequest(StateRequest<TYPE> request) {
        // If the current state request is still running and has a higher priority than the incoming state request...
        if (
            StateRequestUtils.isStateRequestStillActive(activeStateRequest) && 
            StateRequestUtils.isPriorityHigher(this.activeStateRequest, request)) 
            {
                // ...reject the incoming state request.
                request.updateStatus(StateRequestStatus.REJECTED);
        } else {
            // Otherwise, override the old state request with the new one.
            this.activeStateRequest.updateStatus(StateRequestStatus.OVERRIDDEN);
            this.activeStateRequest = request;
            request.updateStatus(StateRequestStatus.PENDING);
        }
    }

    public void returnToDefaultState() {
        recieveStateRequest(defaultState);
    }

    public TYPE getStateRequestType() {
        return null;
    }

    @SuppressWarnings("rawtypes")
    public static synchronized SubStateManager getInstance()
    {
        if (instance == null) instance = new SubStateManager();
        return instance;
    }
}
