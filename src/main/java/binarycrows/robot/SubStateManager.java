package binarycrows.robot;

import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.Utils.StateRequestUtils;

public class SubStateManager<TYPE extends Enum<TYPE>> {

    /**
     * The active state request for the sub-state manager. 
     * Note that this should be initialized to a default state by any extending classes.
     */
    protected StateRequest<TYPE> activeStateRequest;
    protected StateRequest<TYPE> defaultState;

    protected SubStateManager(StateRequest<TYPE> defaultState)
    {
        this.activeStateRequest = defaultState;
        this.defaultState = defaultState;
    }


    public void periodic() {
        if (activeStateRequest == null) activeStateRequest = defaultState;
    }

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

    @SuppressWarnings("unchecked")
    public Class<TYPE> getStateRequestType() {
        return (Class<TYPE>) this.defaultState.getStateRequestType().getClass();
    }

    @Override
    public String toString() {
        return "Unnamed SubState manager";
    }
}
