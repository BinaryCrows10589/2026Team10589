package binarycrows.robot;

import java.util.Map;

import binarycrows.robot.Enums.StateRequestGroupChildTimeoutBehavior;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.Enums.StateRequestStatus;

public class StateRequest<TYPE extends Enum<TYPE>> {

    public record LoggableStateRequest(StateRequestStatus status, StateRequestPriority priority) {}

    protected StateRequestStatus status = StateRequestStatus.FRESH;
    private TYPE stateRequestType;
    private StateRequestPriority priority;

    protected long requestTimeout;
    private long timeOfDeployment;

    protected StateRequestGroupChildTimeoutBehavior childTimeoutBehavior;

    /**
     * State requests are objects that are passed between state managers to request that a subsystem put itself into a particular state.
     * @param stateRequestType The type of state request that this is, which is derived from an enum provided by the targeted substate manager
     * @param priority A priority level of this state request to determine what it can and cannot override.
     * @param childTimeoutBehavior A behavior for the parent state request group (if any) to adopt if this request were to time out.
     */
    public StateRequest(TYPE stateRequestType, StateRequestPriority priority, StateRequestGroupChildTimeoutBehavior childTimeoutBehavior) {
        this.stateRequestType = stateRequestType;
        this.priority = priority;
        this.status = StateRequestStatus.FRESH;
        this.childTimeoutBehavior = childTimeoutBehavior;
    }


    /**
     * State requests are objects that are passed between state managers to request that a subsystem put itself into a particular state.
     * @param stateRequestType The type of state request that this is, which is derived from an enum provided by the targeted substate manager
     * @param priority A priority level of this state request to determine what it can and cannot override.
     */
    public StateRequest(TYPE stateRequestType, StateRequestPriority priority) {
        this(stateRequestType, priority, StateRequestGroupChildTimeoutBehavior.KILL);
    }

    /**
     * Reset this state request as if it was just constructed.
     */
    public void reinitialize() {
        this.status = StateRequestStatus.FRESH;
    }

    public TYPE getStateRequestType() {
        return this.stateRequestType;
    }

    public StateRequestGroupChildTimeoutBehavior getChildTimeoutBehavior() {
        return this.childTimeoutBehavior;
    }

    @SuppressWarnings("rawtypes")
    public SubStateManager getSubStateManager() {
        return MainStateManager.getInstance().resolveSubStateManager(stateRequestType.getClass());
    }

    public void updateStatus(StateRequestStatus status) {
        this.status = status;
    }

    public StateRequestStatus getStatus() {
        return this.status;
    }

    public StateRequestPriority getPriority() {
        return this.priority;
    }


    public LoggableStateRequest getAsLoggable() {
        return new LoggableStateRequest(status, priority);
    }

    public void dispatchSelf() {
        MainStateManager.getInstance().dispatchStateRequest(this);
    }


    public void setTimeOfDeployment() {
        this.timeOfDeployment = System.currentTimeMillis();
    }

    public boolean getIsTimedOut() {
        return System.currentTimeMillis() - this.timeOfDeployment >= requestTimeout;
    }

}
