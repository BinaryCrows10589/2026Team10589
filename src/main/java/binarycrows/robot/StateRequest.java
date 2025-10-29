package binarycrows.robot;

import java.util.Map;

import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.Enums.StateRequestStatus;

public class StateRequest<TYPE extends Enum<TYPE>> {

    public record LoggableStateRequest(StateRequestStatus status, StateRequestPriority priority, boolean isLongRunning) {}

    protected StateRequestStatus status = StateRequestStatus.FRESH;
    private TYPE stateRequestType;
    private StateRequestPriority priority;
    private boolean isLongRunning;
    

    /**
     * State requests are objects that are passed between state managers to request that a subsystem put itself into a particular state.
     * @param stateRequestType The type of state request that this is, which is derived from an enum provided by the targeted substate manager
     * @param priority A priority level of this state request to determine what it can and cannot override.
     */
    public StateRequest(TYPE stateRequestType, StateRequestPriority priority, boolean isLongRunning) {
        this.stateRequestType = stateRequestType;
        this.priority = priority;
        this.isLongRunning = isLongRunning;
        this.status = StateRequestStatus.FRESH;
    }

    public StateRequest(TYPE stateRequestType, StateRequestPriority priority) {
        this(stateRequestType, priority, false);
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

    public boolean getIsLongRunning() {
        return this.isLongRunning;
    }

    public LoggableStateRequest getAsLoggable() {
        return new LoggableStateRequest(status, priority, isLongRunning);
    }

}
