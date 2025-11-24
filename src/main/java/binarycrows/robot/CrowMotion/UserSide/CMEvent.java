package binarycrows.robot.CrowMotion.UserSide;

public class CMEvent {

    private String eventName;
    private Runnable eventFunction;
    private double eventTriggerPercent;
    private boolean hasBeenTriggered = false;

    /**
     * Constructs a new {@code CMEvent} with the specified event name, event
     * function, and trigger percentage along the path.
     *
     * @param eventName
     * The name of the event, primarily used for logging and debugging.
     * <ul>
     *   <li>Should be descriptive (e.g., "IntakeDown", "ShootHigh").</li>
     *   <li>No strict bounds, but unique names improve clarity.</li>
     * </ul>
     *
     * @param eventFunction
     * The function (Runnable) to be executed when the event is triggered.
     * <ul>
     *   <li>Should contain non-blocking, lightweight logic to avoid slowing the trajectory loop.</li>
     *   <li>Examples: scheduling a command, logging a marker</li>
     * </ul>
     *
     * @param eventTriggerPercent
     * The percentage along the trajectory path at which the event will be
     * triggered.
     * <ul>
     *   <li>Lower bound: 0.0 (immediately at start of path).</li>
     *   <li>Upper bound: 1.0 (at the very end of the path).</li>
     *   <li>Suggested values: choose based on when along the trajectory the
     *   action should occur (e.g., 0.25 = quarter of the way, 0.5 = halfway).</li>
     * </ul>
     * <b>Note:</b> Values outside the range [0.0, 1.0] are invalid.
     */

    public CMEvent(String eventName, Runnable eventFunction, double eventTriggerPercent) {
        this.eventName = eventName;
        this.eventFunction = eventFunction;
        this.eventTriggerPercent = eventTriggerPercent;

        if(eventTriggerPercent < 0 || eventTriggerPercent > 1) {
            throw new ExceptionInInitializerError(
                "CrowMotion Event's eventTriggerPercent must be between 0 and 1(inclusive), not: " + eventTriggerPercent
            );
        }
    }

    /**
     * Gets the name of the event.
     *
     * @return The event name.
     */
    public String getEventName() {
        return eventName;
    }

    /**
     * Gets the function that will be executed when the event is triggered.
     *
     * @return The event function (Runnable).
     */
    public Runnable getEventFunction() {
        return eventFunction;
    }

    /**
     * Gets the percent along a path at which the event will be triggered. 
     *
     * @return The percent along a path at which the event will be triggered. 
     */
    public double getEventTriggerPercent() {
        return eventTriggerPercent;
    }

    public boolean setHasBeenTriggered(boolean triggered) {
        return this.hasBeenTriggered = triggered;
    }

    public boolean getHasBeenTriggered() {
        return this.hasBeenTriggered;
    }

}