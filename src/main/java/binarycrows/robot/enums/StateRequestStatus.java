package binarycrows.robot.enums;

/**
 * Describes different states that a state request can have.
 */
public enum StateRequestStatus {
    /**
     * A fresh state request is one that has just been created or dispatched, and it hasn't been touched by any state managers yet
     */
    FRESH,
    /**
     * A pending state request is in the process of being fulfilled. It cannot be cancelled if it is a simple state request, but it may be overridden.
     */
    PENDING,
    /**
     * A fulfilled state request has been completed by its relevant state managers.
     */
    FULFILLED,
    /**
     * An overridden state request has been stopped because a more important or newer state request has taken its place.
     */
    OVERRIDDEN,
    /**
     * A cancelled state request has been stopped for one reason or another. This state is only valid for complex state requests, which can be reasonably undone.
     */
    CANCELLED
}
