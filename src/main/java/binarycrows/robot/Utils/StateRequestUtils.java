package binarycrows.robot.Utils;

import java.security.InvalidParameterException;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.Enums.StateRequestPriority;

public class StateRequestUtils {
    /**
     * Compares two state request priorities and returns which is higher.
     * @param subject
     * @param incoming
     * @return A boolean that will be "true" if the subject is higher priority, or "false" if the incoming is higher priority or the priority is the same.
     */
    @SuppressWarnings("rawtypes")
    public static boolean isPriorityHigher(StateRequest subject, StateRequest incoming) {
        return subject.getPriority() < incoming.getPriority();
    }


    @SuppressWarnings("rawtypes")
    public static boolean isStateRequestStillActive(StateRequest subject) {
        switch (subject.getStatus()) {
            case PENDING:
                return true;
            case FRESH:
                return true;
            case CANCELLED:
                return false;
            case RUNNING:
                return true;
            case OVERRIDDEN:
                return false;
            case REJECTED:
                return false;
            case FULFILLED:
                return false;
            default:
                throw new InvalidParameterException("No activity value found for given status."); 
        }
    }

    public static Runnable createStateRequestRunnable(@SuppressWarnings("rawtypes") Enum type, int priority) {
        return () -> {
            MainStateManager.getInstance().dispatchStateRequest(
                new StateRequest<>(type, priority));
        };
    }
    public static Runnable createStateRequestRunnable(@SuppressWarnings("rawtypes") Enum type) {
        return createStateRequestRunnable(type, StateRequestPriority.NORMAL);
    }

    public static Runnable createDualStateRequestRunnable(@SuppressWarnings("rawtypes") Enum typeA, @SuppressWarnings("rawtypes") Enum typeB, int priorityA, int priorityB) {
        return () -> {
            MainStateManager.getInstance().dispatchStateRequest(
                new StateRequest<>(typeA, priorityA));
            MainStateManager.getInstance().dispatchStateRequest(
                new StateRequest<>(typeB, priorityB));
        };
    }

    public static Runnable createDualStateRequestRunnable(@SuppressWarnings("rawtypes") Enum typeA, @SuppressWarnings("rawtypes") Enum typeB) {
        return createDualStateRequestRunnable(typeA, typeB, StateRequestPriority.NORMAL, StateRequestPriority.NORMAL);
    }
}
