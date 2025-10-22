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
        return resolvePriorityValue(subject.getPriority()) < resolvePriorityValue(subject.getPriority());
    }

    public static int resolvePriorityValue(StateRequestPriority priority) {
        switch (priority) {
            case DO_NOT_IGNORE:
                return 0;
            case HIGH:
                return 1;
            case NORMAL:
                return 2;
            case LOW:
                return 3;
            case ALWAYS_IGNORE:
                return 4;
            default:
                throw new InvalidParameterException("No numerical value found for given priority.");
        }
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

    public static Runnable createStateRequestRunnable(@SuppressWarnings("rawtypes") Enum type, StateRequestPriority priority) {
        return () -> {
            MainStateManager.getInstance().dispatchStateRequest(
                new StateRequest<>(type, priority));
        };
    }
    public static Runnable createStateRequestRunnable(@SuppressWarnings("rawtypes") Enum type) {
        return createStateRequestRunnable(type, StateRequestPriority.NORMAL);
    }
}
