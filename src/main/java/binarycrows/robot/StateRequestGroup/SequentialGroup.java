package binarycrows.robot.StateRequestGroup;

import java.time.Instant;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.Enums.StateRequestStatus;

@SuppressWarnings("rawtypes")
public class SequentialGroup extends StateRequestGroup {

    public SequentialGroup(StateRequestPriority priority, long timeout, StateRequest... children) {
        super(priority, timeout, children);
    }

    public void execute() {
        this.updateStatus(StateRequestStatus.RUNNING);

        // Run through list of state requests sequentially
        for (StateRequest childStateRequest : this.children) {

            // Handle regular state request
            if (childStateRequest.getClass() == StateRequest.class) {

                long requestStartTime = Instant.now().getEpochSecond();

                MainStateManager.getInstance().dispatchStateRequest(childStateRequest);

                // Poll to block thread until done
                while (true) {
                    StateRequestStatus requestStatus = childStateRequest.getStatus();
                    if (requestStatus == StateRequestStatus.FULFILLED) break;
                    else if (
                        requestStatus == StateRequestStatus.CANCELLED || 
                        requestStatus == StateRequestStatus.OVERRIDDEN || 
                        requestStatus == StateRequestStatus.REJECTED) {
                            abort(false);
                            return;
                        }
                    else if (requestStatus == StateRequestStatus.TIMED_OUT || Instant.now().getEpochSecond() - requestStartTime >= this.requestTimeout) {
                        abort(true);
                        return;
                    }
                    else {
                        try {
                            wait(100);
                        } catch (InterruptedException e) {}
                    }
                }

            } else {
                // Doesn't need to be polled since execute is blocking (IT SHOULDNT BE!!!!!!!!!!! should be handled by MainStateManager)
                ((StateRequestGroup) childStateRequest).execute();

                StateRequestStatus requestStatus = childStateRequest.getStatus();
                if (requestStatus == StateRequestStatus.FULFILLED) break;
                else if (
                    requestStatus == StateRequestStatus.CANCELLED || 
                    requestStatus == StateRequestStatus.OVERRIDDEN || 
                    requestStatus == StateRequestStatus.REJECTED) {
                        abort(false);
                        return;
                    }
                else if (requestStatus == StateRequestStatus.TIMED_OUT) {
                    abort(true);
                    return;
                }
            }

            
        }
        this.updateStatus(StateRequestStatus.FULFILLED);
    }

    private void abort(boolean timedOut) {
        this.updateStatus(timedOut ? StateRequestStatus.TIMED_OUT : StateRequestStatus.CANCELLED);
    }
}
