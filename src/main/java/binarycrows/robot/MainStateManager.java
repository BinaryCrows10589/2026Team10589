package binarycrows.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.StateRequestGroup.SequentialGroup;
import binarycrows.robot.StateRequestGroup.StateRequestGroup;

@SuppressWarnings("rawtypes")
public class MainStateManager extends Thread {
    // Static variable reference of single_instance
    // of type Singleton
    private static MainStateManager instance = null;
    private boolean isRunning = false;

    private ArrayList<SubStateManager> subStateManagers = new ArrayList<>();

    private HashMap<Enum, SubStateManager> subStateManagerTypeLookup = new HashMap<Enum, SubStateManager>();

    private ArrayList<StateRequestGroup> activeStateRequestGroups = new ArrayList<>();

    private MainStateManager()
    {
        assert instance == null;
    }

    public void registerSubStateManagers(SubStateManager... subStateManagers) {
        List<SubStateManager> subStateManagerArray = Arrays.asList(subStateManagers);
        this.subStateManagers.addAll(subStateManagerArray);
        subStateManagerArray.forEach((SubStateManager subStateManager) -> {
            subStateManagerTypeLookup.put(subStateManager.getStateRequestType(), subStateManager);
        });
    }

    public void run() {
        isRunning = true;
        while (isRunning) {
            periodic();
        }

    }

    /**
     * Marks that the periodic loop should end, which will cause the MainStateManager thread to end after the current periodic iteration
     */
    public void endPeriodic() {
        isRunning = false;
    }

    private void periodic() {
        // Keep track of state request groups to remove without live updating the arraylist
        ArrayList<StateRequestGroup> finishedStateRequestGroups = new ArrayList<>();

        for (StateRequestGroup stateRequestGroup : activeStateRequestGroups) {

            if (stateRequestGroup.status == StateRequestStatus.PENDING) {
                stateRequestGroup.updateStatus(StateRequestStatus.RUNNING);
            }

            if (stateRequestGroup instanceof SequentialGroup) { // Sequential Command Group
                SequentialGroup sequentialGroup = (SequentialGroup) stateRequestGroup;

                // This case (we have ran out of commands) be handled by the SequentialGroup... but you never know.
                if (sequentialGroup.currentStateRequestIndex >= sequentialGroup.getLength()) {
                    sequentialGroup.updateStatus(StateRequestStatus.FULFILLED);
                    finishedStateRequestGroups.add(sequentialGroup);
                    continue;
                }

                StateRequest childStateRequest = sequentialGroup.getCurrentStateRequest();
                StateRequestStatus requestStatus = childStateRequest.getStatus();

                if (requestStatus == StateRequestStatus.FRESH) {

                    dispatchStateRequest(childStateRequest);

                } else if (requestStatus == StateRequestStatus.FULFILLED) {

                    sequentialGroup.increment();

                } else if (
                    requestStatus == StateRequestStatus.CANCELLED || 
                    requestStatus == StateRequestStatus.OVERRIDDEN || 
                    requestStatus == StateRequestStatus.REJECTED ||
                    requestStatus == StateRequestStatus.TIMED_OUT) {
                        
                        sequentialGroup.updateStatus(requestStatus);
                        sequentialGroup.setFaultyStateRequest(childStateRequest);
                        finishedStateRequestGroups.add(sequentialGroup);
                }
            } else { // Any other command group class
                throw new UnsupportedOperationException("The class '" + stateRequestGroup.getClass().getCanonicalName() + "' has no StateRequestGroup behavior implementation");
            }
        }

        // Remove unneeded state request groups
        for (StateRequestGroup stateRequestGroup : finishedStateRequestGroups) {
            activeStateRequestGroups.remove(stateRequestGroup);
        }
    }

    @SuppressWarnings("unchecked")
    public void dispatchStateRequest(StateRequest stateRequest) {
        if (stateRequest instanceof StateRequestGroup) {
            StateRequestGroup stateRequestGroup = (StateRequestGroup) stateRequest;
            activeStateRequestGroups.add(stateRequestGroup);
            stateRequestGroup.updateStatus(StateRequestStatus.PENDING);
        }
        Enum stateRequestType = stateRequest.getStateRequestType();
        SubStateManager targetManager = resolveSubStateManager(stateRequestType);
        targetManager.recieveStateRequest(stateRequest);
    }

    public void requestReturnToDefaultState(SubStateManager subStateManager) {
        subStateManager.returnToDefaultState();
    }


    public SubStateManager resolveSubStateManager(Enum type) {
        return subStateManagerTypeLookup.get(type);
    }

    public ArrayList<SubStateManager> getSubStateManagers() {
        return subStateManagers;
    }

    public static synchronized MainStateManager getInstance()
    {
        if (instance == null) instance = new MainStateManager();
        return instance;
    }


}
