package binarycrows.robot;

import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

@SuppressWarnings("rawtypes")
public class MainStateManager extends Thread {
    // Static variable reference of single_instance
    // of type Singleton
    private static MainStateManager instance = null;
    private boolean isRunning = false;

    private ArrayList<SubStateManager> subStateManagers = new ArrayList<>();

    private HashMap<Enum, SubStateManager> subStateManagerTypeLookup = new HashMap<Enum, SubStateManager>();

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
        subStateManagers.forEach(subStateManager -> {
            subStateManager.periodic();
        });
    }

    public void dispatchStateRequest(StateRequest stateRequest) {
        
    }

    public void requestReturnToDefaultState(SubStateManager subStateManager) {
        subStateManager.returnToDefaultState();
    }


    public SubStateManager resolveSubStateManager(Enum type) {
        return subStateManagerTypeLookup.get(type);
    }

    public static synchronized MainStateManager getInstance()
    {
        if (instance == null) instance = new MainStateManager();
        return instance;
    }
}
