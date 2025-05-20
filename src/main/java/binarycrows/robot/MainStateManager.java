package binarycrows.robot;

public class MainStateManager extends Thread {
    // Static variable reference of single_instance
    // of type Singleton
    private static MainStateManager instance = null;
    private boolean isRunning = false;

    

    private MainStateManager()
    {
        assert instance == null;
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

    }

    public static synchronized MainStateManager getInstance()
    {
        if (instance == null) instance = new MainStateManager();
        return instance;
    }
}
