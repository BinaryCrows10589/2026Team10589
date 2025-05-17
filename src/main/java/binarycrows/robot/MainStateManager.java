package binarycrows.robot;

public class MainStateManager {
    // Static variable reference of single_instance
    // of type Singleton
    private static MainStateManager instance = null;

    public String s;

    private MainStateManager()
    {
        assert instance == null;
    }

    // Static method to create instance of Singleton class
    public static synchronized MainStateManager getInstance()
    {
        if (instance == null) instance = new MainStateManager();
        return instance;
    }
}
