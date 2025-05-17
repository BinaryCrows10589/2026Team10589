package binarycrows.robot;

import java.util.HashMap;

public class StateTable {

    private static HashMap<String, Object> table = new HashMap<String, Object>();

    public static synchronized void setValue(String key, Object value) {
        table.put(key, value);
    }

    public static synchronized Object getValue(String key) {
        return table.get(key);
    }

}
