package binarycrows.robot;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;


public class StateTable {

    private static volatile HashMap<String, Object> stateTableObjects = new HashMap<String, Object>();

    /**
     * Records an arbitrary object to the State Table.
     * These do not support Advantage Kit logging. You must use a function with a corresponding type to publish to AdvantageKit.
     * @param path
     * @param value
     */
    public static synchronized void putValue(String path, Object value) {
        stateTableObjects.put(path, value);
    }

    
    /**
     * Records a value to the State Table and Advantage Kit.
     * @param path
     * @param value
     */
    public static synchronized void putValue(String path, boolean value) {
        stateTableObjects.put(path, value);
        Logger.recordOutput(path, value);
    }
    /**
     * Records a value to the State Table and Advantage Kit.
     * @param path
     * @param value
     */
    public static synchronized void putValue(String path, int value) {
        stateTableObjects.put(path, value);
        Logger.recordOutput(path, value);
    }
    /**
     * Records a value to the State Table and Advantage Kit.
     * @param path
     * @param value
     */
    public static synchronized void putValue(String path, long value) {
        stateTableObjects.put(path, value);
        Logger.recordOutput(path, value);
    }
    /**
     * Records a value to the State Table and Advantage Kit.
     * @param path
     * @param value
     */
    public static synchronized void putValue(String path, float value) {
        stateTableObjects.put(path, value);
        Logger.recordOutput(path, value);
    }
    /**
     * Records a value to the State Table and Advantage Kit.
     * @param path
     * @param value
     */
    public static synchronized void putValue(String path, double value) {
        stateTableObjects.put(path, value);
        Logger.recordOutput(path, value);
    }
    /**
     * Records a value to the State Table and Advantage Kit.
     * @param path
     * @param value
     */
    public static synchronized void putValue(String path, String value) {
        stateTableObjects.put(path, value);
        Logger.recordOutput(path, value);
    }

    /**
     * Records a value to the State Table and Advantage Kit.
     * @param path
     * @param value
     */
    public static synchronized void putValue(String path, boolean[] value) {
        stateTableObjects.put(path, value);
        Logger.recordOutput(path, value);
    }
    /**
     * Records a value to the State Table and Advantage Kit.
     * @param path
     * @param value
     */
    public static synchronized void putValue(String path, int[] value) {
        stateTableObjects.put(path, value);
        Logger.recordOutput(path, value);
    }
    /**
     * Records a value to the State Table and Advantage Kit.
     * @param path
     * @param value
     */
    public static synchronized void putValue(String path, long[] value) {
        stateTableObjects.put(path, value);
        Logger.recordOutput(path, value);
    }
    /**
     * Records a value to the State Table and Advantage Kit.
     * @param path
     * @param value
     */
    public static synchronized void putValue(String path, float[] value) {
        stateTableObjects.put(path, value);
        Logger.recordOutput(path, value);
    }
    /**
     * Records a value to the State Table and Advantage Kit.
     * @param path
     * @param value
     */
    public static synchronized void putValue(String path, double[] value) {
        stateTableObjects.put(path, value);
        Logger.recordOutput(path, value);
    }
    /**
     * Records a value to the State Table and Advantage Kit.
     * @param path
     * @param value
     */
    public static synchronized void putValue(String path, String[] value) {
        stateTableObjects.put(path, value);
        Logger.recordOutput(path, value);
    }

    public static synchronized void recordNonFatalException(Exception e) {
        stateTableObjects.put("ErrorLog", stateTableObjects.get(e.getMessage()) + "\n");
    }

    public static synchronized boolean getValueAsBoolean(String path) {
        return (boolean) getValue(path);
    }
    public static synchronized int getValueAsInteger(String path) {
        return (int) getValue(path);
    }
    public static synchronized long getValueAsLong(String path) {
        return (long) getValue(path);
    }
    public static synchronized float getValueAsFloat(String path) {
        return (float) getValue(path);
    }
    public static synchronized double getValueAsDouble(String path) {
        return (double) getValue(path);
    }
    public static synchronized String getValueAsString(String path) {
        return (String) getValue(path);
    }
    public static synchronized Object getValue(String path) {
        return stateTableObjects.get(path);
    }
}
