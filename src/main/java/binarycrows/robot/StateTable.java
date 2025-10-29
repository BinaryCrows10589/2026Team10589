package binarycrows.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map.Entry;
import java.util.Set;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;

import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class StateTable {

    private static volatile HashMap<String, Object> stateTableObjects = new HashMap<String, Object>();
    private static volatile HashMap<String, Object> advantageKitStateTable = new HashMap<String, Object>();

    /**
     * Records an arbitrary object to the State Table.
     * @param <T>
     * @param path
     * @param value
     */
    @SuppressWarnings("unchecked")
    public static synchronized void putValue(String path, Object value) {
        // Fix common types to be correct
        if (value instanceof ArrayList) {
            if (((ArrayList)value).size() != 0) {

                @SuppressWarnings("rawtypes")
                Object firstIndex = ((ArrayList)value).get(0);

                System.out.println("First index of " + firstIndex);

                if (firstIndex instanceof Double) {
                    value = ((ArrayList<Double>)value).stream().mapToDouble(Double::doubleValue).toArray();
                }
            } else return; // TODO: Don't ignore empty arrays???
        }

        stateTableObjects.put(path, value);
        updateAdvantageKit();
    }

    public static synchronized void recordNonFatalException(Exception e) {
        stateTableObjects.put("ErrorLog", stateTableObjects.get(e.getMessage()) + "\n");
    }

    public static synchronized boolean getValueAsBoolean(String path) {
        System.out.println(path);
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
    public static synchronized Rotation2d getValueAsRotation2d(String path) {
        return (Rotation2d) getValue(path);
    }
    public static synchronized Object getValue(String path) {
        System.out.println("Accessing " + path + "... (its value is " + stateTableObjects.get(path) + ")");
        return stateTableObjects.get(path);
    }

    public static void updateAdvantageKit() { // TODO: This is written to make it possible to function on a separate thread to the updating, but it can be inefficient for the poor Rio due to the hashmap shenanigans
        Set<Entry<String, Object>> entries = new HashSet<Entry<String, Object>>(stateTableObjects.entrySet());
        entries.removeAll(advantageKitStateTable.entrySet());
        if (entries.isEmpty()) return;

        for (Entry<String, Object> entry : entries) {
            advantageKitStateTable.put(entry.getKey(), entry.getValue());
            logToAdvantageKit(entry.getValue(), entry.getKey());
            System.out.println("Logged " + entry.getValue() + " to " + entry.getKey());
        }
    }

    public static void logToAdvantageKit(Object fieldValue, String absolutePath) {
        if (!absolutePath.endsWith("/")) absolutePath += "/";
        // I'm sorry
        if      (fieldValue instanceof Boolean            ) Logger.recordOutput(absolutePath, (Boolean            ) fieldValue);
        else if (fieldValue instanceof Integer            ) Logger.recordOutput(absolutePath, (Integer            ) fieldValue);
        else if (fieldValue instanceof Long               ) Logger.recordOutput(absolutePath, (Long               ) fieldValue);
        else if (fieldValue instanceof Float              ) Logger.recordOutput(absolutePath, (Float              ) fieldValue);
        else if (fieldValue instanceof Double             ) Logger.recordOutput(absolutePath, (Double             ) fieldValue);
        else if (fieldValue instanceof String             ) Logger.recordOutput(absolutePath, (String             ) fieldValue);
        else if (fieldValue instanceof Rotation2d         ) Logger.recordOutput(absolutePath, (Rotation2d         ) fieldValue);
        else if (fieldValue instanceof boolean[]          ) Logger.recordOutput(absolutePath, (boolean[]          ) fieldValue);
        else if (fieldValue instanceof int[]              ) Logger.recordOutput(absolutePath, (int[]              ) fieldValue);
        else if (fieldValue instanceof long[]             ) Logger.recordOutput(absolutePath, (long[]             ) fieldValue);
        else if (fieldValue instanceof float[]            ) Logger.recordOutput(absolutePath, (float[]            ) fieldValue);
        else if (fieldValue instanceof double[]           ) Logger.recordOutput(absolutePath, (double[]           ) fieldValue);
        else if (fieldValue instanceof String[]           ) Logger.recordOutput(absolutePath, (String[]           ) fieldValue);
        else if (fieldValue instanceof SwerveModuleState[]) Logger.recordOutput(absolutePath, (SwerveModuleState[]) fieldValue);
        
        else if (fieldValue instanceof StateRequest       ) Logger.recordOutput(absolutePath, ((StateRequest       ) fieldValue).getAsLoggable());
        else {
            if (!MetaConstants.inProduction) {
                if (fieldValue == null) {
                    throw new ClassCastException("Null value recorded to state table at " + absolutePath);
                }
                throw new ClassCastException("Could not find a suitable AdvantageKit compatible type for the state table object of class " + fieldValue.getClass().getCanonicalName() + " recorded at " + absolutePath);
            }
        }
    }
}
