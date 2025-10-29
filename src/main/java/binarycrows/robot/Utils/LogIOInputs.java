package binarycrows.robot.Utils;

import java.lang.reflect.Field;

import binarycrows.robot.StateTable;
import edu.wpi.first.math.geometry.Rotation2d;

public class LogIOInputs {

    /*public static void logObjectToStateTable(Object inputs, String path) {
        if (!path.endsWith("/")) path += "/";
        
        for (Field field : inputs.getClass().getFields()) {
            try {

                Object fieldValue = field.get(inputs);
                if (fieldValue == null) continue;

                String absolutePath = path + field.getName();
                
                
                logToStateTable(fieldValue, absolutePath);

            } catch (IllegalArgumentException e) {
                e.printStackTrace();
                StateTable.recordNonFatalException(e);
                
            } catch (IllegalAccessException e) {
                e.printStackTrace();
                StateTable.recordNonFatalException(e);
            }
        }
    }*/

    public static void logToStateTable(Object fieldValue, String absolutePath) {
        if (!absolutePath.endsWith("/")) absolutePath += "/";
        StateTable.putValue(absolutePath, fieldValue);
    }
}
