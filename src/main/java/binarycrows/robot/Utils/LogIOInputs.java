package binarycrows.robot.Utils;

import java.io.InvalidClassException;
import java.lang.reflect.Field;

import binarycrows.robot.StateTable;

public class LogIOInputs {

    public static void logToStateTable(Object inputs, String path) {
        
        for (Field field : inputs.getClass().getFields()) {
            try {

                Object fieldValue = field.get(inputs);

                String absolutePath = path + field.getName();
                
                // I'm sorry
                if      (fieldValue instanceof Boolean  ) StateTable.putValue(absolutePath, (Boolean  ) fieldValue);
                else if (fieldValue instanceof Integer  ) StateTable.putValue(absolutePath, (Integer  ) fieldValue);
                else if (fieldValue instanceof Long     ) StateTable.putValue(absolutePath, (Long     ) fieldValue);
                else if (fieldValue instanceof Float    ) StateTable.putValue(absolutePath, (Float    ) fieldValue);
                else if (fieldValue instanceof Double   ) StateTable.putValue(absolutePath, (Double   ) fieldValue);
                else if (fieldValue instanceof String   ) StateTable.putValue(absolutePath, (String   ) fieldValue);
                else if (fieldValue instanceof boolean[]) StateTable.putValue(absolutePath, (boolean[]) fieldValue);
                else if (fieldValue instanceof int[]    ) StateTable.putValue(absolutePath, (int[]) fieldValue);
                else if (fieldValue instanceof long[]   ) StateTable.putValue(absolutePath, (long[]   ) fieldValue);
                else if (fieldValue instanceof float[]  ) StateTable.putValue(absolutePath, (float[]  ) fieldValue);
                else if (fieldValue instanceof double[] ) StateTable.putValue(absolutePath, (double[] ) fieldValue);
                else if (fieldValue instanceof String[] ) StateTable.putValue(absolutePath, (String[] ) fieldValue);
                else assert false;
                

            } catch (IllegalArgumentException e) {
                e.printStackTrace();
                StateTable.recordNonFatalException(e);
                
            } catch (IllegalAccessException e) {
                e.printStackTrace();
                StateTable.recordNonFatalException(e);
            }
        }
    }
}
