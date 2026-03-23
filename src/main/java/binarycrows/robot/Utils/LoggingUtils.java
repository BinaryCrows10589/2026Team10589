package binarycrows.robot.Utils;

import java.lang.reflect.Field;

import org.littletonrobotics.junction.Logger;

import binarycrows.robot.StateRequest;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class LoggingUtils {
    public static void logObject(String path, Object outputs) {
        if (!path.endsWith("/")) path += "/";
        
        for (Field field : outputs.getClass().getFields()) {
            try {

                Object fieldValue = field.get(outputs);
                if (fieldValue == null) continue;

                String absolutePath = path + field.getName();
                
                
                logToAdvantageKit(fieldValue, absolutePath);

            } catch (IllegalArgumentException e) {
                e.printStackTrace();
                System.err.println(e);
                
            } catch (IllegalAccessException e) {
                e.printStackTrace();
                System.err.println(e);
            }
        }
    }


    @SuppressWarnings("rawtypes")
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
        else if (fieldValue instanceof Pose2d             ) Logger.recordOutput(absolutePath, (Pose2d             ) fieldValue);
        else if (fieldValue instanceof Pose3d             ) Logger.recordOutput(absolutePath, (Pose3d             ) fieldValue);
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
