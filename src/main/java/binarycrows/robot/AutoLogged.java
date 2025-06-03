package binarycrows.robot;

import java.lang.reflect.Field;

public class AutoLogged {
    private String path = "";

    protected void updatePath(String path) {
        this.path = path;
    }


    public void logToStateTable() {
        for (Field field : this.getClass().getFields()) {
            try {

                Object fieldValue = field.get(null);

                if (fieldValue instanceof Boolean) StateTable.putValue(path + (path == "" ? "" : "/") + field.getName(), (Boolean) fieldValue);
                if (fieldValue instanceof Integer) StateTable.putValue(path + (path == "" ? "" : "/") + field.getName(), (Integer) fieldValue);
                if (fieldValue instanceof Long) StateTable.putValue(path + (path == "" ? "" : "/") + field.getName(), (Long) fieldValue);
                if (fieldValue instanceof Float) StateTable.putValue(path + (path == "" ? "" : "/") + field.getName(), (Float) fieldValue);
                if (fieldValue instanceof Double) StateTable.putValue(path + (path == "" ? "" : "/") + field.getName(), (Double) fieldValue);
                if (fieldValue instanceof String) StateTable.putValue(path + (path == "" ? "" : "/") + field.getName(), (String) fieldValue);

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
