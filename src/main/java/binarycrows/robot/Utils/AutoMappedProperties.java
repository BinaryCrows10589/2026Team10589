package binarycrows.robot.Utils;

import java.util.HashMap;


public class AutoMappedProperties<E> {
    private HashMap<String, Integer> lookupTable;
    
    public AutoMappedProperties(Class<E> object) {
        lookupTable = ConversionUtils.fieldsToMap(object.getFields(), null);
    }
    
    public int nameToID(String name) {
        return lookupTable.get(name);
    }
    public String IDToName(int id) {
        return ConversionUtils.getKeyByValue(lookupTable, id);
    }
}
