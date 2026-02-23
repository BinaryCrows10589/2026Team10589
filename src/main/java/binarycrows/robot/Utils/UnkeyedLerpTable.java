package binarycrows.robot.Utils;

public class UnkeyedLerpTable {
    private double[][] lerpTable;
    private boolean translateNegatives;

    public UnkeyedLerpTable(double[][] lerpTable, boolean translateNegatives) {
        this.lerpTable = lerpTable;
        this.translateNegatives = translateNegatives;
    }

    /***
     * Retrieve a value from the table with interpolation
     * @param value The value to be interpolated
     * @param targetIndice The indice of the values to be returned from
     * @param keyIndice The indice of the values to apply the interpolation to
     * @return
     */
    public double get(double value, int targetIndice, int keyIndice) {
        double sign = Math.signum(value);
        if (translateNegatives) value = Math.abs(value);
        int[] indices = findClosestIndices(value, keyIndice);
        return findLerpValue(indices[0], indices[1], value, targetIndice, keyIndice) * (translateNegatives ? sign : 1);
    }

    private double findLerpValue(int lowerIndex, int higherIndex, double value, int targetIndice, int keyIndice) {
        if (lowerIndex == higherIndex) return lerpTable[higherIndex][targetIndice];
        double x1 = lerpTable[lowerIndex][keyIndice];
        double y1 = lerpTable[lowerIndex][targetIndice];
        double x2 = lerpTable[higherIndex][keyIndice];
        double y2 = lerpTable[higherIndex][targetIndice];
        return (y2-y1) / (x2-x1) * (value-x1) + y1; // Two-point form of a line
    }

    /* Returns {Smaller, Larger} */
    // Algorithm is a modified version of binary search from GeeksForGeeks
    private int[] findClosestIndices(double x, int keyIndice){
        int subArrayStart = 0;
        int subArrayEnd = lerpTable.length - 1;
        if (x < lerpTable[0][keyIndice]) return new int[] {0,0};
        if (x > lerpTable[lerpTable.length-1][keyIndice]) return new int[] {lerpTable.length-1, lerpTable.length-1};
        
        while (subArrayStart <= subArrayEnd){
            
            int targetIndex = (subArrayStart + subArrayEnd) / 2;

            // Exact match!
            if (lerpTable[targetIndex][keyIndice] == x) {
                return new int[] {targetIndex, targetIndex};

            }
            // Target lies in smaller portion
            else if (lerpTable[targetIndex][keyIndice] > x) {
                subArrayEnd = targetIndex - 1;
            }
            // Target lies in larger portion
            else {
                subArrayStart = targetIndex + 1;
            }
        }

        // Search finished with no exact match
        return new int[] {subArrayEnd, subArrayStart};
    }
}
