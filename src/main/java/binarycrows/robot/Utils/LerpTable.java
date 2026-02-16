package binarycrows.robot.Utils;

public class LerpTable {
    private double[] lerpKey;
    private double[] lerpValue;
    private boolean translateNegatives;

    public LerpTable(double[] lerpKey, double[] lerpValue, boolean translateNegatives) {
        this.lerpKey = lerpKey;
        this.lerpValue = lerpValue;
        this.translateNegatives = translateNegatives;
    }

    public double get(double value) {
        double sign = Math.signum(value);
        if (translateNegatives) value = Math.abs(value);
        int[] indices = findClosestIndices(value);
        return findLerpValue(indices[0], indices[1], value) * (translateNegatives ? sign : 1);
    }

    private double findLerpValue(int lowerIndex, int higherIndex, double value) {
        if (lowerIndex == higherIndex) return lerpValue[higherIndex];
        double x1 = lerpKey[lowerIndex];
        double y1 = lerpValue[lowerIndex];
        double x2 = lerpKey[higherIndex];
        double y2 = lerpValue[higherIndex];
        return (y2-y1) / (x2-x1) * (value-x1) + y1; // Two-point form of a line
    }

    /* Returns {Smaller, Larger} */
    // Algorithm is binary search from GeeksForGeeks
    private int[] findClosestIndices(double x){
        int subArrayStart = 0;
        int subArrayEnd = lerpKey.length - 1;
        if (x < lerpKey[0]) return new int[] {0,0};
        if (x > lerpKey[lerpKey.length-1]) return new int[] {lerpKey.length-1, lerpKey.length-1};
        
        while (subArrayStart <= subArrayEnd){
            
            int targetIndex = (subArrayStart + subArrayEnd) / 2;

            // Exact match!
            if (lerpKey[targetIndex] == x) {
                return new int[] {targetIndex, targetIndex};

            }
            // Target lies in smaller portion
            else if (lerpKey[targetIndex] > x) {
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
