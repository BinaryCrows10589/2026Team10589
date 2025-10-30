package binarycrows.robot.SeasonCode.Utils;

public class DesiredMetersPerSecondToVoltageLerpTable {
    //TODO: Boyne look over this
    private double groundAjustFeedForword;
    private double minSpeedForFeedforword = .3; //TODO: Calabrate this

    private double[][] acceleratingTable = {
        new double[] {
            // Meters Per Second
        }, new double[]{
            // Voltage
        }
    };

    private double[][] deceleratingTable = {
        new double[] {
            // Meters Per Second
        }, new double[]{
            // Voltage
        }
    };


    public double metersPerSecondToVoltage(double desiredMetersPerSecond, double currentVelocity) {
          boolean isAccelerating = desiredMetersPerSecond >= currentVelocity;
          double voltage = 0;
     
          if(isAccelerating) {
             voltage = lerp(acceleratingTable, Math.abs(desiredMetersPerSecond)) + groundAjustFeedForword;
          } else {
              double rawVoltage = lerp(deceleratingTable, Math.abs(desiredMetersPerSecond));
              double feedForword = Math.abs(desiredMetersPerSecond) > minSpeedForFeedforword ? groundAjustFeedForword : 0;
              voltage = rawVoltage + feedForword;
          }
          voltage *= Math.signum(desiredMetersPerSecond);
          return voltage;
     }

    private double lerp(double[][] lerpTable, double input) {
        int low = 0;
        int high = lerpTable[0].length - 1;
        int closestLowerIndex = -1;
    
        while (low <= high) {
            int mid = (low + high) / 2;
            double midVal = lerpTable[0][mid];
    
            if (midVal <= input) {
                closestLowerIndex = mid;
                low = mid + 1;
            } else {
                high = mid - 1;
            }
        }
        double output = 0;
        if(closestLowerIndex == -1) {
            output = 0;
        } else if(closestLowerIndex == high) {
            output = lerpTable[1][high];
        } else {
            double lowNumber = lerpTable[1][closestLowerIndex];
            double highNumber = lerpTable[1][closestLowerIndex+1];
            double outputDelta = highNumber - lowNumber;
            double inputLowNumber = lerpTable[0][closestLowerIndex];
            double inputHighNumber = lerpTable[0][closestLowerIndex+1];
            double inputDelta = inputHighNumber - inputLowNumber;
            output = lowNumber + (outputDelta * ((input - inputLowNumber)/ inputDelta));
        }
        return output;
    }
}
