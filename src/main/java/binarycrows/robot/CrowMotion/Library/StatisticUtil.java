package binarycrows.robot.CrowMotion.Library;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

public class StatisticUtil {
    public static String calculateRegressionLineFormula(ArrayList<double[]> dataTable) {
        if (dataTable == null || dataTable.size() == 0) {
            return "No data";
        }
    
        int n = dataTable.size();
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    
        for (double[] point : dataTable) {
            if (point.length != 2) continue; // skip invalid entries
            double x = point[0];
            double y = point[1];
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }
    
        double denominator = (n * sumX2 - sumX * sumX);
        if (denominator == 0) {
            return "Undefined slope (vertical line)";
        }
    
        double m = (n * sumXY - sumX * sumY) / denominator;
        double b = (sumY - m * sumX) / n;

        // Now compute R^2
        double ssTot = 0;
        double ssRes = 0;
        double meanY = sumY / n;

        for (double[] point : dataTable) {
            if (point.length != 2) continue;
            double x = point[0];
            double y = point[1];
            double predictedY = m * x + b;
            ssRes += Math.pow(y - predictedY, 2);
            ssTot += Math.pow(y - meanY, 2);
        }

        double rSquared = (ssTot == 0) ? 1.0 : 1 - (ssRes / ssTot);
         try (FileWriter writer = new FileWriter("logs/log.csv")) {
            writer.write("x,y\n"); // CSV header
            for (double[] point : dataTable) {
                if (point.length != 2) continue;
                writer.write(point[0] + "," + point[1] + "\n");
            }
        } catch (IOException e) {
        }
        return String.format("%.4fx+%.4f", m, b);
    }
}
