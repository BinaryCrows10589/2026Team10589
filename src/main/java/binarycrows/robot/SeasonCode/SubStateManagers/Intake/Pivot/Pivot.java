package binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot.PivotIO.PivotOutputs;
import binarycrows.robot.Utils.Tuning.RuntimeTunableValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pivot {
    private PivotIO pivotIO;
    private PivotOutputs outputs;

    private boolean usePID = false;
    private double externalControlValue;

    private LoggedMechanism2d pivotMech2d;
    private LoggedMechanismRoot2d pivotMechRoot;
    private LoggedMechanismLigament2d pivotMechLigament;

    public Pivot(PivotOutputs outputs) {
        this.outputs = outputs;
        pivotIO = MetaConstants.isReal ? new PivotTalonFX(outputs) : new PivotSim(outputs);

        pivotMech2d = new LoggedMechanism2d(1, 1);
        pivotMechRoot = pivotMech2d.getRoot("pivot", 0.5, 0.5);
        pivotMechLigament = new LoggedMechanismLigament2d("pivotLigament", 0.5, 0);
        pivotMechRoot.append(pivotMechLigament);

    }

    public void setPIDTarget(Rotation2d position) {
        externalControlValue = position.getRadians();
        usePID = true;
    }

    public void setVoltage(double voltage) {
        externalControlValue = voltage;
        usePID = false;
    }

    public void update() { 
       
        if (usePID) {
            pivotIO.setTargetPosition(Rotation2d.fromRadians(externalControlValue));
        } else {
            pivotIO.setRotorVoltage(externalControlValue);
        }
        pivotIO.update();

        pivotMechLigament.setAngle(outputs.pivotRotation);
        Logger.recordOutput("PivotMech2d", pivotMech2d);
    }
}
