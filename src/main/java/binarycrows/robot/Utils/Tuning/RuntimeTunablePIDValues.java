package binarycrows.robot.Utils.Tuning;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class RuntimeTunablePIDValues {
    private RuntimeTunableValue changablePValue;
    private RuntimeTunableValue changableIValue;
    private RuntimeTunableValue changableDValue;
    private RuntimeTunableValue changableFFValue;

    // Specific to certain implementations of PID
    private RuntimeTunableValue changableGValue; // Gravity
    private RuntimeTunableValue changableSValue; // Static friction
    private RuntimeTunableValue changableVValue; // Velocity
    private RuntimeTunableValue changableAValue; // Acceleration
    private RuntimeTunableValue changableCruiseVelocityValue;
    private RuntimeTunableValue changableAccelerationValue;
    private RuntimeTunableValue changableJerkValue;
    
    private boolean isUsingAdvancedPID = false; // Whether the variables above will be used


    /**
     * WARNING!!! THIS WILL NOT WORK DURING MATCHES AT COMPETION.
     * ALL PID VALUES TUNED HERE SHOULD BE ADDED TO THE RELEVENT CONSTANT FILE ONCE TUNING IS COMPLETE!!!
     * WARNING!!! VALUES WILL NOT STAY BETWEEN DASHBOARD OR ROBOT REBOOTS/CODE REDEPLOYS.
     * Please record the value somehow before proforming these actions to not loose progress
     * @param networkTablesKey String: The start of the key under which the 
     * values will be added to network tables. e.x Module/DrivePIDValues
     *
     * @param defaultPValue Double: The defualt P PID value
     * @param defaultIValue Double: The defualt I PID value
     * @param defaultDValue Double: The defualt D PID value
     * @param defaultFFValue Double: The defualt FF PID value
     */
    public RuntimeTunablePIDValues(String baseNetworkTablesKey, double defaultPValue,
        double defaultIValue, double defaultDValue, double defaultFFValue) {

        this.changablePValue = new RuntimeTunableValue(baseNetworkTablesKey + "/PValue", (double)defaultPValue);
        this.changableIValue = new RuntimeTunableValue(baseNetworkTablesKey + "/IValue", (double)defaultIValue);
        this.changableDValue = new RuntimeTunableValue(baseNetworkTablesKey + "/DValue", (double)defaultDValue);
        this.changableFFValue = new RuntimeTunableValue(baseNetworkTablesKey + "/FFValue", (double)defaultFFValue);
    }

    public RuntimeTunablePIDValues(String baseNetworkTablesKey,
    double defaultPValue,
    double defaultIValue,
    double defaultDValue,
    double defaultFFValue,
    double defaultGValue,
    double defaultSValue,
    double defaultVValue,
    double defaultAValue,
    double defaultVelocityValue,
    double defaultAccelerationValue,
    double defaultJerkValue) {
        this.changablePValue = new RuntimeTunableValue(baseNetworkTablesKey + "/PValue", (double)defaultPValue);
        this.changableIValue = new RuntimeTunableValue(baseNetworkTablesKey + "/IValue", (double)defaultIValue);
        this.changableDValue = new RuntimeTunableValue(baseNetworkTablesKey + "/DValue", (double)defaultDValue);
        this.changableFFValue = new RuntimeTunableValue(baseNetworkTablesKey + "/FFValue", (double)defaultFFValue);
        this.changableGValue = new RuntimeTunableValue(baseNetworkTablesKey + "/GValue", (double)defaultGValue);
        this.changableSValue = new RuntimeTunableValue(baseNetworkTablesKey + "/SValue", (double)defaultSValue);
        this.changableVValue = new RuntimeTunableValue(baseNetworkTablesKey + "/VValue", (double)defaultVValue);
        this.changableAValue = new RuntimeTunableValue(baseNetworkTablesKey + "/AValue", (double)defaultAValue);
        this.changableCruiseVelocityValue = new RuntimeTunableValue(baseNetworkTablesKey + "/CruseVelocityValue", (double) defaultVelocityValue);
        this.changableAccelerationValue = new RuntimeTunableValue(baseNetworkTablesKey + "/AccelerationValue", (double)defaultAccelerationValue);
        this.changableJerkValue = new RuntimeTunableValue(baseNetworkTablesKey + "/JerkValue", (double)defaultJerkValue);
        isUsingAdvancedPID = true;
    }

    
    

    /**
     * The values of all four PID Constant as an array of doubles
     * @return Double[]-{PValue, IValue, DValue, FFValue, (GValue, SValue)} The values of the PID as an array
     */
    public double[] getUpdatedPIDConstants() {
        double[] arrayOfPIDValues;

        if (isUsingAdvancedPID) {

            arrayOfPIDValues = new double[] {
                (double)this.changablePValue.getValue(),
                (double)this.changableIValue.getValue(),
                (double)this.changableDValue.getValue(),
                (double)this.changableFFValue.getValue(),
                (double)this.changableGValue.getValue(),
                (double)this.changableSValue.getValue(),
                (double)this.changableVValue.getValue(),
                (double)this.changableAValue.getValue(),
                (double)this.changableCruiseVelocityValue.getValue(),
                (double)this.changableAccelerationValue.getValue(),
                (double)this.changableJerkValue.getValue()
            };

        } else {

            arrayOfPIDValues = new double[] {
                (double)this.changablePValue.getValue(),
                (double)this.changableIValue.getValue(),
                (double)this.changableDValue.getValue(),
                (double)this.changableFFValue.getValue()
            };

        }
        return arrayOfPIDValues;
    }

    public boolean hasAnyPIDValueChanged() {
        boolean valuesChanged =  
        this.changablePValue.hasValueChanged() || 
        this.changableIValue.hasValueChanged() ||
        this.changableDValue.hasValueChanged() || 
        this.changableFFValue.hasValueChanged();
        if (isUsingAdvancedPID) {
            return valuesChanged ||
            this.changableGValue.hasValueChanged() ||
            this.changableSValue.hasValueChanged() ||
            this.changableVValue.hasValueChanged() ||
            this.changableAValue.hasValueChanged() ||
            this.changableCruiseVelocityValue.hasValueChanged() ||
            this.changableAccelerationValue.hasValueChanged() ||
            this.changableJerkValue.hasValueChanged();
        } else {
            return valuesChanged;
        }
    }

    public void updatePIDValues(SparkMax motor) {
        if (hasAnyPIDValueChanged()) {
            double[] pidConstants = getUpdatedPIDConstants();
            SparkMaxConfig newConfig = new SparkMaxConfig();
            newConfig.closedLoop.pid(pidConstants[0], pidConstants[1], pidConstants[2]);
            motor.configure(newConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }
    public void updatePIDValues(TalonFX motor) {
        if (hasAnyPIDValueChanged()) {
            double[] pidConstants = getUpdatedPIDConstants();
            Slot0Configs newConfig = new Slot0Configs();
            newConfig.kP = pidConstants[0];
            newConfig.kI = pidConstants[1];
            newConfig.kD = pidConstants[2];
            motor.getConfigurator().apply(newConfig);
        }
    }

    public void updatePIDValues(TalonFXS motor) {
        if (hasAnyPIDValueChanged()) {
            double[] pidConstants = getUpdatedPIDConstants();
            Slot0Configs newConfig = new Slot0Configs();
            newConfig.kP = pidConstants[0];
            newConfig.kI = pidConstants[1];
            newConfig.kD = pidConstants[2];
            motor.getConfigurator().apply(newConfig);
        }
    }
}
