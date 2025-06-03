package binarycrows.robot.Utils.Tuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RuntimeTunableValue {
    private Object changableValue;
    private Object changableValueType;
    private Object lastChangableValue;
    private Integer genericInt = 0;
    private Double genericDouble = 0.0;
    private Boolean genericBoolean = false; 
    private String networkTablesKey = "NoChangableValueKeyProvided";
    private boolean hasChangableValueChanged = false;

    /**
     * WARNING!!! THIS WILL NOT WORK DURING MATCHES AT COMPETION.
     * ALL VALUES TUNED HERE SHOULD BE ADDED TO THE RELEVENT CONSTANT FILE ONCE TUNING IS COMPLETE!!!
     * WaRNING!!! VALUES MAY NOT STAY BETWEEN DASHBOARD OR ROBOT REBOOTS/CODE REDEPLOYS.
     * Please record the value somehow before proforming these actions to not loose progress
     * Adds a changable value to Smartdashboard. This allows for PID values to be 
     * tuned without redeploying code every change or for flywheel speed tuning, or other things of this nature.
     * @param networkTablesKey String: The key underwhich the value will be added to networktables. 
     * @param changableValue Object: The inital value for the changable value. This can be a Integer, Double or Boolean
     */
    public RuntimeTunableValue(String networkTablesKey, Object changableValue) {
        this.changableValue = changableValue;
        this.networkTablesKey = networkTablesKey;
        this.lastChangableValue = this.changableValue;
        this.changableValueType = this.changableValue.getClass();
        publishValue();
    }   

    /**
     * Adds the changable value in network tables.. 
     */
    private void publishValue() {
        if(this.changableValueType.equals(this.genericInt.getClass())) {
            SmartDashboard.putNumber(networkTablesKey, (int)changableValue);
        } else if(this.changableValueType.equals(this.genericDouble.getClass())) {
            SmartDashboard.putNumber(networkTablesKey, (double)changableValue);
        } else if(this.changableValueType.equals(this.genericBoolean.getClass())) {
            SmartDashboard.putBoolean(networkTablesKey, (boolean)changableValue);
        } else {
            SmartDashboard.putString(networkTablesKey, "Unsupported Type added; Only integers, doubles, and booleans are supported.");
        }
    }

    /**
     * Returns the current changableValue's value as a genaric object. Must be cased to desired type
     * @return Object: The current changableValue's value as a genaric object. Must be cased to desired type
     */
    public Object getValue() {
        boolean hasChanged = false;
        if(this.changableValueType.equals(this.genericInt.getClass())) {
            this.changableValue =  SmartDashboard.getNumber(networkTablesKey, (int)changableValue);
           hasChanged =  (int)this.changableValue != (int)this.lastChangableValue;
        } else if (this.changableValueType.equals(this.genericDouble.getClass())) {
            this.changableValue = SmartDashboard.getNumber(networkTablesKey, (double)changableValue);
            hasChanged = (double)this.changableValue != (double)this.lastChangableValue;
        } else if(this.changableValueType.equals(this.genericBoolean.getClass())) {
            this.changableValue = SmartDashboard.getBoolean(networkTablesKey, (boolean)changableValue);
            hasChanged = (boolean)this.changableValue != (boolean)this.lastChangableValue;
        } else {
            SmartDashboard.putString(networkTablesKey, "Unsupported Type added; Only integers, doubles, and booleans are supported.");
        }
    
        if(hasChanged) {
            this.lastChangableValue = this.changableValue;
            this.hasChangableValueChanged = true;
        }

        return this.changableValue;
    }

    /**
     * Returns Wether or not the changabelValue has changed sence last check and sets it to false if it has.
     * @return Boolean: Wether or not the changabelValue has changed sence last check
     */
    public boolean hasValueChanged() {
        boolean originalValue = this.hasChangableValueChanged;
        this.hasChangableValueChanged = this.hasChangableValueChanged ? !this.hasChangableValueChanged : this.hasChangableValueChanged;
        return originalValue;
    }
}
