package frc.robot.utilities;

import edu.wpi.first.wpilibj.Preferences;


public class ModifiableConstant{
    private final double defaultValue;
    private final String name;
    private final String lookup;

    public ModifiableConstant(String name, double defaultValue){
        this.defaultValue = defaultValue;
        this.name = name;
        this.lookup = "../Constants/" + this.name;
        Preferences.setDouble(lookup,defaultValue);
    }

    public double value(){
        return Preferences.getDouble(lookup,defaultValue);
    }

    public void setValue(double value){
        Preferences.setDouble(lookup,value);
    }
}
