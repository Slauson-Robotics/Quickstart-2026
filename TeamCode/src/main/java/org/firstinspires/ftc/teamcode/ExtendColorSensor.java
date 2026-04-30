package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class ExtendColorSensor {
    private final ColorSensor cs;

    public ExtendColorSensor(ColorSensor cs) {
        this.cs = cs;
    }

    public boolean ballPresent(int minValue) {
        return this.cs.green() >= minValue || this.cs.blue() >= minValue;
    }

    public int green() {return cs.green();}
    public int blue() {return cs.blue();}
}
