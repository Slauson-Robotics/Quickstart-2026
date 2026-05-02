package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

public interface customsensor extends ColorSensor {
    default boolean ballPresent(int val) {
        return (green() >= val || blue() >= val);
    }


}
