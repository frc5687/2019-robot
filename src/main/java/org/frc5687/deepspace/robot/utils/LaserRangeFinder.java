package org.frc5687.deepspace.robot.utils;

import edu.wpi.first.wpilibj.I2C;
import org.frc5687.deepspace.robot.Robot;

import static org.frc5687.deepspace.robot.utils.Helpers.hexStringToByteArray;

/**
 * https://github.com/OlivierLD/raspberry-coffee/blob/master/I2C.SPI/src/i2c/sensor/VL53L0X.java
 * 
 */

public class LaserRangeFinder extends OutliersProxy {
    private I2C _rangeFinder;

    public LaserRangeFinder(Robot robot) {
        _rangeFinder = new I2C(I2C.Port.kOnboard, 0x52);
//		if (this.readU8(0xC0) != 0xEE || this.readU8(0xC1) != 0xAA || this.readU8(0xC2) != 0x10) {
        if (!_rangeFinder.verifySensor(0xC0, 3, hexStringToByteArray("EEAA10"))) {
            error("Unable to initialize RangeFinder.");
        }


    }

    @Override
    public void updateDashboard() {

    }
}
