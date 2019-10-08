package org.frc5687.deepspace.robot.utils.sensors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Navx extends AHRS {

    /**
     * Taken from 1114 Simbotics.
     */
    private double _prevAngle;
    private double _angle;

    public Navx() {
        super(SPI.Port.kMXP);
        reset();
    }

    @Override
    public double getAngle() {
        return 180 - (_angle + getFusedHeading());
    }

    public void update() {
        double difference = getFusedHeading() - _prevAngle;
        if (difference > 180) {
            _angle -= 360;
        } else if (difference < -180) {
            _angle += 360;
        }
        _prevAngle += difference;
    }

    @Override
    public void reset() {
        _angle = 90 - getFusedHeading();
        _prevAngle = getFusedHeading();
    }

}
