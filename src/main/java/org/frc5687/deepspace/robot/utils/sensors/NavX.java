package org.frc5687.deepspace.robot.utils.sensors;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import edu.wpi.first.wpilibj.SPI;
import org.frc5687.deepspace.robot.utils.math.Rotation2D;

import javax.security.auth.callback.Callback;

public class NavX {

    protected class Callback implements ITimestampedDataSubscriber {
        @Override
        public void timestampedDataReceived(long systemTimestamp, long sensorTimestamp, AHRSUpdateBase update, Object content) {
            synchronized (NavX.this) {
                if (_lastSensorTimestampMs != _invalidTimestampK && _lastSensorTimestampMs < sensorTimestamp) {
                    _yawRateDegreesPerSecond = 1000.0 * (-_yawDegrees - update.yaw) / (double) (sensorTimestamp - _lastSensorTimestampMs);
                }
                _lastSensorTimestampMs = sensorTimestamp;
                _yawDegrees = -update.yaw;
            }
        }
    }

    protected AHRS _AHRS;

    protected Rotation2D _angleAdjustment = Rotation2D.getIdentity();
    protected double _yawDegrees;
    protected double _yawRateDegreesPerSecond;
    protected final long _invalidTimestampK = -1;
    protected long _lastSensorTimestampMs;



    public NavX(SPI.Port spiPortID) {
        _AHRS = new AHRS(spiPortID, (byte) 200);
        resetState();
        _AHRS.registerCallback(new Callback(), null);
    }

    public synchronized void reset() {
        _AHRS.reset();
        resetState();
    }

    public synchronized void zeroYaw() {
        _AHRS.zeroYaw();
        resetState();
    }

    private void resetState() {
        _lastSensorTimestampMs = _invalidTimestampK;
        _yawDegrees = 0.0;
        _yawRateDegreesPerSecond = 0.0;
    }

    public synchronized void setAngleAdjustment(Rotation2D adjustment) {
        _angleAdjustment = adjustment;
    }

    protected synchronized double getRawYawDegrees() {
        return _yawDegrees;
    }

    public Rotation2D getYaw() {
        return _angleAdjustment.rotateBy(Rotation2D.fromDegrees(getRawYawDegrees()));
    }

    public double getYawRateDegreesPerSecond() {
        return _yawRateDegreesPerSecond;
    }

    public double getYawRateRadiansPerSecond() {
        return 180.0 / Math.PI * getYawRateDegreesPerSecond();
    }

    public double getRawAccelX() {
        return _AHRS.getRawAccelX();
    }

}
