package org.frc5687.deepspace.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Lights;
import org.frc5687.deepspace.robot.utils.Limelight;

public class StatusProxy extends OutliersProxy {

    OI _oi;
    Lights _lights;
    Limelight _limelight;
    boolean _hatchSecured;
    boolean _cargoDetcted;
    boolean _cargoSecured;

    TargetDistance _targetDistance;
    TargetAlignment _tartgetAlignment;
    Robot.Configuration _configuration;

    public StatusProxy(Robot robot) {
        _lights = robot.getLights();
        _oi = robot.getOI();
        _limelight = robot.getLimelight();

    }

    public void setConfiguration(Robot.Configuration configuration) {
        _configuration = configuration;
        update();
    }

    public void setHatchSecured(boolean hatchSecured) {
        _hatchSecured = hatchSecured;
    }

    public boolean getHatchSecured() {
        return _hatchSecured;
    }

    public void update() {
        switch (_configuration) {
            case starting:
                if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Red) {
                    _lights.setColor(Constants.Lights.PULSING_RED, 0);
                } else if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
                    _lights.setColor(Constants.Lights.PULSING_BLUE, 0);
                } else {
                    _lights.setColor(Constants.Lights.SOLID_YELLOW, 0);
                }
                break;
            case hatch:
                _oi.setCargoLED(0);
                if (_hatchSecured) {
                    _lights.setColor(Constants.Lights.SOLID_PURPLE, 0);
                    _oi.setHatchLED(1);
                } else {
                    _lights.setColor(Constants.Lights.SOLID_PURPLE, 2);
                    _oi.setHatchLED(2);
                }
                break;
            case cargo:
                _oi.setHatchLED(0);
                if (_cargoSecured) {
                    _lights.setColor(Constants.Lights.SOLID_ORANGE, 0);
                    _oi.setCargoLED(1);
                } else if (_cargoDetcted) {
                    _lights.setColor(Constants.Lights.SOLID_ORANGE, 4);
                    _oi.setCargoLED(2);
                } else {
                    _lights.setColor(Constants.Lights.SOLID_ORANGE, 2);
                    _oi.setCargoLED(2);
                }
                break;
        }
        if(_limelight.isTargetSighted()){
            _oi.setTargetLED(2);
        }
        else if (_limelight.isTargetCentered()) {
            _oi.setTargetLED(1);
        } else {
            _oi.setTargetLED(0);
        }

        if(_limelight.getHorizontalAngle() < 0){
            _oi.setSideLED(0);
        } else if(_limelight.getHorizontalAngle() > 0){
            _oi.setSideLED(1);
        }else {
            _oi.setSideLED(2);
        }
    }




    @Override
    public void updateDashboard() {
        metric("Configuration", _configuration.name());
    }




    public enum TargetDistance {
        None,
        TooFar,
        OnTarget,
        TooClose
    }

    public enum TargetAlignment {
        None,
        Left,
        Center,
        Right
    }
}
