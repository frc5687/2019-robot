package org.frc5687.deepspace.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Lights;

public class StatusProxy extends OutliersProxy {

    OI _oi;
    Lights _lights;


    boolean _hatchSecured;
    boolean _cargoDetcted;
    boolean _cargoSecured;

    TargetDistance _targetDistance;
    TargetAlignment _tartgetAlignment;
    Robot.Configuration _configuration;

    public StatusProxy(Robot robot) {
        _lights = robot.getLights();
        _oi = robot.getOI();
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
                if (DriverStation.getInstance().getAlliance()== DriverStation.Alliance.Red) {
                    _lights.setColor(Constants.Lights.PULSING_RED, 0);
                } else if (DriverStation.getInstance().getAlliance()== DriverStation.Alliance.Blue) {
                    _lights.setColor(Constants.Lights.PULSING_BLUE, 0);
                } else {
                    _lights.setColor(Constants.Lights.SOLID_YELLOW, 0);
                }
                break;
            case hatch:
                if (_hatchSecured) {
                    _lights.setColor(Constants.Lights.SOLID_PURPLE, 0);
                } else {
                    _lights.setColor(Constants.Lights.SOLID_PURPLE, 2);
                }
                break;
            case cargo:
                if (_cargoSecured) {
                    _lights.setColor(Constants.Lights.SOLID_ORANGE, 0);
                } else if (_cargoDetcted) {
                    _lights.setColor(Constants.Lights.SOLID_ORANGE, 4);
                } else {
                    _lights.setColor(Constants.Lights.SOLID_ORANGE, 2);
                }
                break;
            case climb:
                _lights.setColor(Constants.Lights.CONFETTI, 0);
                break;
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
