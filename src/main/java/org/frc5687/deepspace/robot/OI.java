package org.frc5687.deepspace.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.frc5687.deepspace.robot.commands.DropBall;
import org.frc5687.deepspace.robot.utils.Gamepad;
import org.frc5687.deepspace.robot.utils.OutliersProxy;

import static org.frc5687.deepspace.robot.utils.Helpers.applyDeadband;
import static org.frc5687.deepspace.robot.utils.Helpers.applySensitivityFactor;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;

    private Button _operatorLeftBumper;
    private Button _operatorRightBumper;

    public OI(){
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
        _operatorLeftBumper = new JoystickButton(_operatorGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());
        _operatorRightBumper = new JoystickButton(_operatorGamepad, Gamepad.Buttons.RIGHT_BUMPER.getNumber());
    }

    public void initializeButtons(Robot robot) {
        _operatorLeftBumper.whenPressed(new )
        _operatorRightBumper.whenPressed(new DropBall(robt));
    }

    public double getDriveSpeed() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return applySensitivityFactor(speed, Constants.DriveTrain.SPEED_SENSITIVITY);
    }

    public double getDriveRotation() {
        double speed = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return speed;
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {

    }
}

