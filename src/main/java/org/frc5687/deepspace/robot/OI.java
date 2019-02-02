package org.frc5687.deepspace.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.frc5687.deepspace.robot.commands.CloseSpear;
import org.frc5687.deepspace.robot.commands.OpenSpear;
import org.frc5687.deepspace.robot.utils.Gamepad;
import org.frc5687.deepspace.robot.utils.OutliersProxy;

import static org.frc5687.deepspace.robot.utils.Helpers.applyDeadband;
import static org.frc5687.deepspace.robot.utils.Helpers.applySensitivityFactor;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;

    private Button _operatorLeftTrigger;
    private Button _operatorRightTrigger;

    public OI(){
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);


    }

    public void initializeButtons(Robot robot){
        _operatorLeftTrigger.whenPressed(new OpenSpear(robot.getSpear()));
        _operatorLeftTrigger.whenReleased(new CloseSpear(robot.getSpear()));
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

