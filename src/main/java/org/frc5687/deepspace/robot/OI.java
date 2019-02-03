package org.frc5687.deepspace.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.frc5687.deepspace.robot.commands.CloseSpear;
import org.frc5687.deepspace.robot.commands.OpenSpear;
import org.frc5687.deepspace.robot.commands.GobblerIntake;
import org.frc5687.deepspace.robot.utils.AxisButton;
import org.frc5687.deepspace.robot.commands.WristDown;
import org.frc5687.deepspace.robot.commands.WristUp;
import org.frc5687.deepspace.robot.utils.Gamepad;
import org.frc5687.deepspace.robot.utils.OutliersProxy;

import static org.frc5687.deepspace.robot.utils.Helpers.applyDeadband;
import static org.frc5687.deepspace.robot.utils.Helpers.applySensitivityFactor;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
    private Button _operatorRightTrigger;

    private Button _operatorYButton;
    private Button _operatorXButton;

    public OI(){
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
        _operatorRightTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _operatorYButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.Y.getNumber());
        _operatorXButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.X.getNumber());
    }

    public void initializeButtons(Robot robot){
        _operatorXButton.whenPressed(new WristDown(robot.getWrist()));
        _operatorYButton.whenPressed(new WristUp(robot.getWrist()));
        _operatorRightTrigger.whenPressed(new CloseSpear(robot.getSpear()));
        _operatorRightTrigger.whenReleased(new OpenSpear(robot.getSpear()));

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
    public double getArmSpeed() {
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber()) * Constants.Gobbler.MAX_INTAKE_SPEED;
        speed = applyDeadband(speed, Constants.Gobbler.ARM_DEADBAND);
        return applySensitivityFactor(speed, Constants.Gobbler.ARM_SENSITIVITY);
    }
    public double getRollerSpeed() {
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber()) * Constants.Gobbler.MAX_ROLLER_SPEED;
        speed = applyDeadband(speed, Constants.Gobbler.ROLLER_DEADBAND);
        return applySensitivityFactor(speed, Constants.Gobbler.ROLLER_SENSITIVITY);
    }
    public double getElevatorSpeed() {
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber()) * Constants.Elevator.MAX_ELEVATOR_SPEED;
        speed = applyDeadband(speed, Constants.Elevator.DEADBAND);
        return applySensitivityFactor(speed, Constants.Elevator.SENSITIVITY);
    }


    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {

    }
}

