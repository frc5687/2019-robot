package org.frc5687.deepspace.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.frc5687.deepspace.robot.commands.*;
import org.frc5687.deepspace.robot.utils.AxisButton;
import org.frc5687.deepspace.robot.utils.Gamepad;
import org.frc5687.deepspace.robot.utils.OutliersProxy;

import static org.frc5687.deepspace.robot.utils.Helpers.applyDeadband;
import static org.frc5687.deepspace.robot.utils.Helpers.applySensitivityFactor;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
    private Button _operatorRightTrigger;
    private Button _operatorLeftTrigger;

    private Button _operatorYButton;
    private Button _operatorXButton;
    private Button _driverAButton;

    private Button _operatorLeftBumper;

    public OI(){
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
        _operatorRightTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _operatorLeftBumper = new JoystickButton(_operatorGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());
        _operatorLeftTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _operatorYButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.Y.getNumber());
        _operatorXButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.X.getNumber());
        _driverAButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.A.getNumber());
    }
    public void initializeButtons(Robot robot){
        if (robot.getGripper()!=null) {
            _operatorLeftBumper.whenPressed(new SuckBall(robot.getGripper()));
            _operatorLeftBumper.whenReleased(new DropBall(robot.getGripper()));
        }
        _operatorXButton.whenPressed(new WristDown(robot.getWrist()));
        _operatorYButton.whenPressed(new WristUp(robot.getWrist()));
        _operatorRightTrigger.whenPressed(new CloseSpear(robot.getSpear()));
        _operatorRightTrigger.whenReleased(new OpenSpear(robot.getSpear()));
        _operatorLeftTrigger.whileHeld(new DriveRoller(robot.getRoller()));
        _driverAButton.whenPressed(new AutoDriveToTarget(robot,.5,12,.5,""));
    }

    public boolean endIfPressed() { return _driverAButton.get(); }
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
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber()) * Constants.Arm.MAX_DRIVE_SPEED;
        speed = applyDeadband(speed, Constants.Arm.DEADBAND);
        return applySensitivityFactor(speed, Constants.Arm.SENSITIVITY);
    }
    public double getRollerSpeed() {
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber()) * Constants.Roller.MAX_SPEED;
        speed = applyDeadband(speed, Constants.Roller.DEADBAND);
        return applySensitivityFactor(speed, Constants.Roller.SENSITIVITY);
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

