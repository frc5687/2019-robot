package org.frc5687.deepspace.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.frc5687.deepspace.robot.commands.*;
import org.frc5687.deepspace.robot.subsystems.Arm;
import org.frc5687.deepspace.robot.subsystems.Elevator;
import org.frc5687.deepspace.robot.subsystems.Shifter;
import org.frc5687.deepspace.robot.utils.*;

import static org.frc5687.deepspace.robot.utils.Helpers.applyDeadband;
import static org.frc5687.deepspace.robot.utils.Helpers.applySensitivityFactor;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
    protected Launchpad _launchpad;
    private Button _operatorRightTrigger;
    private Button _operatorLeftTrigger;

    private Button _operatorAButton;
    private Button _operatorBButton;
    private Button _operatorYButton;
    private Button _operatorXButton;

    private Button _operatorLeftBumper;
    private Button _operatorRightBumper;

    private Button _driverAButton;
    private Button _driverBButton;
    private Button _driverXButton;
    private Button _driverYButton;

    private Button _driverLeftBumper;
    private Button _driverRightBumper;

    private Button _operatorStartButton;
    private Button _operatorBackButton;
    private Button _driverStartButton;
    private Button _driverBackButton;

    public JoystickLight _testLight;

    public OI(){
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
        _launchpad = new Launchpad(2);

        _operatorLeftTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _operatorRightTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);

        _operatorLeftBumper = new JoystickButton(_operatorGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());
        _operatorRightBumper = new JoystickButton(_operatorGamepad, Gamepad.Buttons.RIGHT_BUMPER.getNumber());

        _driverLeftBumper = new JoystickButton(_driverGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());
        _driverRightBumper = new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_BUMPER.getNumber());

        _operatorAButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.A.getNumber());
        _operatorBButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.B.getNumber());
        _operatorYButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.Y.getNumber());
        _operatorXButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.X.getNumber());
        

        _driverAButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.A.getNumber());
        _driverBButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.B.getNumber());
        _driverXButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.X.getNumber());
        _driverYButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.Y.getNumber());

        _operatorStartButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.START.getNumber());
        _operatorBackButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.BACK.getNumber());

        _driverStartButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.START.getNumber());
        _driverBackButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.BACK.getNumber());
        _testLight = new JoystickLight(_launchpad, 1);

    }
    public void initializeButtons(Robot robot){
        _driverStartButton.whenPressed(new RunGripper(robot.getGripper()));
        _driverBackButton.whenReleased(new StopGripper(robot.getGripper()));

        _operatorStartButton.whenPressed(new CloseSpear(robot.getSpear()));
        _operatorBackButton.whenReleased(new OpenSpear(robot.getSpear()));

        _operatorLeftBumper.whenPressed(new WristDown(robot.getWrist()));
        _operatorRightBumper.whenPressed(new WristUp(robot.getWrist()));

        _driverLeftBumper.whenPressed(new Shift(robot.getDriveTrain(), robot.getShifter(), Shifter.Gear.HIGH, false));
        _driverRightBumper.whenPressed(new Shift(robot.getDriveTrain(), robot.getShifter(), Shifter.Gear.LOW, false));


        _operatorAButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch1, Elevator.MotionMode.PID));
        _operatorBButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch2, Elevator.MotionMode.PID));
        _operatorYButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Top, Elevator.MotionMode.PID));
        _operatorXButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.PID));

        _driverAButton.whenPressed(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Floor, Arm.HallEffectSensor.LOW, Arm.MotionMode.Simple));
        _driverBButton.whenPressed(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Intake, Arm.HallEffectSensor.INTAKE, Arm.MotionMode.Simple));
        _driverXButton.whenPressed(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Secure, Arm.HallEffectSensor.SECURE, Arm.MotionMode.Simple));
        _driverYButton.whenPressed(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Stowed, Arm.HallEffectSensor.STOWED, Arm.MotionMode.Simple));

        _testLight.set(true);
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
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber()) * Constants.Arm.MAX_DRIVE_SPEED;
        speed = applyDeadband(speed, Constants.Arm.DEADBAND);
        return applySensitivityFactor(speed, Constants.Arm.SENSITIVITY);
    }
    public double getRollerSpeed() {
        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber()) * Constants.Roller.MAX_SPEED;
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

