package org.frc5687.deepspace.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.frc5687.deepspace.robot.commands.*;
import org.frc5687.deepspace.robot.commands.intake.*;
import org.frc5687.deepspace.robot.subsystems.Elevator;
import org.frc5687.deepspace.robot.subsystems.Shifter;
import org.frc5687.deepspace.robot.utils.AxisButton;
import org.frc5687.deepspace.robot.utils.Gamepad;
import org.frc5687.deepspace.robot.utils.OutliersProxy;
import org.frc5687.deepspace.robot.utils.POV;

import static org.frc5687.deepspace.robot.utils.Helpers.applyDeadband;
import static org.frc5687.deepspace.robot.utils.Helpers.applySensitivityFactor;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
    private Button _operatorRightTrigger;
    private Button _operatorLeftTrigger;
    private Button _driverRightTrigger;
    private Button _driverLeftTrigger;

    private Button _driverRightStickButton;


    private Button _operatorAButton;
    private Button _operatorBButton;
    private Button _operatorYButton;
    private Button _operatorXButton;

    private Button _operatorRightBumper;
    private Button _operatorLeftBumper;

    private Button _driverAButton;
    private Button _driverBButton;
    private Button _driverXButton;
    private Button _driverYButton;

    private Button _driverRightBumper;
    private Button _driverLeftBumper;

    private Button _operatorStartButton;
    private Button _operatorBackButton;
    private Button _driverStartButton;
    private Button _driverBackButton;

    private Button _operatorUpButton;
    private Button _operatorDownButton;
    private Button _driverUpButton;
    private Button _driverDownButton;

    private AxisButton _operatorRightXAxisRightButton;
    private AxisButton _operatorRightXAxisLeftButton;

    private Button _operatorRightStickButton;

    private POV _operatorPOV;

    public OI(){
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);

        _operatorRightTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _operatorLeftTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _driverRightTrigger = new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(),Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _driverLeftTrigger = new AxisButton(_driverGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);

        _operatorRightBumper = new JoystickButton(_operatorGamepad, Gamepad.Buttons.RIGHT_BUMPER.getNumber());
        _operatorLeftBumper = new JoystickButton(_operatorGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());

        _operatorRightStickButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.RIGHT_STICK.getNumber());

        _driverRightBumper = new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_BUMPER.getNumber());
        _driverLeftBumper = new JoystickButton(_driverGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());

        _operatorAButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.A.getNumber());
        _operatorBButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.B.getNumber());
        _operatorYButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.Y.getNumber());
        _operatorXButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.X.getNumber());

        _driverAButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.A.getNumber());
        _driverBButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.B.getNumber());
        _driverXButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.X.getNumber());
        _driverYButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.Y.getNumber());

        _driverRightStickButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_STICK.getNumber());

        _operatorStartButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.START.getNumber());
        _operatorBackButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.BACK.getNumber());

        _driverStartButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.START.getNumber());
        _driverBackButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.BACK.getNumber());

        _operatorUpButton = new JoystickButton(_operatorGamepad, Gamepad.Axes.D_PAD_VERTICAL.getNumber());
        _operatorDownButton = new JoystickButton(_operatorGamepad, Gamepad.Axes.D_PAD_VERTICAL.getNumber());

        _driverUpButton = new JoystickButton(_driverGamepad, Gamepad.Axes.D_PAD_VERTICAL.getNumber());
        _driverDownButton = new JoystickButton(_driverGamepad, Gamepad.Axes.D_PAD_VERTICAL.getNumber());

        _operatorRightXAxisLeftButton = new AxisButton(_operatorGamepad,Gamepad.Axes.RIGHT_X.getNumber(), -.5);
        _operatorRightXAxisRightButton = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_X.getNumber(), .5);

        // _operatorPOV = new POV();
    }
    public void initializeButtons(Robot robot){


        _driverStartButton.whenPressed(new AutoClimb(robot.getStilt(), robot.getArm(), robot.getDriveTrain(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getStatusProxy(), true));
        _driverBackButton.whenPressed(new AutoClimb(robot.getStilt(), robot.getArm(), robot.getDriveTrain(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getStatusProxy(), false));

        _operatorBackButton.whenPressed(new ScoreStart(robot));

//        _operatorStartButton.whenPressed(new CargoIntakeUp(robot,robot.getWrist()));
//        _operatorBackButton.whenReleased(new CargoIntakeDown(robot, robot.getWrist()));

        //_operatorRightBumper.whenPressed(new ClawWristUp(robot));
        //_operatorLeftBumper.whenPressed(new ClawWristDown(robot));

        _operatorLeftBumper.whenPressed(new HatchMode(robot));
        _operatorRightBumper.whenPressed(new CargoMode(robot));

        _driverRightBumper.whenPressed(new Shift(robot.getDriveTrain(), robot.getShifter(), Shifter.Gear.LOW, false));
        _driverLeftBumper.whenPressed(new Shift(robot.getDriveTrain(), robot.getShifter(), Shifter.Gear.HIGH, false));

//        _operatorRightTrigger.whenPressed(new Score(robot));
        _operatorRightTrigger.whenPressed(new IntakeCargo(robot));
        _operatorLeftTrigger.whileHeld(new HoldClawOpen(robot));

        _driverRightTrigger.whenPressed(new AutoAlignToTarget(robot.getDriveTrain(), robot.getOI(), robot.getIMU(), robot.getLimelight(), 0.5, 5000, 2.0, ""));

        _operatorRightXAxisLeftButton.whenPressed(new CargoIntakeDown(robot.getCargoIntake()));
        _operatorRightXAxisRightButton.whenPressed(new CargoIntakeUp(robot.getCargoIntake()));


        _operatorAButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch1, Elevator.MotionMode.Ramp, this));
        _operatorBButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch2, Elevator.MotionMode.Ramp, this));
        _operatorYButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch3, Elevator.MotionMode.Ramp, this));
        _operatorXButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.HPMode, Elevator.MotionMode.Ramp, this));

        _operatorStartButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.StartHatch, Elevator.MotionMode.Ramp, this));
        _operatorRightStickButton.whenPressed(new Safeguard(robot, new StartingConfiguration(robot), -30));
//        _driverLeftTrigger.whenPressed(new StopRoller(robot.getCargoIntake()));
        //_driverYButton.whenPressed(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Floor, Arm.HallEffectSensor.LOW, Arm.MotionMode.Simple));
        //_driverBButton.whenPressed(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Intake, Arm.HallEffectSensor.INTAKE, Arm.MotionMode.Simple));
        //_driverXButton.whenPressed(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Secure, Arm.HallEffectSensor.SECURE, Arm.MotionMode.Simple));
        //_driverAButton.whenPressed(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Stowed, Arm.HallEffectSensor.STOWED, Arm.MotionMode.Simple));

    }

    public boolean isAutoTargetPressed() { return _driverRightTrigger.get(); }
    public double getDriveSpeed() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return speed;
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
        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_X.getNumber()) * Constants.Intake.MAX_ROLLER_SPEED;
        speed = applyDeadband(speed, Constants.Intake.DEADBAND);
        return applySensitivityFactor(speed, Constants.Intake.SENSITIVITY);
    }
    public double getElevatorSpeed() {
//        return 0;
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber()) * Constants.Elevator.MAX_SPEED;
        speed = applyDeadband(speed, Constants.Elevator.DEADBAND);
        return applySensitivityFactor(speed, Constants.Elevator.SENSITIVITY);
    }
    public double getStiltSpeed() {
        return 0;
//        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber())*Constants.Stilt.MAX_UP_SPEED;
//        speed = applyDeadband(speed, Constants.Stilt.DEADBAND);
//        return speed;
    }

    public double getWheelieSpeed() {
        return 0;
//        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber());
//        speed = applyDeadband(speed, Constants.Stilt.DEADBAND);
//        return speed;
    }


    public int getOperatorPOV() {
        return POV.fromWPILIbAngle(0, _operatorGamepad.getPOV()).getDirectionValue();
    }
    public int getDriverPOV() {
        return POV.fromWPILIbAngle(0, _driverGamepad.getPOV()).getDirectionValue();
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {

    }

    private int _driverRumbleCount = 0;
    private int _operatorRumbleCount = 0;
    private long _driverRumbleTime = System.currentTimeMillis();
    private long _operatorRumbleTime = System.currentTimeMillis();

    public void pulseDriver(int count) {
        _driverRumbleTime = System.currentTimeMillis() + Constants.OI.RUMBLE_PULSE_TIME;
        _driverRumbleCount = count * 2;
    }

    public void pulseOperator(int count) {
        _operatorRumbleTime = System.currentTimeMillis() + Constants.OI.RUMBLE_PULSE_TIME;
        _operatorRumbleCount = count * 2;
    }

    public void poll() {
        if (_driverRumbleCount > 0) {
            _driverGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, _driverRumbleCount % 2 == 0 ? 0 : 1);
            _driverGamepad.setRumble(GenericHID.RumbleType.kRightRumble, _driverRumbleCount % 2 == 0 ? 0 : 1);
            if (System.currentTimeMillis() > _driverRumbleTime) {
                _driverRumbleTime = System.currentTimeMillis() + Constants.OI.RUMBLE_PULSE_TIME;
                _driverRumbleCount--;
            }
        } else {
            _driverGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            _driverGamepad.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }

        if (_operatorRumbleCount > 0) {
            _operatorGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, _operatorRumbleCount % 2 == 0 ? 0 : 1);
            _operatorGamepad.setRumble(GenericHID.RumbleType.kRightRumble, _operatorRumbleCount % 2 == 0 ? 0 : 1);
            if (System.currentTimeMillis() > _operatorRumbleTime) {
                _operatorRumbleTime = System.currentTimeMillis() + Constants.OI.RUMBLE_PULSE_TIME;
                _operatorRumbleCount--;
            }
        } else {
            _operatorGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            _operatorGamepad.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }
    }

    public boolean isCreepPressed() {
        return  _driverRightStickButton.get();
    }
}

