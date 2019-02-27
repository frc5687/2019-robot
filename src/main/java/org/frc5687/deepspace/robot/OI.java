package org.frc5687.deepspace.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.frc5687.deepspace.robot.commands.*;
import org.frc5687.deepspace.robot.commands.intake.*;
import org.frc5687.deepspace.robot.subsystems.Elevator;
import org.frc5687.deepspace.robot.utils.*;
import org.frc5687.deepspace.robot.utils.AxisButton;
import org.frc5687.deepspace.robot.utils.Gamepad;
import org.frc5687.deepspace.robot.utils.OutliersProxy;
import org.frc5687.deepspace.robot.utils.POV;

import static org.frc5687.deepspace.robot.utils.Helpers.applyDeadband;
import static org.frc5687.deepspace.robot.utils.Helpers.applySensitivityFactor;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
    protected Launchpad _launchpad;
    private Button _operatorRightTrigger;
    private Button _operatorLeftTrigger;
    private Button _driverRightTrigger;
    private Button _driverLeftTrigger;


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


    public JoystickLight _hatchModeLEDLeft;
    public JoystickLight _cargoModeLEDLeft;
    public JoystickLight _hatchModeLEDRight;
    public JoystickLight _cargoModeLEDRight;
    public JoystickLight _targetCenterLED;
    public JoystickLight _targetLeftLED;
    public JoystickLight _targetRightLED;

    private Button _operatorUpButton;
    private Button _operatorDownButton;
    private Button _driverUpButton;
    private Button _driverDownButton;

    private AxisButton _operatorRightXAxisRightButton;
    private AxisButton _operatorRightXAxisLeftButton;

    private POV _operatorPOV;

    private boolean _cargoBlinking = false;
    private boolean _hatchBlinking = false;
    private int _tickNum;

    public OI(){
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
        _launchpad = new Launchpad(2);

        _operatorRightTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _operatorLeftTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _driverRightTrigger = new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(),Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _driverLeftTrigger = new AxisButton(_driverGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);

        _operatorRightBumper = new JoystickButton(_operatorGamepad, Gamepad.Buttons.RIGHT_BUMPER.getNumber());
        _operatorLeftBumper = new JoystickButton(_operatorGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());

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

        _operatorStartButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.START.getNumber());
        _operatorBackButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.BACK.getNumber());

        _driverStartButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.START.getNumber());
        _driverBackButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.BACK.getNumber());

        _hatchModeLEDLeft = new JoystickLight(_launchpad, Launchpad.LEDs.A.getNumber());
        _cargoModeLEDLeft = new JoystickLight(_launchpad, Launchpad.LEDs.B.getNumber());
        _hatchModeLEDRight = new JoystickLight(_launchpad, Launchpad.LEDs.F.getNumber());
        _cargoModeLEDRight = new JoystickLight(_launchpad, Launchpad.LEDs.G.getNumber());
        _targetCenterLED = new JoystickLight(_launchpad, Launchpad.LEDs.C.getNumber());
        _targetLeftLED = new JoystickLight(_launchpad, Launchpad.LEDs.D.getNumber());
        _targetRightLED = new JoystickLight(_launchpad, Launchpad.LEDs.E.getNumber());

        _operatorUpButton = new JoystickButton(_operatorGamepad, Gamepad.Axes.D_PAD_VERTICAL.getNumber());
        _operatorDownButton = new JoystickButton(_operatorGamepad, Gamepad.Axes.D_PAD_VERTICAL.getNumber());

        _driverUpButton = new JoystickButton(_driverGamepad, Gamepad.Axes.D_PAD_VERTICAL.getNumber());
        _driverDownButton = new JoystickButton(_driverGamepad, Gamepad.Axes.D_PAD_VERTICAL.getNumber());

        _operatorRightXAxisLeftButton = new AxisButton(_operatorGamepad,Gamepad.Axes.RIGHT_X.getNumber(), -.5);
        _operatorRightXAxisRightButton = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_X.getNumber(), .5);

        // _operatorPOV = new POV();
    }
    public void initializeButtons(Robot robot){


        _driverStartButton.whenPressed(new AutoClimb(robot.getStilt(), robot.getArm(), robot.getDriveTrain(), robot.getCargoIntake(), robot.getHatchIntake()));
//        _driverBackButton.whenPressed(new CloseSpear(robot.getSpear()));

        //_operatorStartButton.whenPressed(new StartGripper(robot.getGripper()));
        //_operatorBackButton.whenPressed(new StopGripper(robot.getGripper()));

//        _operatorStartButton.whenPressed(new CargoIntakeUp(robot,robot.getWrist()));
//        _operatorBackButton.whenReleased(new CargoIntakeDown(robot, robot.getWrist()));

        _operatorRightBumper.whenPressed(new ClawWristUp(robot));
        _operatorLeftBumper.whenPressed(new ClawWristDown(robot));

       // _driverRightBumper.whenPressed(new Shift(robot.getDriveTrain(), robot.getShifter(), Shifter.Gear.LOW, false));
        //
        // _driverLeftBumper.whenPressed(new Shift(robot.getDriveTrain(), robot.getShifter(), Shifter.Gear.HIGH, false));

//        _operatorRightTrigger.whenPressed(new Score(robot));
        _operatorRightTrigger.whenPressed(new IntakeCargo(robot));
        _operatorLeftTrigger.whileHeld(new HoldClawOpen(robot));

        _driverRightTrigger.whenPressed(new RunIntake(robot, robot.getCargoIntake()));
        _driverLeftTrigger.whenPressed(new StopRoller(robot.getCargoIntake()));


//        _operatorUpButton.whenPressed(new Manual(robot));
//        _operatorDownButton.whenPressed(new CancelAuto(robot));
//        _driverUpButton.whenPressed(new Manual(robot));
//        _driverDownButton.whenPressed(new CancelAuto(robot));

        _operatorRightXAxisLeftButton.whenPressed(new CargoIntakeDown(robot.getCargoIntake()));
        _operatorRightXAxisRightButton.whenPressed(new CargoIntakeUp(robot.getCargoIntake()));


        _operatorAButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch1, Elevator.MotionMode.Ramp));
        _operatorBButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch2, Elevator.MotionMode.Ramp));
        _operatorYButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch3, Elevator.MotionMode.Ramp));
        _operatorXButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.HPMode, Elevator.MotionMode.Ramp));

        //_driverYButton.whenPressed(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Floor, Arm.HallEffectSensor.LOW, Arm.MotionMode.Simple));
        //_driverBButton.whenPressed(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Intake, Arm.HallEffectSensor.INTAKE, Arm.MotionMode.Simple));
        //_driverXButton.whenPressed(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Secure, Arm.HallEffectSensor.SECURE, Arm.MotionMode.Simple));
        //_driverAButton.whenPressed(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Stowed, Arm.HallEffectSensor.STOWED, Arm.MotionMode.Simple));

    }
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

    public void setHatchLED(int flag) {
        switch (flag) {
            case 0:
                _hatchBlinking = false;
                _hatchModeLEDLeft.set(false);
                _hatchModeLEDRight.set(false);
                break;
            case 1:
                _hatchBlinking = false;
                _hatchModeLEDLeft.set(true);
                _hatchModeLEDRight.set(true);
                break;
            case 2:
                _hatchBlinking = true;
                break;
        }
    }

    public void setCargoLED(int flag) {
        switch (flag) {
            case 0:
                _cargoBlinking = false;
                _cargoModeLEDLeft.set(false);
                _cargoModeLEDRight.set(false);
                break;
            case 1:
                _cargoBlinking = false;
                _cargoModeLEDLeft.set(true);
                _cargoModeLEDRight.set(true);
                break;
            case 2:
                _cargoBlinking = true;
                break;
        }
    }





    @Override
    public void updateDashboard() {
        _tickNum++;
        if(_tickNum >= 5){
            _tickNum = 0;
            if(_cargoBlinking){
                _cargoModeLEDLeft.set(!_cargoModeLEDLeft.get());
                _cargoModeLEDRight.set(!_cargoModeLEDRight.get());
            }
            if(_hatchBlinking){
                _hatchModeLEDLeft.set(!_hatchModeLEDLeft.get());
                _hatchModeLEDRight.set(!_hatchModeLEDRight.get());
            }
        }
    }

    public boolean getAbort() {
        return false;
    }

}

