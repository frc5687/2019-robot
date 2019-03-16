package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.*;

import static org.frc5687.deepspace.robot.Constants.Auto.Climb.*;
import static org.frc5687.deepspace.robot.Constants.Arm.*;

public class AutoClimb extends OutliersCommand {
    private Robot _robot;

    private Stilt _stilt;
    private Arm _arm;
    private DriveTrain _driveTrain;

    private CargoIntake _cargoIntake;
    private HatchIntake _hatchIntake;

    private ClimbState _climbState;
    private long _stiltTimeout = 0;
    private boolean _highHab = true;

    private double _contactAngle;
    private double _slowAngle;
    private double _bottomAngle;

    private OI _oi;

    public AutoClimb(Stilt stilt, Arm arm, DriveTrain driveTrain, CargoIntake cargoIntake, HatchIntake hatchIntake, Robot robot, boolean highHab) {
        _stilt = stilt;
        _arm = arm;
        _driveTrain = driveTrain;

        _cargoIntake = cargoIntake;
        _hatchIntake = hatchIntake;

        _robot = robot;
        _oi = _robot.getOI();
        _highHab = highHab;

        requires(_stilt);
        requires(_arm);
        requires(_driveTrain);
        requires(_cargoIntake);
        requires(_hatchIntake);
    }

    @Override
    protected void initialize() {
        //if (DriverStation.getInstance().isFMSAttached() && DriverStation.getInstance().getMatchTime() > ENDGAME_CUTOFF) {
        //    _climbState = ClimbState.Done;
        //    error("Climb pressed before endgame");
        //}

       _climbState =  ClimbState.StowArm;
       _driveTrain.enableBrakeMode();
       _robot.setConfiguration(Robot.Configuration.climbing);

       if (_highHab) {
           _contactAngle = H3_CONTACT_ANGLE;
           _slowAngle = H3_SLOW_ANGLE;
           _bottomAngle = H3_BOTTOM_ANGLE;
       } else {
           _contactAngle = H2_CONTACT_ANGLE;
           _slowAngle = H2_SLOW_ANGLE;
           _bottomAngle = H2_BOTTOM_ANGLE;
       }

        metric("ClimbState", _climbState.name());

    }

    @Override
    public void execute() {
        switch (_climbState) {
            case StowArm:
                _cargoIntake.raiseWrist();
                _hatchIntake.pointClaw();
                _hatchIntake.lowerWrist();
                _arm.enableBrakeMode();
                _stilt.enableBrakeMode();
                _arm.setSpeed(STOW_SPEED);
                if (_arm.isLeftStowed() && _arm.isRightStowed()) {
                    error("Transitioning to " + ClimbState.PositionArm.name());
                    _climbState = ClimbState.PositionArm;
                }
                break;
            case PositionArm:
                _arm.setSpeed(INITIAL_ARM_SPEED);
                if (_arm.getAngle() >= _contactAngle) {
                    error("Transitioning to " + ClimbState.MoveRollerAndStilt.name());
                    _climbState = ClimbState.MoveRollerAndStilt;
                }
                break;
            case MoveRollerAndStilt:
                _stilt.setLifterSpeed(STILT_SPEED);
                metric("StiltSpeed", STILT_SPEED);
                double armSpeed =  _arm.getAngle() >= _slowAngle ? ARM_SLOW_SPEED : ARM_SPEED; // Math.cos(Math.toRadians(_arm.getAngle())) * ARM_SPEED_SCALAR;
                if ((_arm.isLow() || _arm.getAngle() >= _bottomAngle)) {
                    error("Stopping arm");
                    _arm.setSpeed(0);
                } else {
                    error("Running arm");
                    _arm.setSpeed(armSpeed);
                }

                metric("ArmSpeed", armSpeed);
                if (_oi.isWheelieForwardPressed() ||
                        ((_arm.isLow() || _arm.getAngle() >= _bottomAngle)
                        && (_highHab ?_stilt.isExtended() : _stilt.isAtMiddle()))) {
                    metric("StiltSpeed", 0);
                    metric("ArmSpeed", 0);
                    _arm.setSpeed(0);
                    _driveTrain.disableBrakeMode();
                    error("Transitioning to " + ClimbState.WheelieForward.name());
                    _climbState = ClimbState.WheelieForward;
                }
                break;
            case WheelieForward:
                _stilt.setLifterSpeed(_highHab ? STILT_HOLD_SPEED : 0);
                _stilt.setWheelieSpeed(WHEELIE_FORWARD_SPEED);
                _driveTrain.disableBrakeMode();
                _driveTrain.cheesyDrive(DRIVE_FORWARD_SPEED,0, false, false);
                metric("WheelieSpeed", WHEELIE_FORWARD_SPEED);
                metric("DriveSpeed", DRIVE_FORWARD_SPEED);
                metric("StiltSpeed", STILT_HOLD_SPEED);
                if (_stilt.isOnSurface()) {
                    metric("WheelieSpeed", 0);
                    metric("DriveSpeed", 0);
                    metric("StiltSpeed", 0);
                    _arm.enableCoastMode();
                    _stilt.setWheelieSpeed(0);
                    error("Transitioning to " + ClimbState.LiftStilt.name());
                    _climbState = ClimbState.LiftArm;
                }
                break;
            case LiftArm:
                _arm.setSpeed(RAISE_ARM_SPEED);
                if (_arm.getAngle() <= ARM_RETRACT_ANGLE) {
                    _arm.setSpeed(0);
                    _climbState = ClimbState.LiftStilt;
                    error("Transitioning to " + ClimbState.LiftStilt.name());
                    _stiltTimeout = System.currentTimeMillis() + STILT_TIMEOUT;
                }
                break;

            case LiftStilt:
                _stilt.setLifterSpeed(RAISE_STILT_SPEED);
                metric("StiltSpeed", RAISE_STILT_SPEED);
                if (_stilt.isRetracted()) {
                    _stilt.enableCoastMode();
                    _driveTrain.resetDriveEncoders();
                    error("Transitioning to " + ClimbState.Park.name());
                    _climbState = ClimbState.Park;
                }
                if (System.currentTimeMillis() >= _stiltTimeout) {
                    _stilt.setLifterSpeed(0);
                    _stilt.enableCoastMode();
                    error("Transitioning to " + ClimbState.WaitStilt.name());
                    _climbState = ClimbState.WaitStilt;
                }
                break;
            case WaitStilt:
                _stilt.setLifterSpeed(0);
                metric("StiltSpeed", 0);
                if (_stilt.isRetracted()) {
                    _stilt.enableCoastMode();
                    _driveTrain.resetDriveEncoders();
                    _climbState = ClimbState.Park;
                    error("Transitioning to " + ClimbState.Park.name());
                }
                break;

            case Park:
                metric("DriveSpeed", PARK_SPEED);
                _driveTrain.cheesyDrive(PARK_SPEED, 0, false, false);
                if (_driveTrain.getDistance() > PARK_DISTANCE) {
                    metric("DriveSpeed", 0);
                    _driveTrain.cheesyDrive(0.0,0, false, false);
                    error("Transitioning to " + ClimbState.Done.name());
                    _climbState = ClimbState.Done;
                }
                break;
            case Done:
                _robot.setConfiguration(Robot.Configuration.parked);
                _driveTrain.enableBrakeMode();
                _stilt.setLifterSpeed(0);
                _arm.setSpeed(0);
                break;
        }
        metric("ClimbState", _climbState.name());
    }

    @Override
    protected void end() {
        // _stilt.enableCoastMode();
        // _arm.enableCoastMode();
    }

    @Override
    protected boolean isFinished() {
        return _climbState==ClimbState.Done;
    }

    enum ClimbState {
        StowArm(0),
        PositionArm(1),
        MoveRollerAndStilt(2),
        WheelieForward(3),
        LiftArm(4),
        LiftStilt(5),
        WaitStilt(6),
        Park(7),
        Done(8);

        private int _value;

        ClimbState(int value) { this._value = value; }

        public int getValue() { return _value; }
    }
}
