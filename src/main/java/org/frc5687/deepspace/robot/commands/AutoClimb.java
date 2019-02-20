package org.frc5687.deepspace.robot.commands;

import com.revrobotics.CANEncoder;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.subsystems.Roller;
import org.frc5687.deepspace.robot.subsystems.Stilt;

public class AutoClimb extends OutliersCommand {
    private Roller _roller;
    private Robot _robot;
    private Stilt _stilt;
    private DriveTrain _driveTrain;
    private ClimbState _climbState;
    private CANEncoder _encoder;
    private double _angleCos;
    private double _encoderOffset;
    private boolean isDone = false;

    public AutoClimb(Robot robot) {
        _robot = robot;
        _roller = _robot.getRoller();
        _stilt = _robot.getStilt();
        _driveTrain = _robot.getDriveTrain();
        _encoder = _roller.getRollerEncoder();
        requires(_stilt);
        requires(_roller);
    }

    @Override
    protected void initialize() {
       _encoderOffset = _encoder.getPosition();
       _climbState = ClimbState.MoveRoller;
       _driveTrain.enableBrakeMode();
    }

    @Override
    public void execute() {
        switch (_climbState) {
            case MoveRoller:
                if(_roller.getRollerEncoderPos() - _encoderOffset >= Constants.Roller.CLIMB_ANGLE_MAX){
                    _climbState = ClimbState.MoveRollerAndStilt;
                }
                _roller.setSpeed(0.5);
                break;
            case MoveRollerAndStilt:
                _angleCos = Math.cos(_roller.getRollerEncoderPos()) * Constants.Roller.CLIMB_SPEED_CONSTANT;
                _stilt.drive(0.5 );
                break;
            case Done:
                isDone = true;
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        return isDone;
    }

    enum ClimbState {
        MoveRoller(0),
        MoveRollerAndStilt(1),
        Done(2);

        private int _value;

        ClimbState(int value) { this._value = value; }

        public int getValue() { return _value; }
    }
}
