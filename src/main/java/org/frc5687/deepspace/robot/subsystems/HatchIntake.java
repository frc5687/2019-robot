package org.frc5687.deepspace.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.intake.IdleHatchIntake;
import org.frc5687.deepspace.robot.utils.LimitSwitch;

public class HatchIntake extends OutliersSubsystem {


    private Robot _robot;
    private DoubleSolenoid _clawSolenoid;
    private DoubleSolenoid _wristSolenoid;
    private LimitSwitch _hatchDetectionLimit;

    public HatchIntake(Robot robot) {
        _robot = robot;
        _clawSolenoid = new DoubleSolenoid(RobotMap.PCM.CLAW_OPEN, RobotMap.PCM.CLAW_CLOSE);
        _wristSolenoid = new DoubleSolenoid(RobotMap.PCM.CLAW_WRIST_UP, RobotMap.PCM.CLAW_WRIST_DOWN);
        _hatchDetectionLimit = new LimitSwitch(RobotMap.DIO.HATCH_DETECTION_LIMIT);
    }

    @Override
    public void updateDashboard() {
        metric("Wrist", _wristSolenoid.get().name());
        metric("Claw", _clawSolenoid.get().name());
        metric("HatchDetected", isHatchDetected());
    }


    public void gripClaw(){
         _clawSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void pointClaw() {
        _clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void raiseWrist() { _wristSolenoid.set(DoubleSolenoid.Value.kReverse); }

    public void lowerWrist() { _wristSolenoid.set(DoubleSolenoid.Value.kForward); }

    public void releaseWrist() { _wristSolenoid.set(DoubleSolenoid.Value.kOff); }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new IdleHatchIntake(this));
    }

    public boolean isDown() {
        return _wristSolenoid.get() == DoubleSolenoid.Value.kForward;
    }

    public boolean isUp() {
        return _wristSolenoid.get() == DoubleSolenoid.Value.kReverse;
    }

    public boolean isHatchDetected() { return _hatchDetectionLimit.get(); }
    public boolean isPointed() {
            if(_clawSolenoid.get() == DoubleSolenoid.Value.kForward){
            return true;
        }
            return false;
    }
}

