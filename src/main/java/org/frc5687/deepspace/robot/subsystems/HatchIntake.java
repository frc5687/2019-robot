package org.frc5687.deepspace.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.intake.HoldClawOpen;
import org.frc5687.deepspace.robot.commands.intake.RunIntake;
import org.frc5687.deepspace.robot.utils.Helpers;

import static org.frc5687.deepspace.robot.Constants.Intake.*;

public class HatchIntake extends OutliersSubsystem {


    private Robot _robot;
    private DoubleSolenoid _clawSolenoid;
    private DoubleSolenoid _wristSolenoid;

    public HatchIntake(Robot robot) {
        _robot = robot;
        _clawSolenoid = new DoubleSolenoid(RobotMap.PCM.CLAW_OPEN, RobotMap.PCM.CLAW_CLOSE);
        _wristSolenoid = new DoubleSolenoid(RobotMap.PCM.CLAW_WRIST_UP, RobotMap.PCM.CLAW_WRIST_DOWN);
    }

    @Override
    public void updateDashboard() {
        metric("Wrist", _wristSolenoid.get().name());
        metric("Claw", _clawSolenoid.get().name());
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
        setDefaultCommand(new HoldClawOpen(_robot));
    }
}

