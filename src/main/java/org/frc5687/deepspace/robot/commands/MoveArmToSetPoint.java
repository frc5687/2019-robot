package org.frc5687.deepspace.robot.commands;


import org.frc5687.deepspace.robot.subsystems.Arm;

enum HallEffectSensor {
    LOW,
    INTAKE,
    SECURE,
    STOWED
}

public class MoveArmToSetPoint extends OutliersCommand {
    private Arm _arm;
    private double _setPoint;
    private HallEffectSensor _hallEffectSensor;
    public MoveArmToSetPoint(Arm arm, HallEffectSensor hallEffectSensor, double setPoint){
        _arm = arm;
        _hallEffectSensor = hallEffectSensor;
        _setPoint = setPoint;
        requires(_arm);
    }

    @Override
    protected void initialize(){
        _arm.setSetpoint(_setPoint);
    }

    @Override
    protected boolean isFinished() {
        switch (_hallEffectSensor){
            case LOW:
                if(_arm.isLow()){
                    return true;
                }
                break;
            case INTAKE:
                if(_arm.isIntake()){
                    return true;
                }
                break;
            case SECURE:
                if(_arm.isSecured()){
                    return true;
                }
                break;
            case STOWED:
                if(_arm.isStowed()){
                    return true;
                }
                break;
        }
        return _arm.getPIDController().onTarget();
    }
    
    @Override
    protected void end(){

    }
}
