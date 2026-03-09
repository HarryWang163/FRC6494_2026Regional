package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakerSubsystem extends SubsystemBase{
    //定义intakerotater&intakegetter
    private TalonFX intakerotater;
    private TalonFX intakegetter;

    //PID 
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
 
    private double intakeRotaterTargetPosition = 0.0;

    public double getGravityFFVolts(){
        double intakerotaterPosition = getIntakerotaterPosition();
        double gravityFFVolts = Constants.Intaker.IntakeRotaterGravityFF * intakerotaterPosition;
        return gravityFFVolts;
    }

    public void setIntakerRotaterVelocityRps(double IntakeRotaterTargetVelocity) {
        double gravityFeedforward = getGravityFFVolts();
        intakerotater.setControl(velocityRequest.withVelocity(IntakeRotaterTargetVelocity).withFeedForward(gravityFeedforward));// 重力前馈);
    } 

    public IntakerSubsystem() {
        //初始化rotater&getter
        intakegetter = new TalonFX(47);
        intakerotater = new TalonFX(48);
        intakegetter.getConfigurator().apply(Constants.Intaker.intakeGetterSlot0Configs);
        intakerotater.getConfigurator().apply(Constants.Intaker.intakeRotaterSlot0Configs);      
    }

    public void setIntakerRotaterPosition(double targetPosition) {
        //目标position
        if (targetPosition > Constants.Intaker.intakeRotaterUpLimit) {
            targetPosition = Constants.Intaker.intakeRotaterUpLimit;  // 限制最大上限
        } else if (targetPosition < Constants.Intaker.intakeRotaterDownLimit) {
            targetPosition = Constants.Intaker.intakeRotaterDownLimit;  // 限制最小下限
        }

        intakeRotaterTargetPosition = targetPosition;
        intakerotater.setControl(positionRequest.withPosition(targetPosition)); // 设置rotater位置（TalonFX 内部闭环 PID）
    }

    // 获取intakerotater当前角度
    public double getIntakerotaterPosition() {
        return intakerotater.getPosition().getValueAsDouble();
    }

    public boolean isIntakerotaterAtTarget() {
        // 到位判断容差
        double error = Math.abs(intakeRotaterTargetPosition - getIntakerotaterPosition());
        return error <= Constants.Intaker.positionToleranceRotations;
    }

     public void stopIntakerotater() {
        intakerotater.set(0);
        // 如果你希望停止后保持当前位置，建议改用中性模式/或维持 positionRequest（这句fromAI）
    }

    //intakegetter的on/off
    // public void setIntakeGetterOn(boolean on) {
    //     if (on) {
    //         intakegetter.setControl(velocityRequest.withVelocity(Constants.Intaker.IntakeGetterSpeed));
    //     }
    //     else {
    //         intakegetter.setControl(velocityRequest.withVelocity(0));
    //     }
    // }
    public void setIntakerGetterSpeed(double speed) {
        intakegetter.set(speed);
    }
    public void zeroEncoder(){
        intakerotater.setPosition(0);
        stopIntakerotater();
    }
    public void IntakerDownNonStop(){
        double currentPosition = getIntakerotaterPosition();
        if (currentPosition > Constants.Intaker.intakeRotaterDownLimit) {
            // move down (negative power) while above the lower limit
            intakerotater.set(-0.05);
        } else {
            // at or below lower limit: stop
            stopIntakerotater();
        }
    }

    public void IntakerUpNonStop(){
        double currentPosition = getIntakerotaterPosition();
        if (currentPosition < Constants.Intaker.intakeRotaterUpLimit) {
            // move up while below the upper limit
            intakerotater.set(0.3);
        } else {
            // at or above upper limit: stop
            stopIntakerotater();
            }
        }
    public void setIntakerRotater(double rotaterspeed){
        intakerotater.set(rotaterspeed);
    }
}
