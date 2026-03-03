package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX climberMotor;
    private double targetPositionTicks;

    private final PositionVoltage positionHoldRequest =
            new PositionVoltage(0).withSlot(0);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    private double holdPositionTicks = 0.0;

    private final NetworkTable climberTable;
    

    public ClimberSubsystem() {
        climberMotor = new TalonFX(Constants.Climber.motorID);

        climberMotor.getConfigurator().apply(Constants.Climber.slot0Configs);

        var cfg = new MotorOutputConfigs();
        cfg.Inverted = Constants.Climber.inverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        climberMotor.getConfigurator().apply(cfg);

        climberTable = NetworkTableInstance.getDefault().getTable("Climber");

        holdPositionTicks = getPositionTicks();
    }
    public void initialize() {
        climberMotor.set(-0.07);
    }
    
    public void zeroEncoder(){
        climberMotor.setPosition(0);
        stopMotor();
    }

    public double getPositionTicks() {
        return climberMotor.getPosition().getValueAsDouble();
    }

    public double getVelocityRps() {
        return climberMotor.getVelocity().getValueAsDouble();
    }

    public void climbUp() {
        targetPositionTicks = Constants.Climber.climbUpTicks;  
        climberMotor.setControl(positionHoldRequest.withPosition(targetPositionTicks));
    }

    public void climbDown() {
        targetPositionTicks = Constants.Climber.climbDownTicks;  
        climberMotor.setControl(positionHoldRequest.withPosition(targetPositionTicks));
    }

    public void climbDownNonStop(){
        climberMotor.setControl(velocityRequest.withVelocity(Constants.Climber.climbVelocity));
    }
    
    public void climbUpNonStop(){
        climberMotor.setControl(velocityRequest.withVelocity(-Constants.Climber.climbVelocity));
    }

    public void captureHoldPosition() {
        holdPositionTicks = getPositionTicks();
    }

    public void holdPosition() {
        climberMotor.setControl(
                positionHoldRequest.withPosition(holdPositionTicks)
        );
    }

    public void stopMotor() {
        climberMotor.set(0);
    }

    @Override
    public void periodic() {
        climberTable.getEntry("velocityRps").setDouble(getVelocityRps());
        climberTable.getEntry("positionTicks").setDouble(getPositionTicks());
        climberTable.getEntry("holdPositionTicks").setDouble(holdPositionTicks);
    }
}
