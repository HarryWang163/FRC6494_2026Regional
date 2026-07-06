package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * 主 conveyor 的开环控制封装。
 */
public class ConveyorSubsystem extends SubsystemBase {
    public final TalonFX mainConveyor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Conveyor");
    private double targetSpeedRps = 0.0;

    public ConveyorSubsystem() {
        mainConveyor = new TalonFX(Constants.Conveyor.mainConveyorID);
        mainConveyor.getConfigurator().apply(Constants.Conveyor.mainConveyorSlot0Configs);
    }
    public void feedToShooter() {
        setSpeedByRPS(Constants.Conveyor.feedSpeedRps);
    }
    public void reverse() {
        setSpeedByRPS(Constants.Conveyor.reverseSpeedRps);
    }
    public void stop() {
        setSpeedByRPS(0.0);
    }
    public void setSpeedByRPS(double targetSpeedRps) {
        this.targetSpeedRps = targetSpeedRps;
        mainConveyor.setControl(velocityRequest.withVelocity(targetSpeedRps));
    }

    @Override
    public void periodic() {
        table.getEntry("velocity").setDouble(mainConveyor.getVelocity().getValueAsDouble());
        table.getEntry("targetSpeedRps").setDouble(targetSpeedRps);
    }
}
