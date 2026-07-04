package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
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
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Conveyor");

    public ConveyorSubsystem() {
        mainConveyor = new TalonFX(Constants.Conveyor.mainConveyorID);
    }
    public void feedToShooter() {
        setVoltage(Constants.Conveyor.feedVoltage);
    }
    public void reverse() {
        setVoltage(Constants.Conveyor.reverseVoltage);
    }
    public void stop() {
        setVoltage(0.0);
    }
    public void setVoltage(double volts) {
        mainConveyor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void periodic() {
        table.getEntry("velocity").setDouble(mainConveyor.getVelocity().getValueAsDouble());
    }
}
