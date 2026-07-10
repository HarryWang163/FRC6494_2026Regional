package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

/**
 * 主 conveyor 的开环控制封装。
 */
public class ConveyorSubsystem extends SubsystemBase {
    public final TalonFX mainConveyor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final NeutralOut neutralRequest = new NeutralOut();
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Conveyor");
    private double targetOutputVolts = 0.0;

    public ConveyorSubsystem() {
        mainConveyor = new TalonFX(Constants.Conveyor.mainConveyorID);
        mainConveyor.getConfigurator().apply(Constants.Conveyor.mainConveyorSlot0Configs);
    }
    public void feedToShooter() {
        setVoltage(Constants.Conveyor.feedVoltage);
    }
    public void reverse() {
        setVoltage(Constants.Conveyor.reverseVoltage);
    }
    public void stop() {
        targetOutputVolts = 0.0;
        mainConveyor.setControl(neutralRequest);
    }
    public void setVoltage(double outputVolts) {
        targetOutputVolts = MathUtil.clamp(outputVolts, -12.0, 12.0);
        mainConveyor.setControl(voltageRequest.withOutput(targetOutputVolts));
    }

    @Override
    public void periodic() {
        table.getEntry("velocity").setDouble(mainConveyor.getVelocity().getValueAsDouble());
        table.getEntry("targetOutputVolts").setDouble(targetOutputVolts);
    }
}
