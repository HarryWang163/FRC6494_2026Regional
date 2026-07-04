package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * 主 conveyor 的开环控制封装。
 *
 * 本赛季没有球检测传感器，因此这里仅报告电机状态；
 * 所有动作顺序判断都放在 Superstructure。
 */
public class ConveyorSubsystem extends SubsystemBase {
    public final TalonFX mainConveyor;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Conveyor");

    private double voltage = 0.0;

    public ConveyorSubsystem() {
        mainConveyor = new TalonFX(Constants.Conveyor.mainConveyorID);
    }

    public void feedToShooter() {
        setVoltage(Constants.Conveyor.feedVoltage);
    }

    public void reverse() {
        setVoltage(Constants.Conveyor.reverseVoltage);
    }

    public void hold() {
        setVoltage(Constants.Conveyor.holdVoltage);
    }

    public void stop() {
        setVoltage(0.0);
    }

    public void setPercent(double percent) {
        setVoltage(percent * 12.0);
    }

    public void setVoltage(double volts) {
        voltage = MathUtil.clamp(volts, -12.0, 12.0);
        mainConveyor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void periodic() {
        table.getEntry("voltage").setDouble(voltage);
        table.getEntry("velocity").setDouble(mainConveyor.getVelocity().getValueAsDouble());
    }
}
