package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
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
    private final NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("Tuning/mainConveyor");
    private double targetOutputVolts = 0.0;
    private double commandedDebugVolts = Constants.Conveyor.feedVoltage;
    private double clampedDebugVolts = Constants.Conveyor.feedVoltage;
    private boolean voltageTuningActive = false;

    public ConveyorSubsystem() {
        mainConveyor = new TalonFX(Constants.Conveyor.mainConveyorID);
        mainConveyor.getConfigurator().apply(Constants.Conveyor.mainConveyorSlot0Configs);
        var mainConveyorMotorOutputConfigs = new MotorOutputConfigs();
        mainConveyorMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        mainConveyor.getConfigurator().apply(mainConveyorMotorOutputConfigs);
        tuningTable.getEntry("mainConveyorVoltageEnable").setDefaultBoolean(false);
        tuningTable.getEntry("mainConveyorTargetVolts").setDefaultDouble(Constants.Conveyor.feedVoltage);
    }
    public void feedToShooter() {
        setVoltage(Constants.Conveyor.feedVoltage);
    }
    public void feedForIntake() {
        setVoltage(Constants.Conveyor.intakeFeedVoltage);
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

    public boolean isMainConveyorTuningControlActive() {
        return DriverStation.isTestEnabled()
            && tuningTable.getEntry("mainConveyorVoltageEnable").getBoolean(false);
    }

    public void debugMainConveyorVoltagePeriodic(boolean allowMotorOutput) {
        voltageTuningActive =
            allowMotorOutput
                && DriverStation.isTestEnabled()
                && tuningTable.getEntry("mainConveyorVoltageEnable").getBoolean(false);
        commandedDebugVolts = tuningTable.getEntry("mainConveyorTargetVolts")
            .getDouble(Constants.Conveyor.feedVoltage);
        clampedDebugVolts = MathUtil.clamp(commandedDebugVolts, -12.0, 12.0);

        tuningTable.getEntry("mainConveyorVoltageEnableActive").setBoolean(voltageTuningActive);
        tuningTable.getEntry("mainConveyorCommandedTargetVolts").setDouble(commandedDebugVolts);
        tuningTable.getEntry("mainConveyorClampedTargetVolts").setDouble(clampedDebugVolts);

        if (voltageTuningActive) {
            setVoltage(clampedDebugVolts);
        }
    }

    @Override
    public void periodic() {
        table.getEntry("velocity").setDouble(mainConveyor.getVelocity().getValueAsDouble());
        table.getEntry("targetOutputVolts").setDouble(targetOutputVolts);
        table.getEntry("motorOutputVolts").setDouble(mainConveyor.getMotorVoltage().getValueAsDouble());
        table.getEntry("statorCurrent").setDouble(mainConveyor.getStatorCurrent().getValueAsDouble());
        table.getEntry("voltageTuningActive").setBoolean(voltageTuningActive);
        table.getEntry("commandedDebugVolts").setDouble(commandedDebugVolts);
        table.getEntry("clampedDebugVolts").setDouble(clampedDebugVolts);
    }
}
