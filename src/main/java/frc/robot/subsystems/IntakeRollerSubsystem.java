package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * 进球滚轮的硬件封装。
 *
 * 何时 intake、outtake、hold 或 stop 由 Superstructure 决定；
 * 本类只负责成对 TalonFX 输出和 follower 关系。
 */
public class IntakeRollerSubsystem extends SubsystemBase {
    public final TalonFX leftIntakeRoller;
    public final TalonFX rightIntakeRoller;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final Follower rightFollower;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("IntakeRoller");

    private double leftVoltage = 0.0;
    private double rightVoltage = 0.0;

    public IntakeRollerSubsystem() {
        leftIntakeRoller = new TalonFX(Constants.Intaker.leftIntakeRollerID);
        rightIntakeRoller = new TalonFX(Constants.Intaker.rightIntakeRollerID);
        rightFollower = new Follower(leftIntakeRoller.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(50);
        followLeft();
    }

    public void intake() {
        setVoltage(Constants.Intaker.intakeRollerVoltage, -Constants.Intaker.intakeRollerVoltage);
    }

    public void outtake() {
        setVoltage(Constants.Intaker.outtakeRollerVoltage, -Constants.Intaker.outtakeRollerVoltage);
    }

    public void hold() {
        setVoltage(Constants.Intaker.holdRollerVoltage, -Constants.Intaker.holdRollerVoltage);
    }

    public void stop() {
        setVoltage(0.0, 0.0);
    }

    public void setPercent(double left, double right) {
        setVoltage(left * 12.0, right * 12.0);
    }

    public void setVoltage(double left, double right) {
        // rightVoltage 仅用于 dashboard 显示；真实硬件通过反向 follower 跟随左侧电机。
        leftVoltage = MathUtil.clamp(left, -12.0, 12.0);
        rightVoltage = MathUtil.clamp(right, -12.0, 12.0);
        leftIntakeRoller.setControl(voltageRequest.withOutput(leftVoltage));
        followLeft();
    }

    private void followLeft() {
        rightIntakeRoller.setControl(rightFollower);
    }

    @Override
    public void periodic() {
        table.getEntry("leftVoltage").setDouble(leftVoltage);
        table.getEntry("rightVoltage").setDouble(rightVoltage);
        table.getEntry("leftVelocity").setDouble(leftIntakeRoller.getVelocity().getValueAsDouble());
        table.getEntry("rightVelocity").setDouble(rightIntakeRoller.getVelocity().getValueAsDouble());
    }
}
