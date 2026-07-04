package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * 进球旋转臂的位置闭环和开环控制。
 *
 * 机构使用 TalonFX 集成编码器。归零前只允许 SAFE 或受限手动移动，
 * 避免相对编码器未校准时发生大幅度意外旋转。
 */
public class IntakeRotaterSubsystem extends SubsystemBase {
    public enum IntakePosition {
        STOW(Constants.Intaker.intakeRotaterStowPosition),
        GROUND_INTAKE(Constants.Intaker.intakeRotaterGroundPosition),
        HANDOFF(Constants.Intaker.intakeRotaterHandoffPosition),
        SAFE(Constants.Intaker.intakeRotaterSafePosition);

        public final double rotations;

        IntakePosition(double rotations) {
            this.rotations = rotations;
        }
    }

    public final TalonFX leftIntakeRotater;
    public final TalonFX rightIntakeRotater;

    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final Follower rightFollower;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("IntakeRotater");

    private IntakePosition goal = IntakePosition.SAFE;
    private boolean zeroed = false;
    private double manualVoltage = 0.0;

    public IntakeRotaterSubsystem() {
        leftIntakeRotater = new TalonFX(Constants.Intaker.leftIntakeRotaterID);
        rightIntakeRotater = new TalonFX(Constants.Intaker.rightIntakeRotaterID);
        leftIntakeRotater.getConfigurator().apply(Constants.Intaker.intakeRotaterSlot0Configs);
        rightIntakeRotater.getConfigurator().apply(Constants.Intaker.intakeRotaterSlot0Configs);
        rightFollower = new Follower(leftIntakeRotater.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(50);
        followLeft();
    }

    public void setGoal(IntakePosition position) {
        // 相对编码器没有对准机械零点前，禁止执行大幅度位置闭环移动。
        goal = position;
        if (!zeroed && position != IntakePosition.SAFE) {
            stop();
            return;
        }

        double target = MathUtil.clamp(
            position.rotations,
            Constants.Intaker.intakeRotaterDownLimit,
            Constants.Intaker.intakeRotaterUpLimit
        );
        leftIntakeRotater.setControl(positionRequest.withPosition(target));
        manualVoltage = 0.0;
        followLeft();
    }

    public void stow() {
        setGoal(IntakePosition.STOW);
    }

    public void deployToGround() {
        setGoal(IntakePosition.GROUND_INTAKE);
    }

    public void moveToHandoff() {
        setGoal(IntakePosition.HANDOFF);
    }

    public boolean atGoal() {
        return Math.abs(getPosition() - goal.rotations) <= Constants.Intaker.positionToleranceRotations;
    }

    public void zeroPosition() {
        // 只有 intake 物理上处于已知零点姿态时才调用此方法。
        leftIntakeRotater.setPosition(0.0);
        rightIntakeRotater.setPosition(0.0);
        zeroed = true;
        goal = IntakePosition.STOW;
        stow();
    }

    public boolean isZeroed() {
        return zeroed;
    }

    public void clearZeroed() {
        zeroed = false;
        goal = IntakePosition.SAFE;
        stop();
    }

    public double getPosition() {
        return leftIntakeRotater.getPosition().getValueAsDouble();
    }

    public IntakePosition getGoal() {
        return goal;
    }

    public void setOpenLoopVoltage(double voltage) {
        // 归零前允许手动小幅移动，但输出会被限制到较温和的范围。
        if (!zeroed && Math.abs(voltage) > Constants.Intaker.intakeRotaterManualVoltage) {
            voltage = Math.copySign(Constants.Intaker.intakeRotaterManualVoltage, voltage);
        }
        manualVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        leftIntakeRotater.setControl(voltageRequest.withOutput(manualVoltage));
        followLeft();
    }

    public void stop() {
        manualVoltage = 0.0;
        leftIntakeRotater.setControl(voltageRequest.withOutput(0.0));
        followLeft();
    }

    private void followLeft() {
        rightIntakeRotater.setControl(rightFollower);
    }

    @Override
    public void periodic() {
        table.getEntry("goal").setString(goal.name());
        table.getEntry("zeroed").setBoolean(zeroed);
        table.getEntry("position").setDouble(getPosition());
        table.getEntry("atGoal").setBoolean(atGoal());
        table.getEntry("manualVoltage").setDouble(manualVoltage);
    }
}
