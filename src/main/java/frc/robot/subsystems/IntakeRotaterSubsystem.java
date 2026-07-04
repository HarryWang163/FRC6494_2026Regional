package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Intaker rotation control for the 2026 fuel path.
 *
 * The normal match posture is lowered. During shooting, the arm alternates
 * between lowered and an upward assist position to help fuel enter the chamber.
 */
public class IntakeRotaterSubsystem extends SubsystemBase {
    private final TalonFX leftIntakeRotater;
    private final TalonFX rightIntakeRotater;

    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final Follower rightFollower;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("IntakeRotater");

    private double targetPosition = Constants.Intaker.intakeRotaterLoweredPosition;
    private double outputVoltage = 0.0;

    public IntakeRotaterSubsystem() {
        leftIntakeRotater = new TalonFX(Constants.Intaker.leftIntakeRotaterID);
        rightIntakeRotater = new TalonFX(Constants.Intaker.rightIntakeRotaterID);
        leftIntakeRotater.getConfigurator().apply(Constants.Intaker.intakeRotaterSlot0Configs);
        rightIntakeRotater.getConfigurator().apply(Constants.Intaker.intakeRotaterSlot0Configs);
        rightFollower = new Follower(leftIntakeRotater.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(50);
        followLeft();

        leftIntakeRotater.setPosition(0.0);
        rightIntakeRotater.setPosition(0.0);
    }

    public void lowerForMatch() {
        setPosition(Constants.Intaker.intakeRotaterLoweredPosition);
    }

    public Command lowerForMatchCommand() {
        return Commands.runOnce(this::lowerForMatch, this);
    }

    public void shootAssist() {
        double period = Constants.Intaker.intakeRotaterShootAssistPeriodSeconds;
        double phase = (Timer.getFPGATimestamp() % period) / period;
        double target = phase < Constants.Intaker.intakeRotaterShootAssistUpDutyCycle
            ? Constants.Intaker.intakeRotaterShootAssistUpPosition
            : Constants.Intaker.intakeRotaterLoweredPosition;
        setPosition(target);
    }

    public void manualRaise() {
        setVoltage(Constants.Intaker.intakeRotaterManualRaiseVoltage);
    }

    public void manualLower() {
        setVoltage(Constants.Intaker.intakeRotaterManualLowerVoltage);
    }

    public void stop() {
        setVoltage(0.0);
    }

    public boolean atGoal() {
        return Math.abs(getPosition() - targetPosition) <= Constants.Intaker.positionToleranceRotations;
    }

    public double getPosition() {
        return leftIntakeRotater.getPosition().getValueAsDouble();
    }

    private void setPosition(double rotations) {
        targetPosition = MathUtil.clamp(
            rotations,
            Constants.Intaker.intakeRotaterDownLimit,
            Constants.Intaker.intakeRotaterUpLimit
        );
        outputVoltage = 0.0;
        leftIntakeRotater.setControl(positionRequest.withPosition(targetPosition));
        followLeft();
    }

    private void setVoltage(double volts) {
        outputVoltage = MathUtil.clamp(volts, -12.0, 12.0);
        leftIntakeRotater.setControl(voltageRequest.withOutput(outputVoltage));
        followLeft();
    }

    private void followLeft() {
        rightIntakeRotater.setControl(rightFollower);
    }

    @Override
    public void periodic() {
        table.getEntry("position").setDouble(getPosition());
        table.getEntry("targetPosition").setDouble(targetPosition);
        table.getEntry("atGoal").setBoolean(atGoal());
        table.getEntry("outputVoltage").setDouble(outputVoltage);
    }
}
