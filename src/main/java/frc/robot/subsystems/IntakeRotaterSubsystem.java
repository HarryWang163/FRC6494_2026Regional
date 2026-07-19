package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
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
 * The rotater uses timed voltage actions only. Position is published for
 * telemetry, but it is not used for closed-loop control.
 */
public class IntakeRotaterSubsystem extends SubsystemBase {
    public final TalonFX leftIntakeRotater;
    private final TalonFX rightIntakeRotater;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final Follower rightFollower;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("IntakeRotater");

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
        setVoltage(0.05);
    }

    public Command lowerForMatchCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> setVoltage(-1.5), this),
            Commands.waitSeconds(0.7),
            Commands.runOnce(() -> setVoltage(0.05), this),
            Commands.waitSeconds(0.7),
            Commands.runOnce(this::stop, this)
        );
    }

    public void raiseForMatch() {
        setVoltage(0.8);
    }

    public Command raiseForMatchCommand() {
        return Commands.sequence(
            Commands.runOnce(this::raiseForMatch, this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(this::stop, this)
        );
    }

    public void shootAssist() {
        double cycleTime = Timer.getFPGATimestamp() % 1.7;

        if (cycleTime < 0.7) {
            setVoltage(1);
        } else if (cycleTime < 0.9) {
            setVoltage(0.0);
        } else {
            setVoltage(0.03);
        }
    }

    public void postShootIdleAssist(double elapsedSeconds) {
        double cycleTime = elapsedSeconds % 0.2;
        setVoltage(cycleTime < 0.1 ? 0.0 : 0.0);
    }

    public void manualRaise() {
        setVoltage(Constants.Intaker.intakeRotaterRaiseVoltage);
    }

    public void manualLower() {
        setVoltage(Constants.Intaker.intakeRotaterLowerVoltage);
    }

    public void stop() {
        setVoltage(0.0);
    }

    public boolean isRunning() {
        return Math.abs(outputVoltage) > 1e-6;
    }

    public double getPosition() {
        return leftIntakeRotater.getPosition().getValueAsDouble();
    }

    public double getLeftVelocity() {
        return leftIntakeRotater.getVelocity().getValueAsDouble();
    }

    public double getRightVelocity() {
        return rightIntakeRotater.getVelocity().getValueAsDouble();
    }

    private double getVelocityDifference() {
        return Math.abs(getLeftVelocity()) - Math.abs(getRightVelocity());
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
        table.getEntry("isRunning").setBoolean(isRunning());
        table.getEntry("outputVoltage").setDouble(outputVoltage);
        table.getEntry("leftVelocity").setDouble(getLeftVelocity());
        table.getEntry("rightVelocity").setDouble(getRightVelocity());
        table.getEntry("velocityDifference").setDouble(getVelocityDifference());
    }
}
