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
    private final TalonFX leftIntakeRoller;
    private final TalonFX rightIntakeRoller;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final Follower rightFollower;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("IntakeRoller");

    public IntakeRollerSubsystem() {
        leftIntakeRoller = new TalonFX(Constants.Intaker.leftIntakeRollerID);
        rightIntakeRoller = new TalonFX(Constants.Intaker.rightIntakeRollerID);
        rightFollower = new Follower(leftIntakeRoller.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(50);
        followLeft();
    }

    public void intake() {
        setVoltage(Constants.Intaker.intakeRollerVoltage);
    }

    public void outtake() {
        setVoltage(Constants.Intaker.outtakeRollerVoltage);
    }

    public void stop() {
        setVoltage(0);
    }

    public void setVoltage(double left) {
        leftIntakeRoller.setControl(voltageRequest.withOutput(left));
        followLeft();
    }

    private void followLeft() {
        rightIntakeRoller.setControl(rightFollower);
    }

    private double getVelocityDifference() {
    double leftVelocity = leftIntakeRoller.getVelocity().getValueAsDouble();
    double rightVelocity = rightIntakeRoller.getVelocity().getValueAsDouble();
    return Math.abs(leftVelocity) - Math.abs(rightVelocity);
    }

    @Override
    public void periodic() {
        table.getEntry("leftVelocity").setDouble(leftIntakeRoller.getVelocity().getValueAsDouble());
        table.getEntry("rightVelocity").setDouble(rightIntakeRoller.getVelocity().getValueAsDouble());
        table.getEntry("velocityDifference").setDouble(getVelocityDifference());
    }
}
