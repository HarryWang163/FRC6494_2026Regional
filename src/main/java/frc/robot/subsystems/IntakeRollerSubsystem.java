package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

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

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final Follower rightFollower;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("IntakeRoller");
    private double targetSpeedRps = 0.0;

    public IntakeRollerSubsystem() {
        leftIntakeRoller = new TalonFX(Constants.Intaker.leftIntakeRollerID);
        rightIntakeRoller = new TalonFX(Constants.Intaker.rightIntakeRollerID);
        leftIntakeRoller.getConfigurator().apply(Constants.Intaker.intakeRollerSlot0Configs);
        rightIntakeRoller.getConfigurator().apply(Constants.Intaker.intakeRollerSlot0Configs);
        rightFollower = new Follower(leftIntakeRoller.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(50);
        followLeft();
    }

    public void intake() {
        setSpeedByRPS(Constants.Intaker.intakeRollerIntakeSpeedRps);
    }

    public void outtake() {
        setSpeedByRPS(Constants.Intaker.intakeRollerOuttakeSpeedRps);
    }

    public void stop() {
        setSpeedByRPS(0.0);
    }

    public void setSpeedByRPS(double targetSpeedRps) {
        this.targetSpeedRps = targetSpeedRps;
        leftIntakeRoller.setControl(velocityRequest.withVelocity(targetSpeedRps));
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
        table.getEntry("targetSpeedRps").setDouble(targetSpeedRps);
    }
}
