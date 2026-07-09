package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Shooter 总成的硬件封装。
 *
 * 飞轮和 shooter 内部 conveyor 使用 VoltageOut 开环电压输出；
 * shooter 内部 conveyor 仍由右侧电机反向 follower 跟随左侧电机。
 * 背板控制沿用旧代码中的电机、编码器和 PID 调参流程。
 * 何时 spin up、何时喂球由 Superstructure 决定，本类不做比赛动作判断。
 */
public class ShooterSubsystem extends SubsystemBase {
    public final TalonFX leftFlywheel;
    public final TalonFX rightFlywheel;
    public final TalonFX leftConveyor;
    public final TalonFX rightConveyor;

    public final TalonFXS backboardMotor;
    public final Encoder backboardEncoder;
    public final ProfiledPIDController backboardPID;

    private final VoltageOut leftFlywheelVoltageRequest = new VoltageOut(0);
    private final VoltageOut rightFlywheelVoltageRequest = new VoltageOut(0);
    private final VoltageOut conveyorVoltageRequest = new VoltageOut(0);
    private final VoltageOut backboardVoltageRequest = new VoltageOut(0);

    private final Follower conveyorFollower;

    private final NetworkTable shooterNetworkTable;
    private final NetworkTable backboardTuningTable;

    private double flywheelOutputVolts = 0.0;
    private double flywheelVoltageStartTimestamp = 0.0;
    private double shooterConveyorOutputVolts = 0.0;
    private double backboardOutputVolts = 0.0;
    private double backboardRawPidOutputVolts = 0.0;
    private double backboardGravityFeedforwardVolts = 0.0;
    private double backboardClampedPidOutputVolts = 0.0;
    private boolean backboardOutputSaturated = false;
    private boolean backboardMinimumOutputActive = false;

    public ShooterSubsystem() {
        leftFlywheel = new TalonFX(Constants.Shooter.leftFlywheelID);
        rightFlywheel = new TalonFX(Constants.Shooter.rightFlywheelID);
        leftConveyor = new TalonFX(Constants.Shooter.leftConveyorID);
        rightConveyor = new TalonFX(Constants.Shooter.rightConveyorID);

        leftFlywheel.getConfigurator().apply(Constants.Shooter.flyWheelSlot0Configs);
        rightFlywheel.getConfigurator().apply(Constants.Shooter.flyWheelSlot0Configs);
        leftConveyor.getConfigurator().apply(Constants.Shooter.conveyorSlot0Configs);
        rightConveyor.getConfigurator().apply(Constants.Shooter.conveyorSlot0Configs);

        var motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        leftFlywheel.getConfigurator().apply(motorOutputConfigs);
        leftConveyor.getConfigurator().apply(motorOutputConfigs);

        conveyorFollower = new Follower(leftConveyor.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(50);
        setRightConveyorFollowLeft();

        backboardMotor = new TalonFXS(Constants.Shooter.backboardMotorID);
        backboardMotor.getConfigurator().apply(Constants.Shooter.backboardSlot0Configs);

        backboardEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        backboardEncoder.setSamplesToAverage(5);
        backboardEncoder.setMinRate(1.0);
        backboardEncoder.setDistancePerPulse(1);

        backboardPID = new ProfiledPIDController(
            Constants.Shooter.backboardPositionPID.kP,
            Constants.Shooter.backboardPositionPID.kI,
            Constants.Shooter.backboardPositionPID.kD,
            new Constraints(10000, 100000)
        );
        backboardPID.setTolerance(10.0);

        shooterNetworkTable = NetworkTableInstance.getDefault().getTable("Shooter");
        backboardTuningTable = NetworkTableInstance.getDefault().getTable("Tuning/backboard");
        backboardTuningTable.getEntry("backboardPositionEnable").setDefaultBoolean(false);
        backboardTuningTable.getEntry("backboardTargetPosition").setDefaultDouble(0.0);
    }

    /* ====================== */
    /*         飞轮            */
    /* ====================== */

    public void setRightConveyorFollowLeft() {
        // 成对机构机械方向相反，因此右侧电机以反向 follower 跟随左侧电机。
        rightConveyor.setControl(conveyorFollower);
    }

    public void applyLeftConfigurationToRight() {
        Slot0Configs configs = new Slot0Configs();
        leftFlywheel.getConfigurator().refresh(configs);
        rightFlywheel.getConfigurator().apply(configs);
    }

    public void setFlywheelVoltage(double outputVolts) {
        double clampedOutputVolts = MathUtil.clamp(outputVolts, -12.0, 12.0);
        if (Math.abs(clampedOutputVolts - flywheelOutputVolts) > 1e-6) {
            flywheelVoltageStartTimestamp = Timer.getFPGATimestamp();
        }
        flywheelOutputVolts = clampedOutputVolts;
        leftFlywheel.setControl(leftFlywheelVoltageRequest.withOutput(flywheelOutputVolts));
        rightFlywheel.setControl(rightFlywheelVoltageRequest.withOutput(-flywheelOutputVolts));
    }

    public void setFlywheelVoltageByDistance(double distance) {
        double targetVolts = Constants.Shooter.flywheelVoltageByDistance.get(distance);
        setFlywheelVoltage(targetVolts);
    }

    public void stopFlywheel() {
        setFlywheelVoltage(0.0);
    }

    public boolean atVoltage() {
        if (Math.abs(flywheelOutputVolts) < Constants.Shooter.flywheelReadyVoltageThreshold) {
            return false;
        }
        return Timer.getFPGATimestamp() - flywheelVoltageStartTimestamp
            >= Constants.Shooter.flywheelReadyDelaySeconds;
    }
    public double getLeftFlywheelVelocity() {
        return leftFlywheel.getVelocity().getValueAsDouble();
    }

    public double getRightFlywheelVelocity() {
        return rightFlywheel.getVelocity().getValueAsDouble();
    }

    public double getAverageFlywheelVelocity() {
        return (Math.abs(getLeftFlywheelVelocity()) + Math.abs(getRightFlywheelVelocity())) / 2.0;
    }

    public double getFlywheelVelocityDifference() {
        return Math.abs(getLeftFlywheelVelocity()) - Math.abs(getRightFlywheelVelocity());
    }

    public double getTargetFlywheelVoltage() {
        return flywheelOutputVolts;
    }

    /* ====================== */
    /*    shooter conveyor     */
    /* ====================== */

    public void runShooterConveyorVoltage(double outputVolts) {
        shooterConveyorOutputVolts = MathUtil.clamp(outputVolts, -12.0, 12.0);
        leftConveyor.setControl(conveyorVoltageRequest.withOutput(shooterConveyorOutputVolts));
        setRightConveyorFollowLeft();
    }

    public void stopShooterConveyor() {
        runShooterConveyorVoltage(0.0);
    }

    public double getConveyorStatorCurrent() {
        return leftConveyor.getStatorCurrent().getValueAsDouble();
    }

    /* ====================== */
    /*         背板            */
    /* ====================== */

    public void setBackboardVoltage(double outputVolts) {
        backboardOutputVolts = MathUtil.clamp(
            outputVolts,
            -Constants.Shooter.backboardSpeedMax,
            Constants.Shooter.backboardSpeedMax
        );
        backboardMotor.setControl(backboardVoltageRequest.withOutput(backboardOutputVolts));
    }

    public void setBackboardPosition(double targetPosition) {
        // 保留旧背板机构的软件限位。
        if (targetPosition > Constants.Shooter.backboardUpLimit) {
            targetPosition = Constants.Shooter.backboardUpLimit;
        } else if (targetPosition < Constants.Shooter.backboardDownLimit) {
            targetPosition = Constants.Shooter.backboardDownLimit;
        }
        backboardPID.setGoal(targetPosition);
    }

    public void outputBackboard() {
        // 背板仍使用旧外部编码器和 ProfiledPID；Superstructure 只选择目标位置。
        double backboardPosition = getBackboardPosition();
        double rawPidOutput = backboardPID.calculate(backboardPosition);
        double pidOutput = MathUtil.clamp(
            rawPidOutput,
            -Constants.Shooter.backboardSpeedMax,
            Constants.Shooter.backboardSpeedMax
        );
        backboardRawPidOutputVolts = rawPidOutput;
        backboardClampedPidOutputVolts = pidOutput;
        backboardOutputSaturated = Math.abs(rawPidOutput - pidOutput) > 1e-9;

        if (pidOutput != 0.0 && Math.abs(pidOutput) < Constants.Shooter.backboardSpeedMin) {
            pidOutput = Math.copySign(Constants.Shooter.backboardSpeedMin, pidOutput);
            backboardMinimumOutputActive = true;
        } else {
            backboardMinimumOutputActive = false;
        }

        double gravityOutput = calculateBackboardGravityFeedforward(backboardPosition);
        double outputVolts = MathUtil.clamp(
            pidOutput + gravityOutput,
            -Constants.Shooter.backboardSpeedMax,
            Constants.Shooter.backboardSpeedMax
        );
        backboardGravityFeedforwardVolts = gravityOutput;
        backboardOutputSaturated = backboardOutputSaturated
            || Math.abs((pidOutput + gravityOutput) - outputVolts) > 1e-9;

        shooterNetworkTable.getEntry("backboardPIDOutput").setDouble(outputVolts);
        setBackboardVoltage(outputVolts);
    }

    private double calculateBackboardGravityFeedforward(double backboardPosition) {
        double clampedPosition = MathUtil.clamp(
            backboardPosition,
            Constants.Shooter.backboardGravityFeedforwardMinPosition,
            Constants.Shooter.backboardGravityFeedforwardMaxPosition
        );
        double positionRange = Constants.Shooter.backboardGravityFeedforwardMaxPosition
            - Constants.Shooter.backboardGravityFeedforwardMinPosition;
        if (positionRange <= 0.0) {
            return 0.0;
        }

        double positionRatio = (clampedPosition - Constants.Shooter.backboardGravityFeedforwardMinPosition)
            / positionRange;
        double feedforwardVolts = Constants.Shooter.backboardGravityFeedforwardMinVolts
            + positionRatio
                * (Constants.Shooter.backboardGravityFeedforwardMaxVolts
                    - Constants.Shooter.backboardGravityFeedforwardMinVolts);
        return Constants.Shooter.backboardGravityFeedforwardDirection * feedforwardVolts;
    }

    public void holdBackboardAt(double targetPosition) {
        // 上层只给目标位置；到位后仍继续输出重力补偿。
        setBackboardPosition(targetPosition);
        outputBackboard();
    }

    public void stopBackboard() {
        backboardRawPidOutputVolts = 0.0;
        backboardGravityFeedforwardVolts = 0.0;
        backboardClampedPidOutputVolts = 0.0;
        backboardOutputSaturated = false;
        backboardMinimumOutputActive = false;
        setBackboardVoltage(0.0);
    }

    public double getBackboardPosition() {
        return backboardEncoder.getDistance();
    }

    public boolean isBackboardAtTarget() {
        return backboardPID.atGoal();
    }

    public boolean isBackboardPositionTuningActive() {
        return backboardTuningTable.getEntry("backboardPositionEnable").getBoolean(false);
    }

    public void resetbackboardencoder() {
        backboardEncoder.reset();
        backboardPID.reset(0);
    }

    /**
     * 背板回零命令：闭环回到 0 位后停止输出。
     * 命令留在 subsystem 内，controls 层只负责绑定按键。
     */
    public Command homeBackboardCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> setBackboardPosition(0.0), this),
            Commands.run(() -> holdBackboardAt(0.0), this).until(this::isBackboardAtTarget),
            Commands.runOnce(this::stopBackboard, this)
        );
    }

    /* ====================== */
    /*         整体            */
    /* ====================== */

    public void stopAll() {
        stopFlywheel();
        stopShooterConveyor();
        stopBackboard();
    }

    public void debugBackboardTargetPeriodic() {
        boolean positionEnable = backboardTuningTable.getEntry("backboardPositionEnable").getBoolean(false);
        double requestedTarget = backboardTuningTable.getEntry("backboardTargetPosition").getDouble(0.0);
        double clampedTarget = MathUtil.clamp(
            requestedTarget,
            Constants.Shooter.backboardDownLimit,
            Constants.Shooter.backboardUpLimit
        );

        backboardTuningTable.getEntry("backboardPositionEnableActive").setBoolean(positionEnable);
        backboardTuningTable.getEntry("backboardCommandedTargetPosition").setDouble(requestedTarget);
        backboardTuningTable.getEntry("backboardClampedTargetPosition").setDouble(clampedTarget);

        if (positionEnable) {
            holdBackboardAt(clampedTarget);
        }
    }

    @Override
    public void periodic() {
        double backboardPosition = getBackboardPosition();
        double backboardGoalPosition = backboardPID.getGoal().position;
        double backboardSetpointPosition = backboardPID.getSetpoint().position;
        shooterNetworkTable.getEntry("flywheelVelocity").setDouble(getAverageFlywheelVelocity());
        shooterNetworkTable.getEntry("leftFlywheelVelocity").setDouble(getLeftFlywheelVelocity());
        shooterNetworkTable.getEntry("rightFlywheelVelocity").setDouble(getRightFlywheelVelocity());
        shooterNetworkTable.getEntry("flywheelVelocityDifference").setDouble(getFlywheelVelocityDifference());
        shooterNetworkTable.getEntry("flywheelTargetVoltage").setDouble(getTargetFlywheelVoltage());
        shooterNetworkTable.getEntry("flywheelOutputVolts").setDouble(flywheelOutputVolts);
        shooterNetworkTable.getEntry("flywheelAtVoltage").setBoolean(atVoltage());
        shooterNetworkTable.getEntry("shooterConveyorOutputVolts").setDouble(shooterConveyorOutputVolts);
        shooterNetworkTable.getEntry("conveyorVelocity").setDouble(leftConveyor.getVelocity().getValueAsDouble());
        shooterNetworkTable.getEntry("backboardCurrentRate").setDouble(backboardPosition);
        shooterNetworkTable.getEntry("backboardTargetRate").setDouble(backboardSetpointPosition);
        shooterNetworkTable.getEntry("backboardOutputVolts").setDouble(backboardOutputVolts);
        shooterNetworkTable.getEntry("isBackboardAtTarget").setBoolean(isBackboardAtTarget());
        shooterNetworkTable.getEntry("backboardGoalPosition").setDouble(backboardGoalPosition);
        shooterNetworkTable.getEntry("backboardSetpointPosition").setDouble(backboardSetpointPosition);
        shooterNetworkTable.getEntry("backboardSetpointVelocity").setDouble(backboardPID.getSetpoint().velocity);
        shooterNetworkTable.getEntry("backboardGoalError").setDouble(backboardGoalPosition - backboardPosition);
        shooterNetworkTable.getEntry("backboardSetpointError").setDouble(backboardSetpointPosition - backboardPosition);
        shooterNetworkTable.getEntry("backboardRawPIDOutput").setDouble(backboardRawPidOutputVolts);
        shooterNetworkTable.getEntry("backboardGravityFeedforward").setDouble(backboardGravityFeedforwardVolts);
        shooterNetworkTable.getEntry("backboardClampedPIDOutput").setDouble(backboardClampedPidOutputVolts);
        shooterNetworkTable.getEntry("backboardOutputSaturated").setBoolean(backboardOutputSaturated);
        shooterNetworkTable.getEntry("backboardMinimumOutputActive").setBoolean(backboardMinimumOutputActive);
        shooterNetworkTable.getEntry("backboardMaxOutputVolts").setDouble(Constants.Shooter.backboardSpeedMax);
        shooterNetworkTable.getEntry("backboardMinOutputVolts").setDouble(Constants.Shooter.backboardSpeedMin);
        shooterNetworkTable.getEntry("backboardGravityFeedforwardMinVolts").setDouble(Constants.Shooter.backboardGravityFeedforwardMinVolts);
        shooterNetworkTable.getEntry("backboardGravityFeedforwardMaxVolts").setDouble(Constants.Shooter.backboardGravityFeedforwardMaxVolts);
        shooterNetworkTable.getEntry("backboardPositionTolerance").setDouble(10.0);
        shooterNetworkTable.getEntry("backboardShootPosition").setDouble(Constants.Superstructure.backboardShootPosition);
        shooterNetworkTable.getEntry("backboardPassPosition").setDouble(Constants.Superstructure.passBallBackboardPosition);
        shooterNetworkTable.getEntry("backboardMotorPosition").setDouble(backboardMotor.getPosition().getValueAsDouble());
        shooterNetworkTable.getEntry("backboardMotorVelocity").setDouble(backboardMotor.getVelocity().getValueAsDouble());
        shooterNetworkTable.getEntry("backboardMotorVoltage").setDouble(backboardMotor.getMotorVoltage().getValueAsDouble());
        shooterNetworkTable.getEntry("backboardMotorStatorCurrent").setDouble(backboardMotor.getStatorCurrent().getValueAsDouble());
        shooterNetworkTable.getEntry("statorCurrent").setDouble(getConveyorStatorCurrent());
    }
}

