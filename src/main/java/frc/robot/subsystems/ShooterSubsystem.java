package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
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

    private final VelocityVoltage leftFlywheelVelocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage conveyorVelocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut backboardVoltageRequest = new VoltageOut(0);
    private final NeutralOut flywheelNeutralRequest = new NeutralOut();
    private final NeutralOut conveyorNeutralRequest = new NeutralOut();

    private final Follower flywheelFollower;
    private final Follower conveyorFollower;

    private final NetworkTable shooterNetworkTable;
    private final NetworkTable flywheelTuningTable;
    private final NetworkTable backboardTuningTable;
    private final NetworkTable shooterConveyorTuningTable;

    private double flywheelOutputVolts = 0.0;
    private double flywheelTargetVelocity = 0.0;
    private double flywheelVelocityTolerance = Constants.Shooter.flywheelReadyVelocityTolerance;
    private boolean flywheelVelocityTuningActive = false;
    private double shooterConveyorTargetVelocity = 0.0;
    private double shooterConveyorCommandedTargetVelocity = Constants.Shooter.shooterConveyorFeedVelocity;
    private double shooterConveyorClampedTargetVelocity = Constants.Shooter.shooterConveyorFeedVelocity;
    private boolean shooterConveyorVelocityTuningActive = false;
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

        var leftFlywheelMotorOutputConfigs = new MotorOutputConfigs();
        leftFlywheelMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        leftFlywheelMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        leftFlywheel.getConfigurator().apply(leftFlywheelMotorOutputConfigs);

        var rightFlywheelMotorOutputConfigs = new MotorOutputConfigs();
        rightFlywheelMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        rightFlywheel.getConfigurator().apply(rightFlywheelMotorOutputConfigs);

        var conveyorMotorOutputConfigs = new MotorOutputConfigs();
        conveyorMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        leftConveyor.getConfigurator().apply(conveyorMotorOutputConfigs);

        flywheelFollower = new Follower(leftFlywheel.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(50);
        conveyorFollower = new Follower(leftConveyor.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(50);
        setRightFlywheelFollowLeft();
        setRightConveyorFollowLeft();

        backboardMotor = new TalonFXS(Constants.Shooter.backboardMotorID);
        backboardMotor.getConfigurator().apply(Constants.Shooter.backboardSlot0Configs);
        setBackboardBrakeMode();

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
        flywheelTuningTable = NetworkTableInstance.getDefault().getTable("Tuning/flywheel");
        flywheelTuningTable.getEntry("flywheelVelocityEnable").setDefaultBoolean(false);
        flywheelTuningTable.getEntry("flywheelTargetVelocity").setDefaultDouble(Constants.Shooter.defaultFlywheelVelocity);
        flywheelTuningTable.getEntry("flywheelVelocityTolerance").setDefaultDouble(Constants.Shooter.flywheelReadyVelocityTolerance);
        backboardTuningTable = NetworkTableInstance.getDefault().getTable("Tuning/backboard");
        backboardTuningTable.getEntry("backboardPositionEnable").setDefaultBoolean(false);
        backboardTuningTable.getEntry("backboardTargetPosition").setDefaultDouble(0.0);
        shooterConveyorTuningTable = NetworkTableInstance.getDefault().getTable("Tuning/shooterConveyor");
        shooterConveyorTuningTable.getEntry("shooterConveyorVelocityEnable").setDefaultBoolean(false);
        shooterConveyorTuningTable.getEntry("shooterConveyorTargetVelocity")
            .setDefaultDouble(Constants.Shooter.shooterConveyorFeedVelocity);
    }

    /* ====================== */
    /*         飞轮            */
    /* ====================== */

    public void setRightConveyorFollowLeft() {
        // 成对机构机械方向相反，因此右侧电机以反向 follower 跟随左侧电机。
        rightConveyor.setControl(conveyorFollower);
    }

    public void setRightFlywheelFollowLeft() {
        rightFlywheel.setControl(flywheelFollower);
    }

    public void applyLeftConfigurationToRight() {
        Slot0Configs configs = new Slot0Configs();
        leftFlywheel.getConfigurator().refresh(configs);
        rightFlywheel.getConfigurator().apply(configs);
    }

    public void setFlywheelVelocity(double targetVelocityRps) {
        flywheelTargetVelocity = targetVelocityRps;
        leftFlywheel.setControl(leftFlywheelVelocityRequest.withVelocity(flywheelTargetVelocity));
        setRightFlywheelFollowLeft();
    }

    public void stopFlywheel() {
        flywheelTargetVelocity = 0.0;
        leftFlywheel.setControl(flywheelNeutralRequest);
        rightFlywheel.setControl(flywheelNeutralRequest);
    }

    public boolean atVelocity() {
        if (flywheelTargetVelocity <= 10.0) {
            return false;
        }
        double currentvelocity = getAverageFlywheelVelocity();
        if(currentvelocity>flywheelTargetVelocity*0.95){
            return true;
        } else {
            return false;
        }   
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

    public double getTargetFlywheelVelocity() {
        return flywheelTargetVelocity;
    }

    public double getFlywheelVelocityError() {
        return Math.abs(flywheelTargetVelocity) - getAverageFlywheelVelocity();
    }

    /* ====================== */
    /*    shooter conveyor     */
    /* ====================== */

    public void runShooterConveyorVelocity(double targetVelocityRps) {
        shooterConveyorTargetVelocity = targetVelocityRps;
        leftConveyor.setControl(conveyorVelocityRequest.withVelocity(shooterConveyorTargetVelocity));
        setRightConveyorFollowLeft();
    }

    public void stopShooterConveyor() {
        shooterConveyorTargetVelocity = 0.0;
        leftConveyor.setControl(conveyorNeutralRequest);
        rightConveyor.setControl(conveyorNeutralRequest);
    }

    public double getShooterConveyorVelocity() {
        return leftConveyor.getVelocity().getValueAsDouble();
    }

    public double getShooterConveyorTargetVelocity() {
        return shooterConveyorTargetVelocity;
    }

    public double getShooterConveyorVelocityError() {
        return shooterConveyorTargetVelocity - getShooterConveyorVelocity();
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

    public void setBackboardBrakeMode() {
        var backboardMotorOutputConfigs = new MotorOutputConfigs();
        backboardMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        backboardMotor.getConfigurator().apply(backboardMotorOutputConfigs);
    }

    public double getBackboardPosition() {
        return backboardEncoder.getDistance();
    }

    public boolean isBackboardAtTarget() {
        return backboardPID.atGoal();
    }

    public double getBackboardGoalPosition() {
        return backboardPID.getGoal().position;
    }

    public double getBackboardPositionError() {
        return getBackboardGoalPosition() - getBackboardPosition();
    }

    public boolean isBackboardNearTarget(double tolerance) {
        return Math.abs(getBackboardPositionError()) < tolerance;
    }

    public boolean isBackboardPositionTuningActive() {
        return DriverStation.isTest()
            && backboardTuningTable.getEntry("backboardPositionEnable").getBoolean(false);
    }

    public boolean isFlywheelTuningControlActive() {
        return DriverStation.isTest()
            && flywheelTuningTable.getEntry("flywheelVelocityEnable").getBoolean(false);
    }

    public boolean isShooterConveyorTuningControlActive() {
        return DriverStation.isTestEnabled()
            && shooterConveyorTuningTable.getEntry("shooterConveyorVelocityEnable").getBoolean(false);
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

    public void debugBackboardTargetPeriodic(boolean allowMotorOutput) {
        boolean positionEnable =
            allowMotorOutput && backboardTuningTable.getEntry("backboardPositionEnable").getBoolean(false);
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

    public void debugFlywheelVelocityPeriodic(boolean allowMotorOutput) {
        flywheelVelocityTuningActive =
            allowMotorOutput && flywheelTuningTable.getEntry("flywheelVelocityEnable").getBoolean(false);
        double requestedTargetVelocity =
            flywheelTuningTable.getEntry("flywheelTargetVelocity").getDouble(Constants.Shooter.defaultFlywheelVelocity);
        flywheelVelocityTolerance = Math.max(
            0.0,
            flywheelTuningTable.getEntry("flywheelVelocityTolerance")
                .getDouble(Constants.Shooter.flywheelReadyVelocityTolerance)
        );

        double clampedTargetVelocity = MathUtil.clamp(
            requestedTargetVelocity,
            -Constants.Shooter.flywheelMaxVelocity,
            Constants.Shooter.flywheelMaxVelocity
        );
        flywheelTuningTable.getEntry("flywheelVelocityEnableActive").setBoolean(flywheelVelocityTuningActive);
        flywheelTuningTable.getEntry("flywheelCommandedTargetVelocity").setDouble(requestedTargetVelocity);
        flywheelTuningTable.getEntry("flywheelClampedTargetVelocity").setDouble(clampedTargetVelocity);
        flywheelTuningTable.getEntry("flywheelVelocityToleranceActive").setDouble(flywheelVelocityTolerance);

        if (flywheelVelocityTuningActive) {
            setFlywheelVelocity(clampedTargetVelocity);
        }
    }

    public void debugShooterConveyorVelocityPeriodic(boolean allowMotorOutput) {
        shooterConveyorVelocityTuningActive =
            allowMotorOutput
                && DriverStation.isTestEnabled()
                && shooterConveyorTuningTable.getEntry("shooterConveyorVelocityEnable").getBoolean(false);
        shooterConveyorCommandedTargetVelocity = shooterConveyorTuningTable
            .getEntry("shooterConveyorTargetVelocity")
            .getDouble(Constants.Shooter.shooterConveyorFeedVelocity);
        shooterConveyorClampedTargetVelocity = MathUtil.clamp(
            shooterConveyorCommandedTargetVelocity,
            -Constants.Shooter.shooterConveyorMaxVelocity,
            Constants.Shooter.shooterConveyorMaxVelocity
        );

        shooterConveyorTuningTable.getEntry("shooterConveyorVelocityEnableActive")
            .setBoolean(shooterConveyorVelocityTuningActive);
        shooterConveyorTuningTable.getEntry("shooterConveyorCommandedTargetVelocity")
            .setDouble(shooterConveyorCommandedTargetVelocity);
        shooterConveyorTuningTable.getEntry("shooterConveyorClampedTargetVelocity")
            .setDouble(shooterConveyorClampedTargetVelocity);
        shooterConveyorTuningTable.getEntry("shooterConveyorMaxVelocity")
            .setDouble(Constants.Shooter.shooterConveyorMaxVelocity);

        if (shooterConveyorVelocityTuningActive) {
            runShooterConveyorVelocity(shooterConveyorClampedTargetVelocity);
        }
    }

    @Override
    public void periodic() {
        double backboardPosition = getBackboardPosition();
        double backboardGoalPosition = backboardPID.getGoal().position;
        double backboardSetpointPosition = backboardPID.getSetpoint().position;
        flywheelOutputVolts = leftFlywheel.getMotorVoltage().getValueAsDouble();
        shooterNetworkTable.getEntry("flywheelVelocity").setDouble(getAverageFlywheelVelocity());
        shooterNetworkTable.getEntry("leftFlywheelVelocity").setDouble(getLeftFlywheelVelocity());
        shooterNetworkTable.getEntry("rightFlywheelVelocity").setDouble(getRightFlywheelVelocity());
        shooterNetworkTable.getEntry("flywheelVelocityDifference").setDouble(getFlywheelVelocityDifference());
        shooterNetworkTable.getEntry("flywheelTargetVelocity").setDouble(getTargetFlywheelVelocity());
        shooterNetworkTable.getEntry("flywheelVelocityError").setDouble(getFlywheelVelocityError());
        shooterNetworkTable.getEntry("flywheelVelocityTolerance").setDouble(flywheelVelocityTolerance);
        shooterNetworkTable.getEntry("flywheelAtVelocity").setBoolean(atVelocity());
        shooterNetworkTable.getEntry("flywheelVelocityTuningActive").setBoolean(flywheelVelocityTuningActive);
        shooterNetworkTable.getEntry("flywheelFollowerOpposed").setBoolean(true);
        shooterNetworkTable.getEntry("flywheelOutputVolts").setDouble(flywheelOutputVolts);
        shooterNetworkTable.getEntry("leftFlywheelOutputVolts").setDouble(leftFlywheel.getMotorVoltage().getValueAsDouble());
        shooterNetworkTable.getEntry("rightFlywheelOutputVolts").setDouble(rightFlywheel.getMotorVoltage().getValueAsDouble());
        shooterNetworkTable.getEntry("leftFlywheelStatorCurrent").setDouble(leftFlywheel.getStatorCurrent().getValueAsDouble());
        shooterNetworkTable.getEntry("rightFlywheelStatorCurrent").setDouble(rightFlywheel.getStatorCurrent().getValueAsDouble());
        shooterConveyorOutputVolts = leftConveyor.getMotorVoltage().getValueAsDouble();
        shooterNetworkTable.getEntry("shooterConveyorOutputVolts").setDouble(shooterConveyorOutputVolts);
        shooterNetworkTable.getEntry("conveyorTargetVelocity").setDouble(getShooterConveyorTargetVelocity());
        shooterNetworkTable.getEntry("conveyorVelocity").setDouble(getShooterConveyorVelocity());
        shooterNetworkTable.getEntry("conveyorVelocityError").setDouble(getShooterConveyorVelocityError());
        shooterNetworkTable.getEntry("conveyorVelocityTuningActive").setBoolean(shooterConveyorVelocityTuningActive);
        shooterNetworkTable.getEntry("conveyorCommandedTargetVelocity").setDouble(shooterConveyorCommandedTargetVelocity);
        shooterNetworkTable.getEntry("conveyorClampedTargetVelocity").setDouble(shooterConveyorClampedTargetVelocity);
        shooterNetworkTable.getEntry("conveyorLeftStatorCurrent").setDouble(leftConveyor.getStatorCurrent().getValueAsDouble());
        shooterNetworkTable.getEntry("conveyorRightStatorCurrent").setDouble(rightConveyor.getStatorCurrent().getValueAsDouble());
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
