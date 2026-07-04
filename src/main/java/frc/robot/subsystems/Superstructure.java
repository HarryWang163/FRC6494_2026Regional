package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeRotaterSubsystem.IntakePosition;

/**
 * 统一协调所有得分相关机构。
 *
 * subsystem 只暴露硬件级动作，OperatorControls 只表达操作员意图；
 * shooter、intake、conveyor、底盘瞄准和 Limelight 距离如何配合，
 * 统一由这里决定。
 */
public class Superstructure extends SubsystemBase {
    // 操作员或自动程序请求的目标状态。
    public enum WantedState {
        IDLE,
        INTAKE,
        PREP_SHOOT,
        SHOOT,
        EJECT,
        MANUAL
    }

    // 机器人内部实际执行状态，用于动作排序和仪表盘诊断。
    public enum SystemState {
        DISABLED,
        STOWED,
        DEPLOYING_INTAKE,
        INTAKING,
        SPINNING_UP,
        AIMING,
        READY_TO_SHOOT,
        SHOOTING,
        CLEANUP,
        EJECTING,
        MANUAL_OVERRIDE
    }

    private final ShooterSubsystem shooter;
    private final IntakeRollerSubsystem intakeRoller;
    private final IntakeRotaterSubsystem intakeRotater;
    private final ConveyorSubsystem conveyor;
    private final CommandSwerveDrivetrain drive;
    private final LimelightSubsystem limelight;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Superstructure");

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.STOWED;
    private double stateStartTimestamp = Timer.getFPGATimestamp();
    private double flywheelSpeedOffsetRps = 0.0;
    private double backboardPositionOffset = 0.0;

    public Superstructure(
        ShooterSubsystem shooter,
        IntakeRollerSubsystem intakeRoller,
        IntakeRotaterSubsystem intakeRotater,
        ConveyorSubsystem conveyor,
        CommandSwerveDrivetrain drive,
        LimelightSubsystem limelight
    ) {
        this.shooter = shooter;
        this.intakeRoller = intakeRoller;
        this.intakeRotater = intakeRotater;
        this.conveyor = conveyor;
        this.drive = drive;
        this.limelight = limelight;
    }

    public void requestIdle() {
        wantedState = WantedState.IDLE;
    }

    public void requestIntake() {
        wantedState = WantedState.INTAKE;
    }

    public void requestPrepShoot() {
        wantedState = WantedState.PREP_SHOOT;
    }

    public void requestShoot() {
        wantedState = WantedState.SHOOT;
    }

    public void requestEject() {
        wantedState = WantedState.EJECT;
    }

    public void requestManual() {
        wantedState = WantedState.MANUAL;
    }

    public Command requestIdleCommand() {
        return Commands.runOnce(this::requestIdle, this);
    }

    public Command requestIntakeCommand() {
        return Commands.runOnce(this::requestIntake, this);
    }

    public Command requestPrepShootCommand() {
        return Commands.runOnce(this::requestPrepShoot, this);
    }

    public Command requestShootCommand() {
        return Commands.runOnce(this::requestShoot, this);
    }

    public Command requestEjectCommand() {
        return Commands.runOnce(this::requestEject, this);
    }

    public Command requestManualCommand() {
        return Commands.runOnce(this::requestManual, this);
    }

    public WantedState getWantedState() {
        return wantedState;
    }

    public SystemState getSystemState() {
        return systemState;
    }

    public boolean canShoot() {
        // 本赛季不使用球检测传感器，因此射击门控只检查机器人能实际测量的条件。
        return shooter.atSpeed()
            && drive.isAimed();
    }

    public void adjustFlywheelSpeedOffset(double offsetRps) {
        flywheelSpeedOffsetRps += offsetRps;
    }

    public void adjustBackboardPositionOffset(double offset) {
        backboardPositionOffset += offset;
    }

    public void resetOffsets() {
        flywheelSpeedOffsetRps = 0.0;
        backboardPositionOffset = 0.0;
    }

    public void resetBackboardEncoder() {
        shooter.resetbackboardencoder();
    }

    @Override
    public void periodic() {
        // 禁用状态优先级最高，防止旧按钮输入或自动请求在禁用后继续驱动机构。
        if (DriverStation.isDisabled()) {
            setSystemState(SystemState.DISABLED);
            stopAllMechanisms();
            publishTelemetry();
            return;
        }

        updateAimingSetpoint();

        switch (wantedState) {
            case IDLE -> handleIdle();
            case INTAKE -> handleIntake();
            case PREP_SHOOT -> handlePrepShoot(false);
            case SHOOT -> handlePrepShoot(true);
            case EJECT -> handleEject();
            case MANUAL -> handleManual();
        }

        publishTelemetry();
    }

    private void handleIdle() {
        // 安全默认状态：传球不动，intake 收回，背板保持受控。
        conveyor.stop();
        intakeRoller.stop();
        intakeRotater.stop();
        shooter.stopFlywheel();
        shooter.stopShooterConveyor();
        holdBackboardAtZero();

        setSystemState(SystemState.STOWED);
    }

    private void handleIntake() {
        if (systemState != SystemState.DEPLOYING_INTAKE
            && systemState != SystemState.INTAKING) {
            setSystemState(SystemState.DEPLOYING_INTAKE);
        }

        shooter.stopFlywheel();
        shooter.stopShooterConveyor();
        intakeRotater.deployToGround();
        intakeRoller.intake();
        conveyor.feedToShooter();
        holdBackboardAtZero();

        if (intakeRotater.atGoal()) {
            setSystemState(SystemState.INTAKING);
        }

        // 本赛季没有球检测，因此 intake 通过时间退出。
        // 真车确认 roller/conveyor 速度后，需要现场调整这个时间。
        if (systemState == SystemState.INTAKING && timeInState() > 1.0) {
            wantedState = WantedState.IDLE;
            setSystemState(SystemState.STOWED);
        }
    }

    private void handlePrepShoot(boolean feedWhenReady) {
        if (systemState != SystemState.SPINNING_UP
            && systemState != SystemState.AIMING
            && systemState != SystemState.READY_TO_SHOOT
            && systemState != SystemState.SHOOTING
            && systemState != SystemState.CLEANUP) {
            setSystemState(SystemState.SPINNING_UP);
        }

        intakeRotater.moveToHandoff();
        intakeRoller.stop();
        conveyor.stop();
        shooter.stopShooterConveyor();
        // 距离到射速的映射由 LimelightSubsystem 提供；操作员 offset 用于现场微调，
        // 但不会绕过 Superstructure 状态机。
        double targetRps = limelight.getShooterSetpointByDistance() + flywheelSpeedOffsetRps;
        shooter.setFlywheelVelocity(targetRps, targetRps);
        holdBackboardForShot();

        if (shooter.atSpeed()) {
            setSystemState(SystemState.AIMING);
        }

        if (shooter.atSpeed() && drive.isAimed() && intakeRotater.atGoal()) {
            setSystemState(SystemState.READY_TO_SHOOT);
        }

        if (feedWhenReady && canShoot()) {
            setSystemState(SystemState.SHOOTING);
        }

        if (systemState == SystemState.SHOOTING) {
            // 只有 canShoot() 通过后才允许喂球。
            conveyor.feedToShooter();
            shooter.runShooterConveyor(Constants.Superstructure.shooterFeedPercent);

            if (timeInState() > Constants.Superstructure.shootTimeoutSeconds) {
                setSystemState(SystemState.CLEANUP);
            }
        }

        if (systemState == SystemState.CLEANUP) {
            conveyor.stop();
            shooter.stopShooterConveyor();
            if (timeInState() > Constants.Superstructure.cleanupSeconds) {
                wantedState = WantedState.IDLE;
                setSystemState(SystemState.STOWED);
            }
        }
    }

    private void handleEject() {
        // Eject 会反转整条球路，但飞轮保持停止。
        setSystemState(SystemState.EJECTING);
        intakeRotater.deployToGround();
        intakeRoller.outtake();
        conveyor.reverse();
        shooter.runShooterConveyor(Constants.Superstructure.shooterReversePercent);
        shooter.stopFlywheel();
        holdBackboardAtZero();
    }

    private void handleManual() {
        setSystemState(SystemState.MANUAL_OVERRIDE);
        shooter.stopFlywheel();
        shooter.stopShooterConveyor();
        conveyor.stop();
        intakeRoller.stop();
    }

    private void updateAimingSetpoint() {
        // Superstructure 可以更新底盘目标朝向，但不直接下发 swerve 模块或底盘速度命令。
        if (limelight.hasTarget()) {
            drive.setTargetHeading(drive.getRotation().plus(Rotation2d.fromDegrees(limelight.getTargetYaw())));
        }
    }

    private void holdBackboardAtZero() {
        shooter.setBackboardPosition(0.0);
        if (shooter.isBackboardAtTarget()) {
            shooter.backboardMotor.set(0.0);
        } else {
            shooter.outputBackboard();
        }
    }

    private void holdBackboardForShot() {
        shooter.setBackboardPosition(175.0 + backboardPositionOffset);
        if (shooter.isBackboardAtTarget()) {
            shooter.backboardMotor.set(0.0);
        } else {
            shooter.outputBackboard();
        }
    }

    private void stopAllMechanisms() {
        shooter.stopAll();
        intakeRoller.stop();
        intakeRotater.stop();
        conveyor.stop();
    }

    private void setSystemState(SystemState nextState) {
        if (systemState != nextState) {
            systemState = nextState;
            stateStartTimestamp = Timer.getFPGATimestamp();
        }
    }

    private double timeInState() {
        return Timer.getFPGATimestamp() - stateStartTimestamp;
    }

    private void publishTelemetry() {
        // 发布状态和门控信号，方便判断机器人为什么还没进入射击。
        table.getEntry("wantedState").setString(wantedState.name());
        table.getEntry("systemState").setString(systemState.name());
        table.getEntry("canShoot").setBoolean(canShoot());
        table.getEntry("shooterAtSpeed").setBoolean(shooter.atSpeed());
        table.getEntry("driveAimed").setBoolean(drive.isAimed());
        table.getEntry("intakeAtGoal").setBoolean(intakeRotater.atGoal());
        table.getEntry("flywheelSpeedOffsetRps").setDouble(flywheelSpeedOffsetRps);
        table.getEntry("backboardPositionOffset").setDouble(backboardPositionOffset);
    }
}

