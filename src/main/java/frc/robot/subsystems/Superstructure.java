package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * 统一协调所有得分相关机构。
 *
 * subsystem 只暴露硬件级动作，OperatorControls 只表达操作员意图；
 * shooter、intake、conveyor、底盘瞄准和 Limelight 距离如何配合，
 * 统一由这里决定。底盘瞄准本身仍由 DriveControls 的 AutoAimming
 * 模式完成，这里只读取 drive.isAimed() 等状态，不直接控制底盘电机。
 *
 * 没有球检测传感器，持球状态按状态流程推断：INDEXING 完成置位，
 * 射击/EJECT 后清空；跟踪不可靠时可用 Constants 里的开关旁路门控。
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
        INDEXING,
        HOLDING_NOTE,
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

    /* ====================== */
    /*     操作员意图入口       */
    /* ====================== */

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

    public Command toggleNotePresentCommand() {
        // 持球状态是推断值，中途 disable 或漏吸球都可能让它失真；
        // 给操作员一个运行时纠正入口（X 键），不用重启机器人。
        return Commands.runOnce(() -> conveyor.setNotePresent(!conveyor.hasNote()));
    }

    public WantedState getWantedState() {
        return wantedState;
    }

    public SystemState getSystemState() {
        return systemState;
    }

    /* ====================== */
    /*        射击门控          */
    /* ====================== */

    public boolean canShoot() {
        boolean noteGate = !Constants.Superstructure.hasNoteGateEnabled || conveyor.hasNote();
        boolean stationaryGate = !Constants.Superstructure.stationaryGateEnabled || isDriveStationary();
        return shooter.atSpeed()
            && drive.isAimed()
            && intakeRotater.atGoal()
            && noteGate
            && stationaryGate;
    }

    private boolean isDriveStationary() {
        ChassisSpeeds speeds = drive.getChassisSpeeds();
        double translationSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return translationSpeed <= Constants.Superstructure.stationarySpeedToleranceMetersPerSecond;
    }

    /* ====================== */
    /*      现场微调 offset     */
    /* ====================== */

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

    /* ====================== */
    /*        状态机主体        */
    /* ====================== */

    @Override
    public void periodic() {
        // 禁用状态优先级最高，防止旧按钮输入或自动请求在禁用后继续驱动机构。
        if (DriverStation.isDisabled()) {
            // 进入禁用前先结算持球推断，避免中途 disable 留下失真的 hasNote：
            // 喂球中被禁用视为球已交给飞轮；收纳中被禁用视为球已在传送带里。
            if (systemState == SystemState.SHOOTING) {
                conveyor.setNotePresent(false);
            } else if (systemState == SystemState.INDEXING) {
                conveyor.setNotePresent(true);
            }
            setSystemState(SystemState.DISABLED);
            stopAllMechanisms();
            publishTelemetry();
            return;
        }

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
        // 从其他流程回到 IDLE 时的入口流转。
        switch (systemState) {
            // 松开 intake 键后先收纳，不直接回 STOWED。
            // 未归零被安全逻辑拦下时滚轮从未启动，不可能有球，直接回 STOWED，
            // 避免把 hasNote 误置为 true。
            case DEPLOYING_INTAKE, INTAKING -> setSystemState(
                intakeRotater.isBlockedByNotZeroed() ? SystemState.STOWED : SystemState.INDEXING);
            // 喂球中途松开按键：球已经交给飞轮，按射完处理。
            case SHOOTING -> {
                conveyor.setNotePresent(false);
                setSystemState(SystemState.CLEANUP);
            }
            // EJECT 结束：球路已排空。
            case EJECTING -> {
                conveyor.setNotePresent(false);
                setSystemState(SystemState.STOWED);
            }
            default -> {}
        }

        if (systemState == SystemState.INDEXING) {
            // 收纳中：intake 回 HANDOFF 交接位，滚轮低压保持，conveyor 继续送球。
            intakeRotater.moveToHandoff();
            intakeRoller.hold();
            conveyor.feedToShooter();
            shooter.stopFlywheel();
            shooter.stopShooterConveyor();
            shooter.holdBackboardAt(0.0);

            if (timeInState() > Constants.Superstructure.indexingSeconds) {
                conveyor.setNotePresent(true);
                setSystemState(SystemState.HOLDING_NOTE);
            }
            return;
        }

        if (systemState == SystemState.CLEANUP) {
            intakeRoller.stop();
            conveyor.stop();
            shooter.stopShooterConveyor();
            shooter.stopFlywheel();
            shooter.holdBackboardAt(0.0);
            intakeRotater.stow();

            if (timeInState() > Constants.Superstructure.cleanupSeconds) {
                setSystemState(conveyor.hasNote() ? SystemState.HOLDING_NOTE : SystemState.STOWED);
            }
            return;
        }

        // 常规空闲姿态：有球时保持球位，无球时全部停止。
        setSystemState(conveyor.hasNote() ? SystemState.HOLDING_NOTE : SystemState.STOWED);
        intakeRotater.stow();
        intakeRoller.stop();
        if (conveyor.hasNote()) {
            conveyor.hold();
        } else {
            conveyor.stop();
        }
        shooter.stopFlywheel();
        shooter.stopShooterConveyor();
        shooter.holdBackboardAt(0.0);
    }

    private void handleIntake() {
        if (systemState != SystemState.DEPLOYING_INTAKE
            && systemState != SystemState.INTAKING) {
            setSystemState(SystemState.DEPLOYING_INTAKE);
        }

        shooter.stopFlywheel();
        shooter.stopShooterConveyor();
        shooter.holdBackboardAt(0.0);
        intakeRotater.deployToGround();

        if (intakeRotater.isBlockedByNotZeroed()) {
            // 旋转机构未归零，安全逻辑拦下展开动作；
            // 滚轮和 conveyor 也不启动，等待操作员先执行归零（A 键）。
            intakeRoller.stop();
            conveyor.stop();
            return;
        }

        // 展开过程中即允许进球，位置到位后进入 INTAKING。
        // 没有球检测，收球何时结束由操作员松开按键决定（IDLE -> INDEXING）。
        intakeRoller.intake();
        conveyor.feedToShooter();

        if (systemState == SystemState.DEPLOYING_INTAKE && intakeRotater.atGoal()) {
            setSystemState(SystemState.INTAKING);
        }
    }

    private void handlePrepShoot(boolean feedWhenReady) {
        if (systemState != SystemState.SPINNING_UP
            && systemState != SystemState.AIMING
            && systemState != SystemState.READY_TO_SHOOT
            && systemState != SystemState.SHOOTING
            && systemState != SystemState.CLEANUP) {
            // 从 intake 流程直接转射击：没有传感器，默认球已吸入。
            // 未归零被安全逻辑拦下时滚轮从未启动，不可能有球，不置位。
            if ((systemState == SystemState.DEPLOYING_INTAKE
                || systemState == SystemState.INTAKING
                || systemState == SystemState.INDEXING)
                && !intakeRotater.isBlockedByNotZeroed()) {
                conveyor.setNotePresent(true);
            }
            setSystemState(SystemState.SPINNING_UP);
        }

        // 射击姿态：intake 到 HANDOFF 让出球路，滚轮低压保持球位。
        intakeRotater.moveToHandoff();
        intakeRoller.hold();

        // 距离到射速的映射由 LimelightSubsystem 提供；操作员 offset 用于现场微调，
        // 但不会绕过 Superstructure 状态机。
        double targetRps = limelight.getShooterSetpointByDistance() + flywheelSpeedOffsetRps;
        shooter.setFlywheelVelocity(targetRps, targetRps);
        shooter.holdBackboardAt(Constants.Superstructure.backboardShootPosition + backboardPositionOffset);

        if (systemState == SystemState.SHOOTING) {
            // 只有 canShoot() 通过后才会进入本状态；喂球期间不再复查门控，
            // 避免球接触飞轮导致的掉速中断喂球。
            conveyor.feedToShooter();
            shooter.runShooterConveyor(Constants.Superstructure.shooterFeedPercent);

            if (timeInState() > Constants.Superstructure.shootTimeoutSeconds) {
                conveyor.setNotePresent(false);
                setSystemState(SystemState.CLEANUP);
            }
            return;
        }

        if (systemState == SystemState.CLEANUP) {
            conveyor.stop();
            shooter.stopShooterConveyor();
            if (timeInState() > Constants.Superstructure.cleanupSeconds) {
                // 不强行改写 wantedState：清理窗口内重按/按住射击键的请求不会被吞掉，
                // 而 hasNote 已清空，门控自然挡住空喂球，只会保持飞轮转速待命。
                setSystemState(conveyor.hasNote() ? SystemState.HOLDING_NOTE : SystemState.STOWED);
            }
            return;
        }

        // 未开始喂球前，球路保持静止。
        conveyor.hold();
        shooter.stopShooterConveyor();

        // 门控条件实时刷新：条件回落时状态同步回退，仪表盘能看到卡在哪一关。
        if (canShoot()) {
            setSystemState(SystemState.READY_TO_SHOOT);
        } else if (shooter.atSpeed()) {
            setSystemState(SystemState.AIMING);
        } else {
            setSystemState(SystemState.SPINNING_UP);
        }

        if (feedWhenReady && systemState == SystemState.READY_TO_SHOOT) {
            setSystemState(SystemState.SHOOTING);
        }
    }

    private void handleEject() {
        // Eject 反转整条球路，飞轮保持停止。
        // 旋转机构未归零时 setGoal 内部会自行拦下，滚轮反转不受影响。
        setSystemState(SystemState.EJECTING);
        intakeRotater.deployToGround();
        intakeRoller.outtake();
        conveyor.reverse();
        shooter.runShooterConveyor(Constants.Superstructure.shooterReversePercent);
        shooter.stopFlywheel();
        shooter.holdBackboardAt(0.0);
    }

    private void handleManual() {
        // 手动模式：状态机不再驱动机构，交给现场手动/调参工具；
        // 只保持飞轮与喂球停止，避免意外射出。
        setSystemState(SystemState.MANUAL_OVERRIDE);
        shooter.stopFlywheel();
        shooter.stopShooterConveyor();
        conveyor.stop();
        intakeRoller.stop();
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
        // 发布状态和各项门控信号，方便判断机器人为什么还没进入射击。
        table.getEntry("wantedState").setString(wantedState.name());
        table.getEntry("systemState").setString(systemState.name());
        table.getEntry("timeInState").setDouble(timeInState());
        table.getEntry("canShoot").setBoolean(canShoot());
        table.getEntry("shooterAtSpeed").setBoolean(shooter.atSpeed());
        table.getEntry("driveAimed").setBoolean(drive.isAimed());
        table.getEntry("driveHeadingError").setDouble(drive.getHeadingError());
        table.getEntry("driveStationary").setBoolean(isDriveStationary());
        table.getEntry("intakeAtGoal").setBoolean(intakeRotater.atGoal());
        table.getEntry("intakeBlockedByNotZeroed").setBoolean(intakeRotater.isBlockedByNotZeroed());
        table.getEntry("hasNote").setBoolean(conveyor.hasNote());
        table.getEntry("flywheelSpeedOffsetRps").setDouble(flywheelSpeedOffsetRps);
        table.getEntry("backboardPositionOffset").setDouble(backboardPositionOffset);
    }
}
