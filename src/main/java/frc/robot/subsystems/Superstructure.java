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
 */
public class Superstructure extends SubsystemBase {
    // 操作员或自动程序请求的目标状态。
    public enum WantedState {
        IDLE,
        INTAKE,
        SHOOT_HUB,
        AIM_HUB,
        PASS_BALL,
        EJECT,
        MANUAL
    }

    // 机器人内部实际执行状态，用于动作排序和仪表盘诊断。
    public enum SystemState {
        DISABLED,
        STOWED,
        INTAKING,
        PREP_SHOOT,
        SHOOTING,
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
    private double flywheelVoltageOffset = 0.0;
    private double backboardPositionOffset = 0.0;
    private boolean shotCompletedThisRequest = false;

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

    public void requestShootHub() {
        if (wantedState != WantedState.SHOOT_HUB) {
            shotCompletedThisRequest = false;
        }
        wantedState = WantedState.SHOOT_HUB;
    }

    public void requestAimHub() {
        wantedState = WantedState.AIM_HUB;
    }

    public void requestPassBall() {
        if (wantedState != WantedState.PASS_BALL) {
            shotCompletedThisRequest = false;
        }
        wantedState = WantedState.PASS_BALL;
    }

    public void requestShoot() {
        requestShootHub();
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

    public Command requestShootHubCommand() {
        return Commands.runOnce(this::requestShootHub, this);
    }

    public Command requestAimHubCommand() {
        return Commands.runOnce(this::requestAimHub, this);
    }

    public Command requestPassBallCommand() {
        return Commands.runOnce(this::requestPassBall, this);
    }

    public Command requestShootCommand() {
        return requestShootHubCommand();
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

    public boolean isShootCycleComplete() {
        return shotCompletedThisRequest;
    }

    /* ====================== */
    /*        射击门控          */
    /* ====================== */

    public boolean canShoot() {
        boolean stationaryGate = !Constants.Superstructure.stationaryGateEnabled || isDriveStationary();
        return shooter.atVoltage()
            && drive.isAimed()
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

    public void adjustFlywheelVoltageOffset(double offsetVolts) {
        flywheelVoltageOffset += offsetVolts;
    }

    public void adjustBackboardPositionOffset(double offset) {
        backboardPositionOffset += offset;
    }

    public void resetOffsets() {
        flywheelVoltageOffset = 0.0;
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
            setSystemState(SystemState.DISABLED);
            stopAllMechanisms();
            publishTelemetry();
            return;
        }

        switch (wantedState) {
            case IDLE -> handleIdle();
            case INTAKE -> handleIntake();
            case SHOOT_HUB -> handleShootHub();
            case AIM_HUB -> handleAimHub();
            case PASS_BALL -> handlePassBall();
            case EJECT -> handleEject();
            case MANUAL -> handleManual();
        }

        publishTelemetry();
    }

    private void handleIdle() {
        // 安全默认状态：球路不动，intaker 不主动动作，背板保持受控。
        conveyor.stop();
        intakeRoller.stop();
        intakeRotater.stop();
        shooter.stopFlywheel();
        shooter.stopShooterConveyor();
        shooter.holdBackboardAt(0.0);
        setSystemState(SystemState.STOWED);
    }

    private void handleIntake() {
        shooter.stopFlywheel();
        shooter.stopShooterConveyor();
        shooter.holdBackboardAt(0.0);
        // Intaker 比赛开始后保持下放，收球何时结束由操作员松开按键决定。
        intakeRoller.intake();
        conveyor.feedToShooter();
        setSystemState(SystemState.INTAKING);
    }

    private void handleAimHub() {
        intakeRoller.stop();
        conveyor.stop();
        shooter.stopFlywheel();
        shooter.stopShooterConveyor();
        shooter.holdBackboardAt(0.0);
        setSystemState(SystemState.STOWED);
    }

    private void handleShootHub() {
        double targetVolts = limelight.getShooterVoltageByDistance() + flywheelVoltageOffset;
        double backboardPosition = Constants.Superstructure.backboardShootPosition + backboardPositionOffset;
        handlePreparedFeed(targetVolts, backboardPosition);
    }

    private void handlePassBall() {
        double targetVolts = Constants.Superstructure.passBallFlywheelVoltage + flywheelVoltageOffset;
        double backboardPosition = Constants.Superstructure.passBallBackboardPosition + backboardPositionOffset;
        handlePreparedFeed(targetVolts, backboardPosition);
    }

    private void handlePreparedFeed(double targetVolts, double backboardPosition) {
        if (systemState != SystemState.PREP_SHOOT
            && systemState != SystemState.SHOOTING) {
            setSystemState(SystemState.PREP_SHOOT);
        }
        intakeRoller.stop();
        conveyor.stop();
        shooter.stopShooterConveyor();
        // 距离到射速的映射由 LimelightSubsystem 提供；操作员 offset 用于现场微调，
        // 但不会绕过 Superstructure 状态机。
        shooter.setFlywheelVoltage(targetVolts);
        shooter.holdBackboardAt(backboardPosition);

        if (systemState == SystemState.SHOOTING) {
            // 只有 canShoot() 通过后才会进入本状态；喂球期间不再复查门控，
            // 避免球接触飞轮导致的掉速中断喂球。
            intakeRotater.shootAssist();
            conveyor.feedToShooter();
            shooter.runShooterConveyorVoltage(Constants.Superstructure.shooterFeedVoltage);

            if (timeInState() > Constants.Superstructure.shootTimeoutSeconds) {
                shotCompletedThisRequest = true;
                setSystemState(SystemState.PREP_SHOOT);
            }
            return;
        }

        // 未开始喂球前，球路保持静止。
        conveyor.stop();
        shooter.stopShooterConveyor();

        // 门控条件实时刷新：条件回落时状态同步回退，仪表盘能看到卡在哪一关。
        if (canShoot()) {
            setSystemState(SystemState.SHOOTING);
        }
    }

    private void handleEject() {
        // Eject 反转整条球路，飞轮保持停止。
        setSystemState(SystemState.EJECTING);
        intakeRoller.outtake();
        conveyor.reverse();
        shooter.runShooterConveyorVoltage(Constants.Superstructure.shooterReverseVoltage);
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
        table.getEntry("shooterAtVoltage").setBoolean(shooter.atVoltage());
        table.getEntry("driveAimed").setBoolean(drive.isAimed());
        table.getEntry("driveHeadingError").setDouble(drive.getHeadingError());
        table.getEntry("driveStationary").setBoolean(isDriveStationary());
        table.getEntry("flywheelVoltageOffset").setDouble(flywheelVoltageOffset);
        table.getEntry("backboardPositionOffset").setDouble(backboardPositionOffset);
    }
}
