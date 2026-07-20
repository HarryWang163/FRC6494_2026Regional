package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
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
        INIT,
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
        INITIALIZING,
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
    private double flywheelVelocityOffset = 0.0;
    private double backboardPositionOffset = 0.0;
    private boolean shotCompletedThisRequest = false;
    private double postShootIdleAssistStartTimestamp = Double.NaN;

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
        if (systemState == SystemState.SHOOTING) {
            postShootIdleAssistStartTimestamp = Timer.getFPGATimestamp();
        }
        wantedState = WantedState.IDLE;
    }

    public void requestInit() {
        wantedState = WantedState.INIT;
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

    public Command requestInitCommand() {
        return Commands.sequence(
            Commands.runOnce(this::requestInit),
            intakeRotater.lowerForMatchCommand(),
            Commands.runOnce(this::requestIntake)
        );
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
        return shooter.atVelocity()
            && drive.isAimed()
            && isBackboardReady();
    }

    private boolean isDriveStationary() {
        ChassisSpeeds speeds = drive.getChassisSpeeds();
        double translationSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return translationSpeed <= Constants.Superstructure.stationarySpeedToleranceMetersPerSecond;
    }

    private boolean isBackboardReady() {
        return shooter.isBackboardNearTarget(Constants.Superstructure.backboardReadyPositionTolerance);
    }

    /* ====================== */
    /*      现场微调 offset     */
    /* ====================== */

    public void adjustFlywheelVelocityOffset(double offsetVelocity) {
        flywheelVelocityOffset += offsetVelocity;
    }

    public void adjustFlywheelVoltageOffset(double offsetVolts) {
        adjustFlywheelVelocityOffset(offsetVolts / Constants.Shooter.flywheelNominalKv);
    }

    public void adjustBackboardPositionOffset(double offset) {
        backboardPositionOffset += offset;
    }

    public void resetOffsets() {
        flywheelVelocityOffset = 0.0;
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
            case INIT -> handleInit();
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

    private void handleInit() {
        if (systemState != SystemState.INITIALIZING) {
            setSystemState(SystemState.INITIALIZING);
        }

        stopMainConveyorUnlessTuning();
        intakeRoller.stop();
        stopFlywheelUnlessTuning();
        stopShooterConveyorUnlessTuning();
        holdBackboardStowedUnlessTuning();
    }

    private void handleIdle() {
        // 安全默认状态：球路不动，intaker 不主动动作，背板保持受控。
        stopMainConveyorUnlessTuning();
        intakeRoller.stop();
        if (!runPostShootIdleAssist()) {
            intakeRotater.stop();
        }
        stopFlywheelUnlessTuning();
        stopShooterConveyorUnlessTuning();
        holdBackboardStowedUnlessTuning();
        setSystemState(SystemState.STOWED);
    }

    private boolean runPostShootIdleAssist() {
        if (Double.isNaN(postShootIdleAssistStartTimestamp)) {
            return false;
        }

        double elapsed = Timer.getFPGATimestamp() - postShootIdleAssistStartTimestamp;
        if (elapsed >= 0.6) {
            postShootIdleAssistStartTimestamp = Double.NaN;
            return false;
        }

        intakeRotater.postShootIdleAssist(elapsed);
        return true;
    }

    private void handleIntake() {
        stopFlywheelUnlessTuning();
        stopShooterConveyorUnlessTuning();
        holdBackboardStowedUnlessTuning();
        // Intaker 比赛开始后保持下放，收球何时结束由操作员松开按键决定。
        intakeRoller.intake();
        intakeFeedMainConveyorUnlessTuning();
        setSystemState(SystemState.INTAKING);
    }

    private void handleAimHub() {
        intakeRoller.stop();
        stopMainConveyorUnlessTuning();
        stopFlywheelUnlessTuning();
        stopShooterConveyorUnlessTuning();
        holdBackboardStowedUnlessTuning();
        setSystemState(SystemState.STOWED);
    }

    private void handleShootHub() {
        double x = NetworkTableInstance.getDefault().getTable("AutoControl").getEntry("distanceToHub").getDouble(0.0);
        x = MathUtil.clamp(x, 1.2, 4.5);
        double rawtargetVelocity = -0.9702766369910669*x*x*x*x*x*x+16.17984114722666*x*x*x*x*x-107.89020547785285*x*x*x*x+365.6895126610877*x*x*x-659.1553030522557*x*x+598.8266122952737*x-188.96203796840197;
        double targetVelocity = rawtargetVelocity + flywheelVelocityOffset;
        double rawtargetbackboardPosition = +72.63723930800967*x*x*x*x*x*x-1226.7072434356646*x*x*x*x*x+8380.732539436454*x*x*x*x-29585.06108203164*x*x*x+56753.91414329895*x*x-55469.56099616276*x+21418.08191862802;
        rawtargetbackboardPosition = MathUtil.clamp(rawtargetbackboardPosition, 0, 2200);
        double backboardPosition = rawtargetbackboardPosition + backboardPositionOffset;
        handlePreparedFeed(targetVelocity, backboardPosition);
    }

    private void handlePassBall() {
        double targetVelocity = 30;
        double backboardPosition = 2000;
        handlePreparedFeed(targetVelocity, backboardPosition);
    }

    private void handlePreparedFeed(double targetVelocity, double backboardPosition) {
        if (systemState != SystemState.PREP_SHOOT
            && systemState != SystemState.SHOOTING) {
            setSystemState(SystemState.PREP_SHOOT);
        }
        // 距离到射速的映射由 LimelightSubsystem 提供；操作员 offset 用于现场微调，
        // 但不会绕过 Superstructure 状态机。
        shooter.setFlywheelVelocity(targetVelocity);
        shooter.holdBackboardAt(backboardPosition);

        if (systemState == SystemState.SHOOTING) {
            // 只有 canShoot() 通过后才会进入本状态；喂球期间不再复查门控，
            // 避免球接触飞轮导致的掉速中断喂球。
            intakeRotater.shootAssist();
            intakeRoller.intakeforshoot();
            feedMainConveyorUnlessTuning();
            runShooterConveyorVelocityUnlessTuning(Constants.Superstructure.shooterFeedVelocity);
            return;
        }

        // 未开始喂球前，球路保持静止。
        stopMainConveyorUnlessTuning();
        stopShooterConveyorUnlessTuning();
        intakeRoller.stop();

        // Keep normal gating, but force the shot if prep takes too long.
        if (canShoot() || prepShootFallbackElapsed()) {
            setSystemState(SystemState.SHOOTING);
        }
    }

    private boolean prepShootFallbackElapsed() {
        return systemState == SystemState.PREP_SHOOT
            && timeInState() >= Constants.Superstructure.prepShootFallbackDelaySeconds;
    }

    private void handleEject() {
        // Eject 反转整条球路，飞轮保持停止。
        setSystemState(SystemState.EJECTING);
        intakeRoller.outtake();
        reverseMainConveyorUnlessTuning();
        runShooterConveyorVelocityUnlessTuning(Constants.Superstructure.shooterReverseVelocity);
        stopFlywheelUnlessTuning();
        holdBackboardStowedUnlessTuning();
    }

    private void holdBackboardStowedUnlessTuning() {
        if (!shooter.isBackboardPositionTuningActive()) {
            shooter.holdBackboardAt(0.0);
        }
    }

    private void stopFlywheelUnlessTuning() {
        if (!shooter.isFlywheelTuningControlActive()) {
            shooter.stopFlywheel();
        }
    }

    private void runShooterConveyorVelocityUnlessTuning(double targetVelocity) {
        if (!shooter.isShooterConveyorTuningControlActive()) {
            shooter.runShooterConveyorVelocity(targetVelocity);
        }
    }

    private void stopShooterConveyorUnlessTuning() {
        if (!shooter.isShooterConveyorTuningControlActive()) {
            shooter.stopShooterConveyor();
        }
    }

    private void feedMainConveyorUnlessTuning() {
        if (!conveyor.isMainConveyorTuningControlActive()) {
            conveyor.feedToShooter();
        }
    }

    private void intakeFeedMainConveyorUnlessTuning() {
        if (!conveyor.isMainConveyorTuningControlActive()) {
            conveyor.feedForIntake();
        }
    }

    private void reverseMainConveyorUnlessTuning() {
        if (!conveyor.isMainConveyorTuningControlActive()) {
            conveyor.reverse();
        }
    }

    private void stopMainConveyorUnlessTuning() {
        if (!conveyor.isMainConveyorTuningControlActive()) {
            conveyor.stop();
        }
    }

    private void handleManual() {
        // 手动模式：状态机不再驱动机构，交给现场手动/调参工具；
        // 只保持飞轮与喂球停止，避免意外射出。
        setSystemState(SystemState.MANUAL_OVERRIDE);
        stopFlywheelUnlessTuning();
        stopShooterConveyorUnlessTuning();
        stopMainConveyorUnlessTuning();
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
            if (nextState == SystemState.SHOOTING) {
                shooter.setBackboardBrakeMode();
            }
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
        table.getEntry("shooterAtVelocity").setBoolean(shooter.atVelocity());
        table.getEntry("driveAimed").setBoolean(drive.isAimed());
        table.getEntry("driveHeadingError").setDouble(drive.getHeadingError());
        table.getEntry("driveStationary").setBoolean(isDriveStationary());
        table.getEntry("backboardReady").setBoolean(isBackboardReady());
        table.getEntry("backboardPositionError").setDouble(shooter.getBackboardPositionError());
        table.getEntry("backboardReadyTolerance").setDouble(Constants.Superstructure.backboardReadyPositionTolerance);
        table.getEntry("flywheelVelocityOffset").setDouble(flywheelVelocityOffset);
        table.getEntry("flywheelTuningControlActive").setBoolean(shooter.isFlywheelTuningControlActive());
        table.getEntry("shooterConveyorTuningControlActive")
            .setBoolean(shooter.isShooterConveyorTuningControlActive());
        table.getEntry("mainConveyorTuningControlActive")
            .setBoolean(conveyor.isMainConveyorTuningControlActive());
        table.getEntry("flywheelVoltageOffset").setDouble(flywheelVelocityOffset * Constants.Shooter.flywheelNominalKv);
        table.getEntry("backboardPositionOffset").setDouble(backboardPositionOffset);
    }
}
