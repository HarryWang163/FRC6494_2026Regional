package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveMode;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Phoenix 6 Tuner X 生成底盘的命令式封装。
 *
 * 底层 swerve、模块控制和里程计保持原结构；本类只在外层补充
 * Command-Based、PathPlanner、Limelight 融合和上层状态读取接口。
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    /* 蓝方视角下，正前方为 0 度。 */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* 红方视角下，正前方为 180 度。 */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* 记录是否已经应用过操作员视角。 */
    private boolean m_hasAppliedOperatorPerspective = false;

    // 上层状态机射击门控使用的目标朝向；正常底盘控制仍由手动驾驶掌控。
    private Rotation2d targetHeading = Rotation2d.kZero;

    private DriveMode m_driveMode = DriveMode.OTHER;

    private NetworkTable positioningNetworkTable;
    private NetworkTable ll_shooter_NT;
    private NetworkTable ll_intaker_NT;
    private NetworkTable driveNetworkTable;

    private final StructPublisher<Pose2d> posePub =
        NetworkTableInstance.getDefault()
            .getStructTopic("/Odometry/RobotPose", Pose2d.struct)
            .publish();

    private Field2d m_field;
    private Field2d shooter_ll_field;
    private Field2d intaker_ll_field;

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        m_field = new Field2d();
        shooter_ll_field = new Field2d();
        intaker_ll_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putData("PositioningShooterll", shooter_ll_field);
        SmartDashboard.putData("PositioningIntakerll", intaker_ll_field);
        LimelightHelpers.SetIMUMode(Constants.Limelight.LIMELIGHT_NAME_Shooter, 0);
        LimelightHelpers.SetIMUMode(Constants.Limelight.LIMELIGHT_NAME_Intaker, 0);
        positioningNetworkTable = NetworkTableInstance.getDefault().getTable("Positioning");
        driveNetworkTable = NetworkTableInstance.getDefault().getTable("Drive");
        ll_shooter_NT = NetworkTableInstance.getDefault().getTable("Positioning/limelight-shooter");
        ll_intaker_NT = NetworkTableInstance.getDefault().getTable("Positioning/limelight-intaker");

        configueAutoBuilder();
    }

    private void configueAutoBuilder() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Failed to load PathPlanner RobotConfig", e);
        }

        AutoBuilder.configure(
            this::getPose,                 // 当前机器人位姿。
            this::resetPose,               // 重置里程计。
            this::getRobotRelativeSpeeds,  // 机器人坐标系速度。
            (speeds, feedforwards) -> driveRobotRelative(speeds), // 按目标速度驱动机器人。
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), // X/Y 位置 PID。
                new PIDConstants(5.0, 0.0, 0.0)  // 朝向 PID。
            ),
            config,
            () -> false,
            this
        );
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().Speeds;
    }

    public ChassisSpeeds getChassisSpeeds() {
        // 给上层代码使用的速度别名；原机器人坐标系方法继续留给 PathPlanner。
        return getRobotRelativeSpeeds();
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getRotation());
    }

    public Rotation2d getRotation() {
        return getState().Pose.getRotation();
    }

    public void setTargetHeading(Rotation2d targetHeading) {
        this.targetHeading = targetHeading;
    }

    public double getHeadingError() {
        return targetHeading.minus(getRotation()).getDegrees();
    }

    public boolean isAimed() {
        // 上层状态机只读取这个门控，不直接控制 swerve 模块。
        return Math.abs(getHeadingError()) <= Constants.Superstructure.aimToleranceDegrees;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        ChassisSpeeds scaled = new ChassisSpeeds(
            speeds.vxMetersPerSecond,          // X 方向速度保持原值。
            speeds.vyMetersPerSecond,          // Y 方向速度保持原值。
            speeds.omegaRadiansPerSecond * 1.3 // 只放大旋转输出，沿用原驾驶手感。
        );

        setControl(
            new SwerveRequest.ApplyRobotSpeeds().withSpeeds(scaled)
        );
    }

    public void stop() {
        setControl(
            new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0))
        );
    }

    public void lockWheels() {
        setControl(new SwerveRequest.SwerveDriveBrake());
    }

    public void aimToTarget() {
        // 给未来命令预留的高层辅助接口，仍走现有底盘请求路径，不触碰模块底层逻辑。
        double headingErrorDegrees = getHeadingError();
        double omega = Math.toRadians(headingErrorDegrees) * Constants.AutoPositioning.TurningkP;
        setControl(
            new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, omega))
        );
    }

    private static final double kMaxOmegaDegPerSec = 360.0;

    /**
     * 返回一个持续下发指定 swerve 请求的命令。
     *
     * @param request 生成待下发请求的函数。
     * @return 可调度执行的 Command。
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier, Supplier<DriveMode> driveModeSupplier) {
        return run(() -> {
            DriveMode mode = driveModeSupplier.get();
            m_driveMode = mode;
            this.setControl(requestSupplier.get());
        });
    }

    public void forceUsingLimelightmt2() {
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight.LIMELIGHT_NAME_Shooter);

        if (mt2 == null || mt2.tagCount == 0) {
            return;
        }

        var stdDevs = VecBuilder.fill(0.7, 0.7, 9999999);
        addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);
    }

    public void forceUsingLimelightmt2WithllName(String llString) {
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llString);

        if (mt2 == null || mt2.tagCount == 0) {
            return;
        }

        var stdDevs = VecBuilder.fill(0.7, 0.7, 9999999);
        addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);
    }

    private void fuseLimelightmt2() {
        Pose2d current = getState().Pose;

        LimelightHelpers.PoseEstimate mt2_shooter =
            getLimelightPoseEstimate(Constants.Limelight.LIMELIGHT_NAME_Shooter);

        LimelightHelpers.PoseEstimate mt2_intaker =
            getLimelightPoseEstimate(Constants.Limelight.LIMELIGHT_NAME_Intaker);

        boolean shooterAvailable = ifLimelightPositioningAvailable(mt2_shooter, current);
        boolean intakerAvailable = ifLimelightPositioningAvailable(mt2_intaker, current);
        ll_shooter_NT.getEntry("available").setBoolean(shooterAvailable);
        ll_intaker_NT.getEntry("available").setBoolean(intakerAvailable);
        ll_shooter_NT.getEntry("TagCount").setNumber(mt2_shooter != null ? mt2_shooter.tagCount : 0);
        ll_intaker_NT.getEntry("TagCount").setNumber(mt2_intaker != null ? mt2_intaker.tagCount : 0);

        LimelightHelpers.PoseEstimate mt2_final = null;

        if (!shooterAvailable && !intakerAvailable) {
            positioningNetworkTable.getEntry("MetaTag2Available").setBoolean(false);
            positioningNetworkTable.getEntry("Usingll").setString("None");
            // 未使用视觉时不发布距离差，避免仪表盘误读旧值。
            return;
        } else if (shooterAvailable && !intakerAvailable) {
            double shooterDistance = current.getTranslation().getDistance(mt2_shooter.pose.getTranslation());
            ll_shooter_NT.getEntry("Distance").setDouble(shooterDistance);
            ll_shooter_NT.getEntry("X").setDouble(mt2_shooter.pose.getX());
            ll_shooter_NT.getEntry("Y").setDouble(mt2_shooter.pose.getY());
            positioningNetworkTable.getEntry("MetaTag2Available").setBoolean(true);
            positioningNetworkTable.getEntry("Usingll").setString("Shooter");
            // 单个 Limelight 可用时直接采用该测量。
            mt2_final = mt2_shooter;
        } else if (!shooterAvailable && intakerAvailable) {
            double intakerDistance = current.getTranslation().getDistance(mt2_intaker.pose.getTranslation());
            ll_intaker_NT.getEntry("Distance").setDouble(intakerDistance);
            ll_intaker_NT.getEntry("X").setDouble(mt2_intaker.pose.getX());
            ll_intaker_NT.getEntry("Y").setDouble(mt2_intaker.pose.getY());
            positioningNetworkTable.getEntry("MetaTag2Available").setBoolean(true);
            positioningNetworkTable.getEntry("Usingll").setString("Intaker");
            // 单个 Limelight 可用时直接采用该测量。
            mt2_final = mt2_intaker;
        } else {
            // 两个 Limelight 都可用时，先选择与当前位姿更接近的测量。
            double shooterDistance = current.getTranslation().getDistance(mt2_shooter.pose.getTranslation());
            double intakerDistance = current.getTranslation().getDistance(mt2_intaker.pose.getTranslation());
            double difference = mt2_shooter.pose.getTranslation().getDistance(mt2_intaker.pose.getTranslation());

            ll_shooter_NT.getEntry("Distance").setDouble(shooterDistance);
            ll_intaker_NT.getEntry("Distance").setDouble(intakerDistance);
            ll_shooter_NT.getEntry("X").setDouble(mt2_shooter.pose.getX());
            ll_shooter_NT.getEntry("Y").setDouble(mt2_shooter.pose.getY());
            ll_intaker_NT.getEntry("X").setDouble(mt2_intaker.pose.getX());
            ll_intaker_NT.getEntry("Y").setDouble(mt2_intaker.pose.getY());
            positioningNetworkTable.getEntry("MetaTag2DistanceDifference").setDouble(difference);
            positioningNetworkTable.getEntry("MetaTag2XDifference").setDouble(Math.abs(mt2_shooter.pose.getX() - mt2_intaker.pose.getX()));
            positioningNetworkTable.getEntry("MetaTag2YDifference").setDouble(Math.abs(mt2_shooter.pose.getY() - mt2_intaker.pose.getY()));
            positioningNetworkTable.getEntry("Usingll").setString(shooterDistance <= intakerDistance ? "ChosenShooter" : "ChosenIntaker");

            mt2_final = shooterDistance <= intakerDistance ? mt2_shooter : mt2_intaker;
        }

        Pose2d mt2Pose = mt2_final.pose;

        double distance = current.getTranslation().getDistance(mt2Pose.getTranslation());
        Rotation2d deltaRotation = current.getRotation().minus(mt2Pose.getRotation());

        positioningNetworkTable.getEntry("CurrentPoseX").setDouble(current.getX());
        positioningNetworkTable.getEntry("CurrentPoseY").setDouble(current.getY());
        positioningNetworkTable.getEntry("MetaTag2PoseX").setDouble(mt2Pose.getX());
        positioningNetworkTable.getEntry("MetaTag2PoseY").setDouble(mt2Pose.getY());
        positioningNetworkTable.getEntry("PoseDistance").setDouble(distance);
        positioningNetworkTable.getEntry("RotationDifference").setDouble(Math.abs(deltaRotation.getDegrees()));

        // 视觉测量噪声：theta 基本不信任，只融合平移。
        // 后续可以根据 mt2.avgTagDist / mt2.tagSpan 动态调整。
        var stdDevs = VecBuilder.fill(0.7, 0.7, 9999999);

        // 融合视觉测量，注意 timestampSeconds 的时间基转换。
        addVisionMeasurement(mt2_final.pose, mt2_final.timestampSeconds, stdDevs);
        positioningNetworkTable.getEntry("MetaTag2Available").setBoolean(true);
    }

    private LimelightHelpers.PoseEstimate getLimelightPoseEstimate(String limelightName) {
        if (Constants.Limelight.UsingMetaTag2) {
           return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        }
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    }

    private boolean ifLimelightPositioningAvailable(LimelightHelpers.PoseEstimate mt2, Pose2d currentPose) {
        // 没看到 tag 时拒绝该帧视觉定位。
        if (mt2 == null || mt2.tagCount == 0) {
            return false;
        }

        // 旋转太快时拒绝该帧，避免高速转动时的视觉跳变。
        double omegaDegPerSec = getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        if (Math.abs(omegaDegPerSec) > kMaxOmegaDegPerSec) {
            return false;
        }

        return true;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void periodic() {
        Pose2d pose = getState().Pose;
        m_field.setRobotPose(pose);
        posePub.set(pose);
        shooter_ll_field.setRobotPose(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight.LIMELIGHT_NAME_Shooter).pose);
        intaker_ll_field.setRobotPose(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight.LIMELIGHT_NAME_Intaker).pose);
        driveNetworkTable.getEntry("YawFromState").setDouble(getState().Pose.getRotation().getDegrees());
        driveNetworkTable.getEntry("DriveMode").setString(m_driveMode.name());
        driveNetworkTable.getEntry("IsFieldCentric").setBoolean(m_driveMode == DriveMode.FIELD_CENTRIC);

        // 保留 Positioning 表中的基础状态输出。
        positioningNetworkTable.getEntry("interestPointsCl").setBoolean(false);

        // 每帧向 Limelight 写入当前 yaw，供 MegaTag2 使用。
        LimelightHelpers.SetRobotOrientation(
            Constants.Limelight.LIMELIGHT_NAME_Shooter,
            pose.getRotation().getDegrees(), // 符合 wpiBlue 坐标约定。
            0, 0, 0, 0, 0
        );
        LimelightHelpers.SetRobotOrientation(
            Constants.Limelight.LIMELIGHT_NAME_Intaker,
            pose.getRotation().getDegrees(), // 符合 wpiBlue 坐标约定。
            0, 0, 0, 0, 0
        );

        fuseLimelightmt2();

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    /**
     * 添加一帧视觉位姿测量，用于修正里程计估计。
     *
     * @param visionRobotPoseMeters 视觉测得的机器人位姿。
     * @param timestampSeconds 视觉测量时间戳，单位秒。
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * 添加一帧带标准差的视觉位姿测量。
     *
     * @param visionRobotPoseMeters 视觉测得的机器人位姿。
     * @param timestampSeconds 视觉测量时间戳，单位秒。
     * @param visionMeasurementStdDevs 视觉测量标准差，格式为 [x, y, theta]。
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
     * 从位姿缓存中读取指定时间戳的机器人位姿。
     *
     * @param timestampSeconds 位姿时间戳，单位秒。
     * @return 指定时间的位姿；缓存为空时返回 Optional.empty()。
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}
