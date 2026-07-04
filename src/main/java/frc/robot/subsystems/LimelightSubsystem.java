package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * 对现有 LimelightHelpers 的薄只读封装。
 *
 * 本类不做视觉融合，也不控制任何机构；只给 Superstructure 提供稳定的
 * 目标和距离读取接口。
 */
public class LimelightSubsystem extends SubsystemBase {
    private final String limelightName;

    public LimelightSubsystem() {
        this(Constants.Limelight.LIMELIGHT_NAME_Shooter);
    }

    public LimelightSubsystem(String limelightName) {
        this.limelightName = limelightName;
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public double getTargetYaw() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTargetPitch() {
        return LimelightHelpers.getTY(limelightName);
    }

    public double getDistanceToTarget() {
        // DriveControls 已经把基于场地位姿计算的距离发布到 AutoControl；
        // 这里复用该值，保留旧定位和目标计算逻辑。
        return NetworkTableInstance.getDefault()
            .getTable("AutoControl")
            .getEntry("distanceToHub")
            .getDouble(0.0);
    }

    public Pose2d getEstimatedPose() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName).pose;
    }

    public double getShooterSetpointByDistance() {
        // 初始测试用临时映射；飞轮和背板调好后应替换为实测射击表。
        double distance = getDistanceToTarget();
        return Constants.Shooter.defaultFlywheelTargetRps + (distance - 2.5) * 4.0;
    }
}
