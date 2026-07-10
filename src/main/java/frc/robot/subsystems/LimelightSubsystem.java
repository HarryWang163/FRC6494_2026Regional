package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Thin read-only wrapper around the existing LimelightHelpers.
 *
 * This subsystem does not fuse vision or command mechanisms; it only gives
 * Superstructure stable target and distance read APIs.
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
        // DriveControls publishes the field-pose distance under AutoControl.
        // Reuse that value to preserve the existing target calculation path.
        return NetworkTableInstance.getDefault()
            .getTable("AutoControl")
            .getEntry("distanceToHub")
            .getDouble(0.0);
    }

    public Pose2d getEstimatedPose() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName).pose;
    }

    public double getShooterVelocityByDistance() {
        return Constants.Shooter.flywheelVelocityByDistance.get(getDistanceToTarget());
    }
}
