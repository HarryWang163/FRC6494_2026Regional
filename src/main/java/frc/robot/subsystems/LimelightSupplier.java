package frc.robot.subsystems;

import frc.robot.Constants;

import java.lang.annotation.Target;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;

public class LimelightSupplier extends SubsystemBase {

    public LimelightSupplier() {
        setPipeline(8);
    }

    public static double getTX() {
        return LimelightHelpers.getTX(Constants.Limelight.LIMELIGHT_NAME_Intaker);
    }

    public static double getTY() {
        return LimelightHelpers.getTY(Constants.Limelight.LIMELIGHT_NAME_Intaker);
    }

    public static double getTA() {
        // Logic to get area from Limelight
        return LimelightHelpers.getTA(Constants.Limelight.LIMELIGHT_NAME_Intaker);
    }
    
    public static boolean isTargetVisible() {
        if(!LimelightHelpers.getTV(Constants.Limelight.LIMELIGHT_NAME_Intaker)){
            return false;
        }
        // Logic to check if a target is visible
        return LimelightHelpers.getTV(Constants.Limelight.LIMELIGHT_NAME_Intaker);
    }
    public static int getAprilTagID(){
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(Constants.Limelight.LIMELIGHT_NAME_Intaker);
        int id = -1;
        for (RawFiducial fiducial : fiducials) {
            id = fiducial.id;                    // 标签ID
        }   
        return id;
    }
    public static void setPipeline(int pipeline) {
        if(((int)LimelightHelpers.getCurrentPipelineIndex(Constants.Limelight.LIMELIGHT_NAME_Intaker)) != pipeline){
            LimelightHelpers.setPipelineIndex(Constants.Limelight.LIMELIGHT_NAME_Intaker, pipeline);
        }
    }

    public static double[] getTargetPose() {
        // Logic to get the target pose from Limelight
        return LimelightHelpers.getBotPose_TargetSpace(Constants.Limelight.LIMELIGHT_NAME_Intaker);
    }

    public static Pose2d getBotPose2d_wpiBlue(){
        return LimelightHelpers.getBotPose2d_wpiBlue(Constants.Limelight.LIMELIGHT_NAME_Intaker);
    }
    public static double getTargetRotationY(){
        if(isTargetVisible()){
            return getTargetPose()[4];
        }
        else{
            return 0.00;
        }
    }
    public static double getTargetTZ(){
        if(isTargetVisible()){
            return getTargetPose()[2];
        }
        else{
            return 0.00;
        }
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LimelightPipeIndex", LimelightHelpers.getCurrentPipelineIndex(Constants.Limelight.LIMELIGHT_NAME_Intaker));
        SmartDashboard.putString("LimelightPipeType", LimelightHelpers.getCurrentPipelineType(Constants.Limelight.LIMELIGHT_NAME_Intaker));
        if (isTargetVisible()){
            SmartDashboard.putString("LimelightTargetVisible", "YES");
            SmartDashboard.putNumber("LimelightTA", getTA());
            SmartDashboard.putNumber("TargetRY", getTargetRotationY());
            SmartDashboard.putNumber("TargetTX", getTX());
            SmartDashboard.putNumber("TargetTZ", getTargetTZ());
            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(Constants.Limelight.LIMELIGHT_NAME_Intaker);
            int id = -1;
            for (RawFiducial fiducial : fiducials) {
                id = fiducial.id;                    // 标签ID
            }
            SmartDashboard.putNumber("LimelightTargetID", id);
            
        }
        else {
            SmartDashboard.putString("LimelightTargetVisible", "NO");
            SmartDashboard.putNumber("TargetRY", 0);
            SmartDashboard.putNumber("TargetTX", 0);
            SmartDashboard.putNumber("TargetTZ", 0);
            SmartDashboard.putNumber("LimelightTargetID", -1);
        }
        
    }
}
