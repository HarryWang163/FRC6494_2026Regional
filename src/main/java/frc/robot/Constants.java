package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Pose2d;

public final class Constants {
    public static final boolean LEDUsing = false;
    public enum RobotStatus{
        Stopped,
        AllTelop,
        PassingBall,
        AutoAimming,
        Climbing,
        CrossingBump,
        CrossingTrench
    }
    public class Shooter {
        public static final double conveyorSpeed = 25;
        public static final Slot0Configs flyWheelSlot0Configs = new Slot0Configs()
            .withKP(0.0).withKI(0).withKD(0)
            .withKV(0.099).withKA(0.0).withKS(0.0);

        public static final Slot0Configs conveyorSlot0Configs = new Slot0Configs()
            .withKP(0.0).withKI(0).withKD(0)
            .withKV(0.16).withKA(0.15).withKS(0);

        public static final Slot0Configs backboardSlot0Configs = new Slot0Configs()
            .withKP(0.01).withKI(0).withKD(0)
            .withKV(0.008).withKA(0.0).withKS(0.010);

        public class backboardPositionPID {
            public static final double kP = 0.1;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            
        }
        public static final double backboardSpeedMax = 30; //也就是output
        // 背板的上下限位角度
        public static final double backboardUpLimit = 3000.0;  // 背板上限角度（根据实际需求设置）
        public static final double backboardDownLimit = 0.0;  // 背板下限角度（根据实际需求设置）
    }
    public class AutoPositioning {
        public static final double autoRotationkP = 0.1;
        public static final double autoRotationForBumpTargetDegrees = 45;
        public static final double autoRotationForTrenchTargetDegrees = 0;
        public static final boolean teleopOffsetEnabled = true;
        public static final double teleopOffsetkP = 15;

        public static final double autoPositioningkP = 1.0;
        public static final double autoPositioningAngleError = 1.0; // 角度误差容忍度，单位为度

        public static final double[] bumpY = {2.51,5.556};
        public static final double[] trenchY = {0.639,7.43};

        public static final double TrenchtargetAngle = 0;
        public static final double TurningkP = 1.5;
    }

    public class StartingPoints{
        public class Red{
            public static final Pose2d Point1 = new Pose2d(12.93, 7.47, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
            public static final Pose2d Point2 = new Pose2d(14.314, 4.15, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-180));
        }
        
    }

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        SYSID,
        OTHER
    }
    public class Pigeon {
        public static final int CANID = 0;
    }

    public class Limelight {
        public static final String LIMELIGHT_NAME_Shooter = "limelight-shooter";
        public static final String LIMELIGHT_NAME_Intaker = "limelight-intaker";
        public static final Boolean UsingMetaTag2 = true;
        public static final double AutoClimbAlignSpeed = 0.5;//等查资料
        public static final double AutoClimbAlignTolerance = 0.1;//？
        public static final double AutoClimbmaxAngularVelocity  = 0.5;//等查资料
    }

    public class Field {
        public static final double RedHubPositionX = 11.914;
        public static final double RedHubPositionY = 4.034;
        public static final double BlueHubPositionX = 4.625;
        public static final double BlueHubPositionY = 4.034;
        public static final double PassingBallPosY1 = 1.405;
        public static final double PassingBallPosY2 = 6.664;
        public static final double RedPassingBallPosX = 13.355;
        public static final double BluePassingBallPosX = 3.184;
    }
    

    public class Intaker {

    public static final double intakeRotaterUpLimit = 0.0;  // rotator上限角度
    public static final double intakeRotaterDownLimit = -7.6;  // rotator下限角度
    public static final double intakeRotaterVelocity = 5; // rotater的速度（如果你用速度控制的话）
    public static final double IntakeRotaterGravityFF = 0.0; // 重力前馈增益（需要实测调整）

    public static final Slot0Configs intakeGetterSlot0Configs = new Slot0Configs()
        .withKP(0.00).withKI(0.00).withKD(0.00)
        .withKV(0.00).withKA(0.00).withKS(0.00);
    // getter 如果你只用固定速度开关，也可以先不启闭环；若用闭环速度，这里 kP/kV/KS 等需要实测调参(这句fromAI)

    public static final Slot0Configs intakeRotaterSlot0Configs = new Slot0Configs()
        .withKP(0.00).withKI(0.00).withKD(0.00)
        .withKV(0.00).withKA(0.00).withKS(0.00);
    //如果 rotater 受重力影响明显，可能需要用 withKG/withGravityType/withGravityArmPositionOffset（看机构形式）(这句fromAI)

    public static final double IntakeGetterSpeedforAuto = 0.4;
    // 单位RPS
    public static final double positionToleranceRotations = 0.00;
    //rotater容差
    public static final double IntakeGetterK = 0.3; // intake getter速度与触发器输入的比例
    public static final double Getterminspeed = 0.3; // intake getter的基础速度（即触发器输入为0时的速度，正值表示默认向内转）
    }   
    public static class Climber {

    public static final int motorID = 49;

    public static final boolean inverted = false;

    public static double climbUpTicks = 94; // TODO: 实测后调整
    public static double climbDownTicks = 0; // TODO: 实测后调整
    public static final double climbVelocity = -200;
    public static final double holdToleranceRot = 0.5; 
    public static final double deltaDownTicks = 1;

    public static final Slot0Configs slot0Configs = new Slot0Configs();

    static {
        // TODO: PID/F 值你们实测后填
        slot0Configs.kP = 0.2;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;

        // TODO: 如果要前馈（速度闭环一般建议有 kV）
        slot0Configs.kS = 0.0;
        slot0Configs.kV = 0.1;
        slot0Configs.kA = 0.0;
    }
}
}