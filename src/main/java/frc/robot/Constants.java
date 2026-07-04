package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
    private Constants() {}

    public static final boolean LEDUsing = false;

    public enum RobotStatus {
        Stopped,
        AllTelop,
        PassingBall,
        AutoAimming,
        CrossingBump,
        CrossingTrench
    }

    public static class Shooter {
        // 机构 CAN ID 目前是占位值；季后赛最终接线确认后，只需要改这里。
        public static final int leftFlywheelID = 41;
        public static final int rightFlywheelID = 42;
        public static final int leftConveyorID = 43;
        public static final int rightConveyorID = 45;
        public static final int backboardMotorID = 44;

        public static final double flywheelVoltsPerRps = 0.12;
        public static final double flywheelIdleVoltage = 0.0;
        public static final double shooterConveyorVoltage = 6.0;
        public static final double shooterConveyorReverseVoltage = -4.0;
        public static final double flywheelSpeedToleranceRps = 5.0;
        public static final double defaultFlywheelTargetRps = 64.0;

        public static final double conveyorSpeed = 150;
        public static final double conveyorSpeedForAuto = 10;

        public static final Slot0Configs flyWheelSlot0Configs = new Slot0Configs()
            .withKP(0.0).withKI(0).withKD(0)
            .withKV(0.099).withKA(0.0).withKS(0.0);

        public static final Slot0Configs conveyorSlot0Configs = new Slot0Configs()
            .withKP(0.0).withKI(0).withKD(0)
            .withKV(0.16).withKA(0.15).withKS(0);

        public static final Slot0Configs backboardSlot0Configs = new Slot0Configs()
            .withKP(0.01).withKI(0).withKD(0)
            .withKV(0.03).withKA(0.0).withKS(0.010);

        public static class backboardPositionPID {
            public static final double kP = 0.3;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static final double backboardSpeedMax = 60;
        public static final double backboardUpLimit = 3000.0;
        public static final double backboardDownLimit = 0.0;
    }

    public static class AutoPositioning {
        public static final double autoRotationkP = 0.08;
        public static final double autoRotationForBumpTargetDegrees = 45;
        public static final double autoRotationForTrenchTargetDegrees = 0;
        public static final boolean teleopOffsetEnabled = true;
        public static final double teleopOffsetkP = 15;

        public static final double autoPositioningkP = 1.0;
        public static final double autoPositioningAngleError = 1.0;

        public static final double[] bumpY = {2.51, 5.556};
        public static final double[] trenchY = {0.639, 7.43};

        public static final double TrenchtargetAngle = 0;
        public static final double TurningkP = 1.5;
    }

    public static class StartingPoints {
        public static class Red {
            public static final Pose2d Point1 = new Pose2d(12.93, 7.47, Rotation2d.fromDegrees(0));
            public static final Pose2d Point2 = new Pose2d(14.314, 4.15, Rotation2d.fromDegrees(-180));
        }
    }

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        SYSID,
        OTHER
    }

    public static class Pigeon {
        public static final int CANID = 0;
    }

    public static class Limelight {
        public static final String LIMELIGHT_NAME_Shooter = "limelight-shooter";
        public static final String LIMELIGHT_NAME_Intaker = "limelight-intaker";
        public static final Boolean UsingMetaTag2 = true;
        public static final int locatePipelineIndex = 0;
    }

    public static class Field {
        public static final double RedHubPositionX = 11.914;
        public static final double RedHubPositionY = 4.034;
        public static final double BlueHubPositionX = 4.625;
        public static final double BlueHubPositionY = 4.034;
        public static final double PassingBallPosY1 = 1.405;
        public static final double PassingBallPosY2 = 6.664;
        public static final double RedPassingBallPosX = 13.355;
        public static final double BluePassingBallPosX = 3.184;
    }

    public static class Intaker {
        // 进球机构 CAN ID 同样是占位值，真实 ID 确认后集中在这里修改。
        public static final int leftIntakeRollerID = 47;
        public static final int rightIntakeRollerID = 46;
        public static final int leftIntakeRotaterID = 48;
        public static final int rightIntakeRotaterID = 49;

        public static final double intakeRollerVoltage = 6.0;
        public static final double outtakeRollerVoltage = -5.0;
        public static final double holdRollerVoltage = 1.0;

        public static final double intakeRotaterStowPosition = 0.0;
        public static final double intakeRotaterGroundPosition = -7.6;
        public static final double intakeRotaterHandoffPosition = -2.5;
        public static final double intakeRotaterSafePosition = 0.0;
        public static final double intakeRotaterManualVoltage = 2.0;

        public static final double intakeRotaterUpLimit = 0.0;
        public static final double intakeRotaterDownLimit = -7.6;
        public static final double intakeRotaterVelocity = 5;
        public static final double IntakeRotaterGravityFF = 0.0;

        public static final Slot0Configs intakeGetterSlot0Configs = new Slot0Configs()
            .withKP(0.00).withKI(0.00).withKD(0.00)
            .withKV(0.00).withKA(0.00).withKS(0.00);

        public static final Slot0Configs intakeRotaterSlot0Configs = new Slot0Configs()
            .withKP(0.00).withKI(0.00).withKD(0.00)
            .withKV(0.00).withKA(0.00).withKS(0.00);

        public static final double IntakeGetterSpeedforAuto = 0.5;
        public static final double positionToleranceRotations = 0.00;
    }

    public static class Conveyor {
        public static final int mainConveyorID = 50;
        public static final double feedVoltage = 6.0;
        public static final double reverseVoltage = -5.0;
        public static final double holdVoltage = 0.8;
    }

    public static class Superstructure {
        // 状态机时间和门控容差。真车基础动作验证后，需要现场调这些值。
        public static final double shooterFeedPercent = 1.0;
        public static final double shooterReversePercent = -0.6;
        public static final double cleanupSeconds = 0.3;
        public static final double shootTimeoutSeconds = 1.0;
        public static final double aimToleranceDegrees = 1.5;
        public static final double stationarySpeedToleranceMetersPerSecond = 0.5;
    }
}
