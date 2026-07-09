package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class Constants {
    private Constants() {}

    public static final boolean LEDUsing = false;

    public static class Shooter {
        // 机构 CAN ID 目前是占位值；季后赛最终接线确认后，只需要改这里。
        public static final int leftFlywheelID = 38;
        public static final int rightFlywheelID = 39;
        public static final int leftConveyorID = 40;
        public static final int rightConveyorID = 37;
        public static final int backboardMotorID = 44;

        public static final double flywheelNominalKv = 0.12;
        public static final double defaultFlywheelVoltage = 3.34;
        public static final double defaultFlywheelVelocity = defaultFlywheelVoltage / flywheelNominalKv;
        public static final double flywheelMaxVelocity = 120.0;
        public static final double flywheelReadyVelocityThreshold = 5.0;
        public static final double flywheelReadyVelocityTolerance = 2.0;
        public static final double flywheelReadyDelaySeconds = 0.5;

        // 距离(米) -> 飞轮电压(V) 射表。当前为占位点；
        // 真车试射后按实测数据加点即可，查表自动线性插值并在两端取边界值。
        public static final InterpolatingDoubleTreeMap flywheelVoltageByDistance = new InterpolatingDoubleTreeMap();
        static {
            flywheelVoltageByDistance.put(1.2, 2.0);
            flywheelVoltageByDistance.put(2.5, 2.1);
            flywheelVoltageByDistance.put(3.5, 2.2);
            flywheelVoltageByDistance.put(5.0, 2.3);
        }

        public static final InterpolatingDoubleTreeMap flywheelVelocityByDistance = new InterpolatingDoubleTreeMap();
        static {
            flywheelVelocityByDistance.put(1.2, 2.0 / flywheelNominalKv);
            flywheelVelocityByDistance.put(2.5, 2.1 / flywheelNominalKv);
            flywheelVelocityByDistance.put(3.5, 2.2 / flywheelNominalKv);
            flywheelVelocityByDistance.put(5.0, 2.3 / flywheelNominalKv);
        }

        public static final double shooterConveyorFeedVoltage = 3.0;
        public static final double shooterConveyorReverseVoltage = -2.0;
        public static final double shooterConveyorNominalKv = 0.125;
        public static final double shooterConveyorMaxVelocity = 12.0 / shooterConveyorNominalKv;
        public static final double shooterConveyorFeedVelocity =
            shooterConveyorFeedVoltage / shooterConveyorNominalKv;
        public static final double shooterConveyorReverseVelocity =
            shooterConveyorReverseVoltage / shooterConveyorNominalKv;

        public static final Slot0Configs flyWheelSlot0Configs = new Slot0Configs()
            .withKP(0.21)
            .withKI(0)
            .withKD(0.004)
            .withKS(0.25)
            .withKA(0.0)
            .withKV(flywheelNominalKv);

        public static final Slot0Configs conveyorSlot0Configs = new Slot0Configs()
            .withKP(0.19).withKI(0).withKD(0.001)
            .withKV(shooterConveyorNominalKv).withKA(0.0).withKS(0.07);

        public static final Slot0Configs backboardSlot0Configs = new Slot0Configs()
            .withKP(0.01).withKI(0).withKD(0)
            .withKV(0.03).withKA(0.0).withKS(0.010);

        public static class backboardPositionPID {
            public static final double kP = 0.007;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static final double backboardSpeedMax = 3.0;
        public static final double backboardSpeedMin = 0;
        public static final double backboardGravityFeedforwardMinVolts = 0.1;
        public static final double backboardGravityFeedforwardMaxVolts = 0.3;
        public static final double backboardGravityFeedforwardMinPosition = 0.0;
        public static final double backboardGravityFeedforwardMaxPosition = 2200.0;
        public static final double backboardGravityFeedforwardDirection = 1.0;
        public static final double backboardUpLimit = 2200.0;
        public static final double backboardDownLimit = 0.0;
    }

    public static class AutoPositioning {
        public static final double autoRotationkP = 0.07;
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
        public static final String LIMELIGHT_NAME_Shooter = "limelight-left";
        public static final String LIMELIGHT_NAME_Intaker = "limelight-front";
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
        public static final int leftIntakeRollerID = 20;
        public static final int rightIntakeRollerID = 47;
        public static final int leftIntakeRotaterID = 48;
        public static final int rightIntakeRotaterID = 43;

        public static final double intakeRollerIntakeVoltage = -6.0;
        public static final double intakeRollerOuttakeVoltage = 3.0;

        public static final double intakeRotaterRaiseVoltage = 0.8;
        public static final double intakeRotaterLowerVoltage = 0.05;

        public static final Slot0Configs intakeRollerSlot0Configs = new Slot0Configs()
            .withKP(0.00).withKI(0.00).withKD(0.00)
            .withKV(0.00).withKA(0.00).withKS(0.00);
        public static final Slot0Configs intakeGetterSlot0Configs = intakeRollerSlot0Configs;

        public static final Slot0Configs intakeRotaterSlot0Configs = new Slot0Configs()
            .withKP(0.00).withKI(0.00).withKD(0.00)
            .withKV(0.00).withKA(0.00).withKS(0.00);

        public static final double IntakeGetterSpeedforAuto = 0.5;
        // 这里给出可用的占位值，真车调好位置闭环后再收紧。
    }

    public static class Conveyor {
        public static final int mainConveyorID = 42;
        public static final double feedVoltage = 2.5;
        public static final double reverseVoltage = -2.0;

        public static final Slot0Configs mainConveyorSlot0Configs = new Slot0Configs()
            .withKP(0.00).withKI(0.00).withKD(0.00)
            .withKV(0.00).withKA(0.00).withKS(0.00);
    }

    public static class Superstructure {
        // 状态机时间和门控容差。真车基础动作验证后，需要现场调这些值。
        public static final double shooterFeedVoltage = Shooter.shooterConveyorFeedVoltage;
        public static final double shooterReverseVoltage = Shooter.shooterConveyorReverseVoltage;
        public static final double shooterFeedVelocity = Shooter.shooterConveyorFeedVelocity;
        public static final double shooterReverseVelocity = Shooter.shooterConveyorReverseVelocity;
        public static final double shootTimeoutSeconds = 1.0;
        public static final double aimToleranceDegrees = 1.5;
        public static final double stationarySpeedToleranceMetersPerSecond = 0.5;
        public static final double passBallFlywheelVoltage = Shooter.defaultFlywheelVoltage;
        public static final double passBallFlywheelVelocity = Shooter.defaultFlywheelVelocity;
        public static final double passBallBackboardPosition = 175.0;

        // 射击时背板的目标位置（外接编码器计数）。
        public static final double backboardShootPosition = 175.0;
        public static final double backboardReadyPositionTolerance = 100.0;
        // 射击门控是否要求底盘接近静止。
        public static final boolean stationaryGateEnabled = true;
    }
}
