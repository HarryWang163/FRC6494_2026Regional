package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class Constants {
    private Constants() {}

    public static final boolean LEDUsing = false;

    public static class Shooter {
        // Placeholder mechanism CAN IDs; update here after final wiring.
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

        // Distance (meters) -> flywheel velocity (rotations per second).

        public static final double shooterConveyorFeedVelocity = 40;
        public static final double shooterConveyorReverseVelocity = -10;

        public static final Slot0Configs flyWheelSlot0Configs = new Slot0Configs()
            .withKP(0.21)
            .withKI(0)
            .withKD(0.004)
            .withKS(0.25)
            .withKA(0.0)
            .withKV(flywheelNominalKv);

        public static final Slot0Configs conveyorSlot0Configs = new Slot0Configs()
            .withKP(0.19).withKI(0).withKD(0.001)
            .withKV(0.125).withKA(0.0).withKS(0.07);

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
        public static final double backboardGravityFeedforwardMinVolts = 0.2;
        public static final double backboardGravityFeedforwardMaxVolts = 0.5;
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
        // Placeholder intake CAN IDs; update here after wiring is confirmed.
        public static final int leftIntakeRollerID = 20;
        public static final int rightIntakeRollerID = 47;
        public static final int leftIntakeRotaterID = 48;
        public static final int rightIntakeRotaterID = 43;

        public static final double intakeRollerIntakeVoltage = -6.0;
        public static final double intakeRollerOuttakeVoltage = 3.0;

        public static final double intakeRotaterRaiseVoltage = 0.8;
        public static final double intakeRotaterLowerVoltage = -0.1;

        public static final Slot0Configs intakeRollerSlot0Configs = new Slot0Configs()
            .withKP(0.00).withKI(0.00).withKD(0.00)
            .withKV(0.00).withKA(0.00).withKS(0.00);
        public static final Slot0Configs intakeGetterSlot0Configs = intakeRollerSlot0Configs;

        public static final Slot0Configs intakeRotaterSlot0Configs = new Slot0Configs()
            .withKP(0.00).withKI(0.00).withKD(0.00)
            .withKV(0.00).withKA(0.00).withKS(0.00);

        public static final double IntakeGetterSpeedforAuto = 0.5;
        // Placeholder values; tighten after the real mechanism is tuned.
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
        // State-machine timing and gate tolerances; tune on the real robot.

        public static final double shooterFeedVelocity = Shooter.shooterConveyorFeedVelocity;
        public static final double shooterReverseVelocity = Shooter.shooterConveyorReverseVelocity;
        public static final double shootTimeoutSeconds = 4.0;
        public static final double aimToleranceDegrees = 1.5;
        public static final double stationarySpeedToleranceMetersPerSecond = 0.5;
        public static final double passBallFlywheelVoltage = Shooter.defaultFlywheelVoltage;
        public static final double passBallFlywheelVelocity = Shooter.defaultFlywheelVelocity;
        public static final double passBallBackboardPosition = 175.0;

        // Shoot backboard target position in external encoder counts.
        public static final double backboardShootPosition = 175.0;
        public static final double backboardReadyPositionTolerance = 100.0;
        // Whether the shooting gate requires the drivetrain to be nearly still.
        public static final boolean stationaryGateEnabled = true;
    }
}
