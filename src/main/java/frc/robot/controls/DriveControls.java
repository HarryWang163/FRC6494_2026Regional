package frc.robot.controls;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveMode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedState;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer; 

public class DriveControls {

    public boolean isRedAlliance;

    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driver;

    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(edu.wpi.first.units.Units.RadiansPerSecond);

    // 榛樿閫熷害姣斾緥锛堝钩鏃跺紑杞︼級
    private double normalDriveScale = 0.3;
    private double normalTurnScale  = 0.7;

    // 鍔犻€熸ā寮忔瘮渚嬶紙鎸変綇 RB锛?
    private double boostDriveScale = 1.0;
    private double boostTurnScale  = 1.0;
    //autoaim妯″紡
    private double slowDriveScale =0.1;

    private boolean boostEnabled = false;

    private final SlewRateLimiter vxLimiter = new SlewRateLimiter(5.0);   // m/s^2 绛夋晥锛堣皟锛?
    private final SlewRateLimiter vyLimiter = new SlewRateLimiter(5.0);
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(9.0); // rad/s^2 绛夋晥锛堣皟锛?

    // ====== 绫绘垚鍛樺彉閲忛噷鏂板锛氳褰曚笂涓€娆¤緭鍑?======
    private double lastVx = 0.0;
    private double lastVy = 0.0;
    private double lastOmega = 0.0;

    private NetworkTable autoControlNetworkTable;
    
    private final Superstructure superstructure;
    public void setBoostEnabled(boolean enabled) {
        boostEnabled = enabled;
    }

    // 鏂板锛氬綋鍓嶆槸鍚︿娇鐢ㄥ満鍦板潗鏍囩郴锛堥粯璁?true锛?
    private boolean fieldCentricEnabled = true;

    private final SwerveRequest.FieldCentric fieldCentric =
        new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.05)
            .withRotationalDeadband(maxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentric fieldCentricAuto =
    new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * 0.05)
        .withRotationalDeadband(maxAngularRate * 0.05)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance);

    private final SwerveRequest.RobotCentric robotCentric =
    new SwerveRequest.RobotCentric()
        .withDeadband(maxSpeed * 0.05)
        .withRotationalDeadband(maxAngularRate * 0.05)
        .withDriveRequestType(DriveRequestType.Velocity);
    
    public DriveControls(CommandSwerveDrivetrain drivetrain, CommandXboxController driver, Superstructure superstructure) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        this.superstructure = superstructure;
        autoControlNetworkTable = NetworkTableInstance.getDefault().getTable("AutoControl");
    }
    
    public void setFieldCentricEnabled(boolean enabled) {
        fieldCentricEnabled = enabled;
    }
    
    public boolean isFieldCentricEnabled() {
        return fieldCentricEnabled;
    }

    /** 搴曣洏鐨勯粯璁ら┚椹跺懡浠わ紙teleop 鏈熼棿鎸佺画鎵ц锛?*/
    public Command defaultDriveCommand() {
        return drivetrain.applyRequest(() -> {
        WantedState wantedState = superstructure.getWantedState();
        SmartDashboard.putString("wantedState", wantedState.name());
        double driveScale = boostEnabled ? boostDriveScale : normalDriveScale;
        double turnScale  = boostEnabled ? boostTurnScale  : normalTurnScale;
        double vx = -driver.getLeftY() * maxSpeed * driveScale;
        double vy = -driver.getLeftX() * maxSpeed * driveScale;
        double vomega = -driver.getRightX() * maxAngularRate * turnScale;
        double limitedVx = limitAccelOnly(vx, lastVx, vxLimiter);
        double limitedVy = limitAccelOnly(vy, lastVy, vyLimiter);
        double limitedOmega = limitAccelOnly(vomega, lastOmega, omegaLimiter);

        lastVx = limitedVx;
        lastVy = limitedVy;
        lastOmega = limitedOmega;

        vx = limitedVx;
        vy = limitedVy;
        vomega = limitedOmega;

        double[] hubAim = calculateDistanceAndRotationToHub();
        double[] passAim = calculateDistanceAndRotationToPassBall();
        autoControlNetworkTable.getEntry("distanceToHub").setDouble(hubAim[0]);
        autoControlNetworkTable.getEntry("angleDifferenceToHub").setDouble(hubAim[1]);
        autoControlNetworkTable.getEntry("distanceToPassball").setDouble(passAim[0]);
        autoControlNetworkTable.getEntry("angleDifferenceToPassball").setDouble(passAim[1]);

        double[] activeAim = wantedState == WantedState.PASS_BALL ? passAim : hubAim;
        drivetrain.setTargetHeading(
            drivetrain.getRotation().plus(Rotation2d.fromDegrees(activeAim[1])));

        switch (wantedState) {
            case SHOOT_HUB:
            case AIM_HUB:
                vomega = calculateRotationSpeedFromRotationAngle(hubAim[1]);
                autoControlNetworkTable.getEntry("autoRotationRate").setDouble(vomega);
                vx = -driver.getLeftY() * maxSpeed * slowDriveScale;
                vy = -driver.getLeftX() * maxSpeed * slowDriveScale;
                break;
            case PASS_BALL:
                vomega = calculateRotationSpeedFromRotationAngle(passAim[1]);
                autoControlNetworkTable.getEntry("autoRotationRate").setDouble(vomega);
                break;
            default:
                autoControlNetworkTable.getEntry("autoRotationRate").setDouble(Double.NaN);
                break;
        }

        autoControlNetworkTable.getEntry("ActualVX").setDouble(vx);
        autoControlNetworkTable.getEntry("ActualVY").setDouble(vy);
        autoControlNetworkTable.getEntry("ActualVOmega").setDouble(vomega);

        if (fieldCentricEnabled) {
        return fieldCentric
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(vomega);
        } else {
            return robotCentric
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(vomega);
        }
        }, () -> fieldCentricEnabled ? DriveMode.FIELD_CENTRIC : DriveMode.ROBOT_CENTRIC);
    }
    /**
     * 
         * 杈撳嚭绉诲姩鍒拌窛绂讳袱涓洰鏍囦箣闂磋緝杩戠殑閭ｄ釜鐐圭殑浣嶇Щ
         * @return 浣嶇Щ 鏈夋璐?
         */
    public double calculateDifferenceToTwoTarget(double current, double target1, double target2){
          double x1 = target1 - current;
          double x2 = target2 - current;
          return Math.abs(x1) < Math.abs(x2) ? x1 : x2;
    }
    public double[] calculateDistanceAndRotationToHub() {
        Pose2d currentPose = drivetrain.getState().Pose; // 鑾峰彇鏈哄櫒浜哄綋鍓嶇殑浣嶇疆鍜岃搴?
        double targetX = isRedAlliance ? Constants.Field.RedHubPositionX : Constants.Field.BlueHubPositionX;
        double targetY = isRedAlliance ? Constants.Field.RedHubPositionY : Constants.Field.BlueHubPositionY;

        // 璁＄畻鐩爣瑙掑害锛堢浉瀵逛簬鍦哄湴鍧愭爣绯伙級
        double deltaX = targetX - currentPose.getX();
        double deltaY = targetY - currentPose.getY();
        double targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX)); // 杞崲涓哄害

        // 褰撳墠鏈哄櫒浜鸿搴?
        double currentAngle = currentPose.getRotation().getDegrees();
        double angleDifference = targetAngle - currentAngle;
        // Normalize angle difference to the range [-180, 180]
        if (angleDifference > 180) {
            angleDifference -= 360;
        } else if (angleDifference < -180) {
            angleDifference += 360;
        }

        // 璁＄畻璺濈
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                // 璁＄畻璺濈鍜屾棆杞搴?
        double[] distanceAndRotation = new double[]{distance, angleDifference};

        // 杩斿洖璺濈鍜岃搴﹀樊
        return distanceAndRotation;
    }
    public double[] calculateDistanceAndRotationToPassBall() {
        Pose2d currentPose = drivetrain.getState().Pose; // 鑾峰彇鏈哄櫒浜哄綋鍓嶇殑浣嶇疆鍜岃搴?
        double targetX = isRedAlliance ? Constants.Field.RedPassingBallPosX : Constants.Field.BluePassingBallPosX;  // 鐩爣 X 鍧愭爣
        double targetY1 = Constants.Field.PassingBallPosY1;  // 鐩爣 Y 鍧愭爣
        double targetY2 = Constants.Field.PassingBallPosY2;  // 鐩爣 Y 鍧愭爣
       
        // 璁＄畻鐩爣瑙掑害锛堢浉瀵逛簬鍦哄湴鍧愭爣绯伙級
        double deltaX = targetX - currentPose.getX();
        double deltaY = calculateDifferenceToTwoTarget(currentPose.getY(), targetY1, targetY2);
        double targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX)); // 杞崲涓哄害

        // 褰撳墠鏈哄櫒浜鸿搴?
        double currentAngle = currentPose.getRotation().getDegrees();
        double angleDifference = targetAngle - currentAngle;
        // Normalize angle difference to the range [-180, 180]
        if (angleDifference > 180) {
            angleDifference -= 360;
        } else if (angleDifference < -180) {
            angleDifference += 360;
        }

        // 璁＄畻璺濈
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                // 璁＄畻璺濈鍜屾棆杞搴?
        double[] distanceAndRotation = new double[]{distance, angleDifference};

        // 杩斿洖璺濈鍜岃搴﹀樊
        return distanceAndRotation;
    }

    public double calculateRotationSpeedFromRotationAngle(double angleDifference) {
        double calDifference = angleDifference;

        // Optional: Apply teleop offset if enabled
        if (Constants.AutoPositioning.teleopOffsetEnabled) {
            double teleopOffset = -driver.getRightX();
            calDifference += teleopOffset * Constants.AutoPositioning.teleopOffsetkP;
        }

        // If angle difference is within threshold, don't rotate
        if (Math.abs(calDifference) > 1.0) {
            return calDifference  * Constants.AutoPositioning.autoRotationkP;
        } else {
            return 0;
        }
    }
    
    private double calculatevomegaFromRotationAngle(double error) {
        double vomega;
        if (Math.abs(error) < Math.toRadians(Constants.AutoPositioning.autoPositioningAngleError)) {
                vomega = 0.0;
            } else {
                vomega = error * Constants.AutoPositioning.TurningkP;
            }
            // 闄愬埗鏈€澶ц閫熷害
            vomega = Math.max(-maxAngularRate, Math.min(maxAngularRate, vomega));
        return vomega;
    }

        /**
     * 鍙檺鍒垛€滃姞閫熲€濓紙骞呭€煎彉澶э級锛屼笉闄愬埗鈥滃噺閫熲€濓紙骞呭€煎彉灏忥級
     * 杩欐牱鏉炬潌/鍒硅溅浼氭洿鍙婃椂锛屽悓鏃?boost 浠嶇劧鑳芥姂鍒剁數娴佸啿鍑?
     */
    private double limitAccelOnly(double target, double last, SlewRateLimiter limiter) {
        // 濡傛灉姝ｅ湪鍑忛€燂紙骞呭€煎彉灏忥級锛岀洿鎺ユ斁琛屽埌鐩爣鍊?
        if (Math.abs(target) < Math.abs(last)) {
        limiter.reset(target);  // 鍚屾 limiter 鍐呴儴鐘舵€侊紝闃叉涓嬩竴娆＄獊鍙?
        return target;
        }
        // 鍚﹀垯鏄姞閫燂紝浣跨敤 limiter
        return limiter.calculate(target);
    }

        /** 绱ф€ュ仠姝細娓呴浂闄愰€熷櫒锛岃閫熷害杈撳嚭绔嬪埢褰掗浂 */
    public void emergencyStop() {
        vxLimiter.reset(0.0);
        vyLimiter.reset(0.0);
        omegaLimiter.reset(0.0);

        lastVx = 0.0;
        lastVy = 0.0;
        lastOmega = 0.0;
    }

    public Command idleCommand() {
        final var idle = new SwerveRequest.Idle();
        return drivetrain.applyRequest(() -> idle).ignoringDisable(true);
    }
    
    public Command brakeWhileHeld() {
        final var brake = new SwerveRequest.SwerveDriveBrake();
        return drivetrain.applyRequest(() -> brake);
    }
    

    public SwerveRequest.SwerveDriveBrake brakeRequest() {
        return new SwerveRequest.SwerveDriveBrake();
    }

    public SwerveRequest.Idle idleRequest() {
        return new SwerveRequest.Idle();
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getMaxAngularRate() {
        return maxAngularRate;
    }
    public void setTeamColors() {
        Optional<Alliance> ally = DriverStation.getAlliance(); // 鑾峰彇闃熶紞棰滆壊
        if (ally.isPresent()) {
            // 鏍规嵁闃熶紞棰滆壊璁剧疆甯冨皵鍊?
            if (ally.get() == Alliance.Red) {
                isRedAlliance = true; // 绾㈤槦
            } else if (ally.get() == Alliance.Blue) {
                isRedAlliance = false; // 钃濋槦
            }
        } else {
            isRedAlliance = true;
        }
        if (isRedAlliance) {
            System.out.println("Robot is on the Red Alliance.");
        } else {
            System.out.println("Robot is on the Blue Alliance.");
        }
    }
    public Command getAllianceColorCommand() {
        return Commands.runOnce(this::setTeamColors);
    }
    public Command autoAimCommand() {
        return drivetrain.applyRequest(() -> {
        double[] distanceAndRotation = calculateDistanceAndRotationToHub();
        double vomega = calculateRotationSpeedFromRotationAngle(distanceAndRotation[1]);
        return fieldCentric
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(vomega);
        }, () -> DriveMode.FIELD_CENTRIC)
        .until(() -> Math.abs(calculateDistanceAndRotationToHub()[1]) < 1.5)
        .withTimeout(1.0)
        .andThen(new InstantCommand(() -> {
            drivetrain.stop();
        }, drivetrain));
    }
    public Command shakeCommand(double amplitude, double switchPeriod, double totalTime) {
    Timer timer = new Timer();

    return Commands.sequence(
        new InstantCommand(timer::restart),
        drivetrain.applyRequest(() -> {
            double t = timer.get();
            int phase = ((int) (t / switchPeriod)) % 4;

            double vx = 0.0;
            double vy = 0.0;
            double[] distanceAndRotation = calculateDistanceAndRotationToHub();
            double vomega = calculateRotationSpeedFromRotationAngle(distanceAndRotation[1]);


            switch (phase) {
                case 0:
                    vx = amplitude;   // 鍓?
                    vy = 0.0;
                    break;
                case 1:
                    vx = 0.0;
                    vy = amplitude;   // 鍙?
                    break;
                case 2:
                    vx = -amplitude;  // 鍚?
                    vy = 0.0;
                    break;
                case 3:
                    vx = 0.0;
                    vy = -amplitude;  // 宸?
                    break;
            }

            return robotCentric
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(vomega);
        }, () -> DriveMode.ROBOT_CENTRIC).withTimeout(totalTime),
        new InstantCommand(() -> {
            timer.stop();
            drivetrain.stop();
        }, drivetrain)
    );
}

        public Command shakeCommand() {
            return shakeCommand(0.3, 0.15, 5.0);
        }

    public void pushDistanceData(){
        double[] distanceAndRotation = calculateDistanceAndRotationToHub();
        // 灏嗚窛绂诲€煎啓鍏?NetworkTable
        autoControlNetworkTable.getEntry("distanceToHub").setDouble(distanceAndRotation[0]);
        autoControlNetworkTable.getEntry("angleDifferenceToHub").setDouble(distanceAndRotation[1]);
        // 鑷姩闃舵榛樿椹鹃┒鍛戒护涓嶈繍琛岋紝杩欓噷鍚屾牱淇濇寔鐩爣鏈濆悜鏈€鏂帮紝
        // 璁╄嚜鍔ㄧ▼搴忛噷鐨勫皠鍑婚棬鎺?isAimed() 鏈夋晥銆?
        drivetrain.setTargetHeading(
            drivetrain.getRotation().plus(Rotation2d.fromDegrees(distanceAndRotation[1])));
    }
    
}
