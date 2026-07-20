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

    // 默认速度比例，普通驾驶使用。
    private double normalDriveScale = 0.6;
    private double normalTurnScale  = 0.8;

    // 加速模式比例，按住 RB 时使用。
    private double boostDriveScale = 1.0;
    private double boostTurnScale  = 1.0;
    // 自动瞄准时降低平移速度。
    private double slowDriveScale = 0.3;

    private boolean boostEnabled = false;

    private final SlewRateLimiter vxLimiter = new SlewRateLimiter(3.5);   // m/s^2 等效加速度限制
    private final SlewRateLimiter vyLimiter = new SlewRateLimiter(5.0);
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(9.0); // rad/s^2 等效角加速度限制

    // 记录上一周期输出，用于只限制加速、不限制减速。
    private double lastVx = 0.0;
    private double lastVy = 0.0;
    private double lastOmega = 0.0;

    private NetworkTable autoControlNetworkTable;
    
    private final Superstructure superstructure;
    public void setBoostEnabled(boolean enabled) {
        boostEnabled = enabled;
    }

    // 当前是否使用场地坐标系，默认启用。
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

    /** 底盘默认驾驶命令，在 teleop 期间持续执行。 */
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
    // 返回当前位置到两个目标点中更近目标的位移。
    // 返回到更近目标的带符号位移。
    public double calculateDifferenceToTwoTarget(double current, double target1, double target2){
          double x1 = target1 - current;
          double x2 = target2 - current;
          return Math.abs(x1) < Math.abs(x2) ? x1 : x2;
    }
    public double[] calculateDistanceAndRotationToHub() {
        Pose2d currentPose = drivetrain.getState().Pose; // 当前机器人位姿
        double targetX = isRedAlliance ? Constants.Field.RedHubPositionX : Constants.Field.BlueHubPositionX;
        double targetY = isRedAlliance ? Constants.Field.RedHubPositionY : Constants.Field.BlueHubPositionY;

        // 计算目标角度，相对于场地坐标系。
        double deltaX = targetX - currentPose.getX();
        double deltaY = targetY - currentPose.getY();
        double targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX)); // 转换为角度

        // 当前机器人朝向。
        double currentAngle = currentPose.getRotation().getDegrees();
        double angleDifference = targetAngle - currentAngle;
        // Normalize angle difference to the range [-180, 180]
        if (angleDifference > 180) {
            angleDifference -= 360;
        } else if (angleDifference < -180) {
            angleDifference += 360;
        }

        // 计算距离。
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        // 返回距离和旋转角度误差。
        double[] distanceAndRotation = new double[]{distance, angleDifference};

        // 距离单位为米，角度单位为度。
        return distanceAndRotation;
    }
    public double[] calculateDistanceAndRotationToPassBall() {
        Pose2d currentPose = drivetrain.getState().Pose; // 当前机器人位姿
        double targetX = isRedAlliance ? Constants.Field.RedPassingBallPosX : Constants.Field.BluePassingBallPosX;  // 传球目标 X 坐标
        double targetY1 = Constants.Field.PassingBallPosY1;  // 传球目标 Y 坐标
        double targetY2 = Constants.Field.PassingBallPosY2;  // 传球目标 Y 坐标
       
        // 计算目标角度，相对于场地坐标系。
        double deltaX = targetX - currentPose.getX();
        double deltaY = calculateDifferenceToTwoTarget(currentPose.getY(), targetY1, targetY2);
        double targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX)); // 转换为角度

        // 当前机器人朝向。
        double currentAngle = currentPose.getRotation().getDegrees();
        double angleDifference = targetAngle - currentAngle;
        // Normalize angle difference to the range [-180, 180]
        if (angleDifference > 180) {
            angleDifference -= 360;
        } else if (angleDifference < -180) {
            angleDifference += 360;
        }

        // 计算距离。
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        // 返回距离和旋转角度误差。
        double[] distanceAndRotation = new double[]{distance, angleDifference};

        // 距离单位为米，角度单位为度。
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
            double rotationRate = calDifference * Constants.AutoPositioning.autoRotationkP;
            if (Math.abs(rotationRate) < 0.24) {
                return Math.copySign(0.24, rotationRate);
            }
            return rotationRate;
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
            // 限制最大角速度。
            vomega = Math.max(-maxAngularRate, Math.min(maxAngularRate, vomega));
        return vomega;
    }

    /**
     * 只限制加速，也就是输出幅值变大时；不限制减速。
     * 这样松杆或刹车能更及时，同时 boost 仍能抑制电流冲击。
     */
    private double limitAccelOnly(double target, double last, SlewRateLimiter limiter) {
        // 如果正在减速，直接放行到目标值。
        if (Math.abs(target) < Math.abs(last)) {
            limiter.reset(target);  // 同步 limiter 内部状态，防止下一次突跳。
            return target;
        }
        // 否则是加速，使用 limiter。
        return limiter.calculate(target);
    }

    /** 紧急停止：清零限速器，让速度输出立刻归零。 */
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
        Optional<Alliance> ally = DriverStation.getAlliance(); // 获取联盟颜色
        if (ally.isPresent()) {
            // 根据联盟颜色更新自动瞄准目标。
            if (ally.get() == Alliance.Red) {
                isRedAlliance = true; // 红队
            } else if (ally.get() == Alliance.Blue) {
                isRedAlliance = false; // 蓝队
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

    // DriveControls.java
    public Command autoAimForShootCommand() {
        return drivetrain.applyRequest(() -> {
            double[] hubAim = calculateDistanceAndRotationToHub();
            double distanceToHub = hubAim[0];
            double angleDifferenceToHub = hubAim[1];

            autoControlNetworkTable.getEntry("distanceToHub").setDouble(distanceToHub);
            autoControlNetworkTable.getEntry("angleDifferenceToHub").setDouble(angleDifferenceToHub);

            drivetrain.setTargetHeading(
                drivetrain.getRotation().plus(Rotation2d.fromDegrees(angleDifferenceToHub))
            );

            double vomega = calculateRotationSpeedFromRotationAngle(angleDifferenceToHub);

            autoControlNetworkTable.getEntry("autoRotationRate").setDouble(vomega);
            autoControlNetworkTable.getEntry("ActualVX").setDouble(0.0);
            autoControlNetworkTable.getEntry("ActualVY").setDouble(0.0);
            autoControlNetworkTable.getEntry("ActualVOmega").setDouble(vomega);

            return fieldCentric
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(vomega);
        }, () -> DriveMode.FIELD_CENTRIC)
        .withTimeout(Constants.Superstructure.shootTimeoutSeconds)
        .andThen(new InstantCommand(() -> drivetrain.stop(), drivetrain));
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
                    vx = amplitude;   // 前
                    vy = 0.0;
                    break;
                case 1:
                    vx = 0.0;
                    vy = amplitude;   // 左
                    break;
                case 2:
                    vx = -amplitude;  // 后
                    vy = 0.0;
                    break;
                case 3:
                    vx = 0.0;
                    vy = -amplitude;  // 右
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
        // 将距离数据写入 NetworkTable。
        autoControlNetworkTable.getEntry("distanceToHub").setDouble(distanceAndRotation[0]);
        autoControlNetworkTable.getEntry("angleDifferenceToHub").setDouble(distanceAndRotation[1]);
        // 自动阶段默认驾驶命令不运行，这里同样保持目标朝向最新，
        // 让自动程序里的射击门控 isAimed() 有效。
        drivetrain.setTargetHeading(
            drivetrain.getRotation().plus(Rotation2d.fromDegrees(distanceAndRotation[1])));
    }
    
}
