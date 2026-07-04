package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Shooter 总成的硬件封装。
 *
 * 飞轮和 shooter 内部 conveyor 都使用电压开环输出。
 * 背板控制沿用旧代码中的电机、编码器和 PID 调参流程。
 */
public class ShooterSubsystem extends SubsystemBase {
    public final TalonFX leftFlywheel;
    public final TalonFX rightFlywheel;
    public final TalonFX leftConveyor;
    public final TalonFX rightConveyor;

    // 保留旧字段别名，让原有 dashboard tuner 在重构后仍能指向左飞轮和左 conveyor。
    public final TalonFX flywheelMotorLeft;
    public final TalonFX conveyorMotor;

    public final TalonFXS backboardMotor;
    public final Encoder backboardEncoder;
    public final ProfiledPIDController backboardPID;

    private final VoltageOut flywheelVoltageRequest = new VoltageOut(0);
    private final VoltageOut conveyorVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private final Follower flywheelFollower;
    private final Follower conveyorFollower;

    private final NetworkTable shooterNetworkTable;

    private double targetFlywheelRps = 0.0;
    private double targetLeftFlywheelRps = 0.0;
    private double targetRightFlywheelRps = 0.0;
    private double shooterConveyorPercent = 0.0;

    public ShooterSubsystem() {
        leftFlywheel = new TalonFX(Constants.Shooter.leftFlywheelID);
        rightFlywheel = new TalonFX(Constants.Shooter.rightFlywheelID);
        leftConveyor = new TalonFX(Constants.Shooter.leftConveyorID);
        rightConveyor = new TalonFX(Constants.Shooter.rightConveyorID);

        flywheelMotorLeft = leftFlywheel;
        conveyorMotor = leftConveyor;

        leftFlywheel.getConfigurator().apply(Constants.Shooter.flyWheelSlot0Configs);
        rightFlywheel.getConfigurator().apply(Constants.Shooter.flyWheelSlot0Configs);
        leftConveyor.getConfigurator().apply(Constants.Shooter.conveyorSlot0Configs);
        rightConveyor.getConfigurator().apply(Constants.Shooter.conveyorSlot0Configs);

        var motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        leftFlywheel.getConfigurator().apply(motorOutputConfigs);
        leftConveyor.getConfigurator().apply(motorOutputConfigs);

        flywheelFollower = new Follower(leftFlywheel.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(50);
        conveyorFollower = new Follower(leftConveyor.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(50);
        setRightFollowLeft();

        backboardMotor = new TalonFXS(Constants.Shooter.backboardMotorID);
        backboardMotor.getConfigurator().apply(Constants.Shooter.backboardSlot0Configs);

        backboardEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        backboardEncoder.setSamplesToAverage(5);
        backboardEncoder.setMinRate(1.0);
        backboardEncoder.setDistancePerPulse(1);

        backboardPID = new ProfiledPIDController(
            Constants.Shooter.backboardPositionPID.kP,
            Constants.Shooter.backboardPositionPID.kI,
            Constants.Shooter.backboardPositionPID.kD,
            new Constraints(10000, 100000)
        );
        backboardPID.setTolerance(10.0);

        shooterNetworkTable = NetworkTableInstance.getDefault().getTable("Shooter");
    }

    public void setRightFollowLeft() {
        // 成对机构机械方向相反，因此右侧电机以反向 follower 跟随左侧电机。
        rightFlywheel.setControl(flywheelFollower);
        rightConveyor.setControl(conveyorFollower);
    }

    public void applyLeftConfigurationToRight() {
        Slot0Configs configs = new Slot0Configs();
        leftFlywheel.getConfigurator().refresh(configs);
        rightFlywheel.getConfigurator().apply(configs);
    }

    public void setFlywheelVelocity(double leftRps, double rightRps) {
        // 对外接口使用 RPS，便于 Superstructure 阅读；实际下发仍是电压开环。
        targetLeftFlywheelRps = leftRps;
        targetRightFlywheelRps = rightRps;
        targetFlywheelRps = (Math.abs(leftRps) + Math.abs(rightRps)) / 2.0;

        double requestedVoltage = targetFlywheelRps * Constants.Shooter.flywheelVoltsPerRps;
        requestedVoltage = MathUtil.clamp(requestedVoltage, -12.0, 12.0);
        leftFlywheel.setControl(flywheelVoltageRequest.withOutput(requestedVoltage));
        setRightFollowLeft();
    }

    public void setFlywheelVelocityByDistance(double distance) {
        // 临时距离映射。拿到真实射击数据后，应替换为实测表或重新调常量。
        double clampedDistance = MathUtil.clamp(distance, 1.2, 5.0);
        double targetRps = Constants.Shooter.defaultFlywheelTargetRps + (clampedDistance - 2.5) * 4.0;
        setFlywheelVelocity(targetRps, targetRps);
    }

    public void setFlywheelSpeedByRPS(double targetSpeed) {
        setFlywheelVelocity(targetSpeed, targetSpeed);
    }

    public void setFlywheelVoltage(double volts) {
        targetLeftFlywheelRps = 0.0;
        targetRightFlywheelRps = 0.0;
        targetFlywheelRps = 0.0;
        leftFlywheel.setControl(flywheelVoltageRequest.withOutput(MathUtil.clamp(volts, -12.0, 12.0)));
        setRightFollowLeft();
    }

    public void runShooterConveyor(double percent) {
        shooterConveyorPercent = MathUtil.clamp(percent, -1.0, 1.0);
        leftConveyor.setControl(conveyorVoltageRequest.withOutput(shooterConveyorPercent * 12.0));
        setRightFollowLeft();
    }

    public void stopShooterConveyor() {
        runShooterConveyor(0.0);
    }

    public void setConveyorSpeedByRPS(double speed) {
        leftConveyor.setControl(velocityRequest.withVelocity(speed));
        setRightFollowLeft();
    }

    public void setConveyorSpeedOpen(double speed) {
        runShooterConveyor(speed);
    }

    public void stopFlywheel() {
        setFlywheelVoltage(Constants.Shooter.flywheelIdleVoltage);
    }

    public void stopAll() {
        stopFlywheel();
        stopShooterConveyor();
        backboardMotor.set(0);
    }

    public void stopMotors() {
        stopAll();
    }

    public boolean atSpeed() {
        // 虽然飞轮控制是电压开环，但仍读取 TalonFX 速度作为喂球门控。
        if (targetFlywheelRps <= 1.0) {
            return false;
        }
        return Math.abs(getLeftFlywheelVelocity() - targetLeftFlywheelRps) <= Constants.Shooter.flywheelSpeedToleranceRps
            && Math.abs(Math.abs(getRightFlywheelVelocity()) - Math.abs(targetRightFlywheelRps)) <= Constants.Shooter.flywheelSpeedToleranceRps;
    }

    public double getLeftFlywheelVelocity() {
        return leftFlywheel.getVelocity().getValueAsDouble();
    }

    public double getRightFlywheelVelocity() {
        return rightFlywheel.getVelocity().getValueAsDouble();
    }

    public double getAverageFlywheelVelocity() {
        return (Math.abs(getLeftFlywheelVelocity()) + Math.abs(getRightFlywheelVelocity())) / 2.0;
    }

    public double getFlywheelVelocityDifference() {
        return Math.abs(getLeftFlywheelVelocity()) - Math.abs(getRightFlywheelVelocity());
    }

    public double getTargetFlywheelRps() {
        return targetFlywheelRps;
    }

    public void setBackboardSpeedByRPS(double speed) {
        if (speed > 0 && speed < 10) {
            speed = 10;
        } else if (speed < 0 && speed > -10) {
            speed = -10;
        }
        backboardMotor.setControl(velocityRequest.withVelocity(speed));
    }

    public void setBackboardPosition(double targetPosition) {
        // 保留旧背板机构的软件限位。
        if (targetPosition > Constants.Shooter.backboardUpLimit) {
            targetPosition = Constants.Shooter.backboardUpLimit;
        } else if (targetPosition < Constants.Shooter.backboardDownLimit) {
            targetPosition = Constants.Shooter.backboardDownLimit;
        }
        backboardPID.setGoal(targetPosition);
    }

    public void outputBackboard() {
        // 背板仍使用旧外部编码器和 ProfiledPID；Superstructure 只选择目标位置。
        double pidOutput = backboardPID.calculate(getBackboardPosition());
        pidOutput = MathUtil.clamp(pidOutput, -Constants.Shooter.backboardSpeedMax, Constants.Shooter.backboardSpeedMax);

        if (Math.abs(pidOutput) < 0.8) {
            pidOutput = 0.0;
        }

        shooterNetworkTable.getEntry("backboardPIDOutput").setDouble(pidOutput);
        setBackboardSpeedByRPS(pidOutput);
    }

    public double getBackboardPosition() {
        return backboardEncoder.getDistance();
    }

    public boolean isBackboardAtTarget() {
        return backboardPID.atGoal();
    }

    public void conveyorWaitForAcceleration() {
        stopShooterConveyor();
    }

    public void conveyorRun() {
        runShooterConveyor(Constants.Superstructure.shooterFeedPercent);
    }

    public void resetbackboardencoder() {
        backboardEncoder.reset();
    }

    public double getConveyorStatorCurrent() {
        return leftConveyor.getStatorCurrent().getValueAsDouble();
    }

    public double getDistanceToHub() {
        return NetworkTableInstance.getDefault().getTable("AutoControl").getEntry("distanceToHub").getDouble(0.0);
    }

    public double getDistanceToPassball() {
        return NetworkTableInstance.getDefault().getTable("AutoControl").getEntry("distanceToPassball").getDouble(0.0);
    }

    @Override
    public void periodic() {
        backboardPID.calculate(getBackboardPosition());
        shooterNetworkTable.getEntry("flywheelSpeed").setDouble(getAverageFlywheelVelocity());
        shooterNetworkTable.getEntry("leftFlywheelSpeed").setDouble(getLeftFlywheelVelocity());
        shooterNetworkTable.getEntry("rightFlywheelSpeed").setDouble(getRightFlywheelVelocity());
        shooterNetworkTable.getEntry("flywheelTargetSpeed").setDouble(targetFlywheelRps);
        shooterNetworkTable.getEntry("flywheelAtSpeed").setBoolean(atSpeed());
        shooterNetworkTable.getEntry("shooterConveyorPercent").setDouble(shooterConveyorPercent);
        shooterNetworkTable.getEntry("conveyorSpeed").setDouble(leftConveyor.getVelocity().getValueAsDouble());
        shooterNetworkTable.getEntry("backboardCurrentRate").setDouble(getBackboardPosition());
        shooterNetworkTable.getEntry("backboardTargetRate").setDouble(backboardPID.getSetpoint().position);
        shooterNetworkTable.getEntry("isBackboardAtTarget").setBoolean(isBackboardAtTarget());
        shooterNetworkTable.getEntry("distanceGetted").setDouble(getDistanceToHub());
        shooterNetworkTable.getEntry("distancetopassball").setDouble(getDistanceToPassball());
        shooterNetworkTable.getEntry("statorCurrent").setDouble(getConveyorStatorCurrent());
    }
}
