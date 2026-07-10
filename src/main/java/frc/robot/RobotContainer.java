package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controls.Autocommand;
import frc.robot.controls.DriveControls;
import frc.robot.controls.OperatorControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeRotaterSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.tuning.ConfigTalonFXMotorTuner;
import frc.robot.tuning.ConfigTalonFXSMotorTuner;
import frc.robot.tuning.DriveGainsTuner;
import frc.robot.utils.GameData2026;
import frc.robot.utils.LedBindings;

public class RobotContainer {

  /* ====================== */
  /*         手柄            */
  /* ====================== */

  // 使用一个 Xbox 手柄：0 号口同时负责底盘驾驶和机构操作。
  private final CommandXboxController controller = new CommandXboxController(0);

  /* ====================== */
  /*         子系统           */
  /* ====================== */

  // 这里继续使用 Phoenix TunerX 生成的底盘，避免改动已经验证过的 swerve 底层。
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();
  public final IntakeRotaterSubsystem intakeRotaterSubsystem = new IntakeRotaterSubsystem();
  public final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  public final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  // Superstructure 是所有得分机构的唯一调度器。
  // 控制层和自动命令只能向它请求状态，不能直接命令电机。
  public final Superstructure superstructure = new Superstructure(
      shooterSubsystem,
      intakeRollerSubsystem,
      intakeRotaterSubsystem,
      conveyorSubsystem,
      drivetrain,
      limelightSubsystem
  );

  // LED 子系统仍然只负责灯带显示，不参与机构状态判断。
  private final LEDSubsystem leds = new LEDSubsystem(0, 72, 0); // 总长度 = 72 + 72

  /* ====================== */
  /*     控制封装/调参        */
  /* ====================== */

  public final DriveControls driveControls = new DriveControls(drivetrain, controller, superstructure);

  // 操作员意图层替代旧的射击/进球分离控制类，让复杂动作判断集中留在 Superstructure。
  private final OperatorControls operatorControls = new OperatorControls(superstructure, shooterSubsystem);
  private final DriveGainsTuner driveGainsTuner = new DriveGainsTuner(drivetrain);
  private final GameData2026 gameData2026 = new GameData2026();
  private ConfigTalonFXMotorTuner configFlywheelTuner =
      new ConfigTalonFXMotorTuner(shooterSubsystem.leftFlywheel, "flywheel", Constants.Shooter.flyWheelSlot0Configs, false);
  private ConfigTalonFXSMotorTuner configBackboardTuner =
      new ConfigTalonFXSMotorTuner(shooterSubsystem.backboardMotor, "backboard", Constants.Shooter.backboardSlot0Configs);

  private final SendableChooser<Command> autoChooser;
  private final Trigger enableTrigger = new Trigger(DriverStation::isEnabled);

  public RobotContainer() {
    Pose2d startingPose = Constants.StartingPoints.Red.Point2;
    drivetrain.resetPose(startingPose);
    configueSwerve();
    configueShooter();
    bingdingLED();
    configueIntaker();
    Autocommand.preNameCommands(
        superstructure,
        intakeRotaterSubsystem,
        driveControls,
        operatorControls
    );
    SignalLogger.enableAutoLogging(false);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /* ====================== */
  /*        底盘配置          */
  /* ====================== */

  private void configueSwerve() {
    enableTrigger.onTrue(driveControls.getAllianceColorCommand());
    Pose2d startingPose = Constants.StartingPoints.Red.Point2;
    drivetrain.resetPose(startingPose);

    // 默认驾驶命令说明：
    // 左摇杆 X 控制底盘左右平移。
    // 左摇杆 Y 控制底盘前后平移，越障/壕沟偏移逻辑目前仍沿用旧状态。
    // 右摇杆 X 在自动瞄准模式下作为偏移量，在全手动模式下控制底盘旋转。
    drivetrain.setDefaultCommand(driveControls.defaultDriveCommand());

    // 禁用状态下进入空闲命令，防止模块继续响应上一周期的运动请求。
    RobotModeTriggers.disabled().whileTrue(driveControls.idleCommand());

    // Back 键：重置场地坐标系角度。
    controller.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // 预留的左摇杆自动瞄准切换，当前保持注释，避免改变驾驶员已习惯的按键。
    // controller.leftStick().onTrue(operatorControls.requestAimHubCommand());
    // controller.leftStick().onFalse(operatorControls.requestIdleCommand());

  }

  private void configueShooter() {
    // 射击相关按钮只请求 Superstructure 状态；是否喂球由 Superstructure
    // 根据飞轮速度和底盘瞄准统一判断。
    // 这些命令 require Superstructure，与自动程序共用需求；
    // 用 teleop 门控避免自动赛阶段误触直接取消整个 PathPlanner 自动。
    Trigger teleopOnly = RobotModeTriggers.teleop();
    controller.rightTrigger().and(teleopOnly)
        .onTrue(operatorControls.requestShootHubCommand())
        .onFalse(operatorControls.requestIdleCommand());
    controller.rightBumper().and(teleopOnly)
        .onTrue(operatorControls.requestEjectCommand())
        .onFalse(operatorControls.requestIdleCommand());
    controller.povLeft().onTrue(Commands.runOnce(() -> operatorControls.adjustFlywheelVelocityOffset(-1.0)));
    controller.povRight().onTrue(Commands.runOnce(() -> operatorControls.adjustFlywheelVelocityOffset(1.0)));
    controller.povUp().onTrue(Commands.runOnce(() -> operatorControls.adjustBackboardRateOffset(100)));
    controller.povDown().onTrue(Commands.runOnce(() -> operatorControls.adjustBackboardRateOffset(-100)));
    controller.start().onTrue(operatorControls.resetBackboardCommand());
  }

  private void bingdingLED() {
    LedBindings.bindModeIndicators(leds);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void tunerPeriodic() {
    boolean allowMotorOutputTuning = DriverStation.isTest();

    driveGainsTuner.periodic();
    configFlywheelTuner.periodic(() -> shooterSubsystem.applyLeftConfigurationToRight());
    configBackboardTuner.periodic();
    shooterSubsystem.debugFlywheelVelocityPeriodic(allowMotorOutputTuning);
    shooterSubsystem.debugBackboardTargetPeriodic(allowMotorOutputTuning);
    shooterSubsystem.setRightFlywheelFollowLeft();
    shooterSubsystem.setRightConveyorFollowLeft();
  }

  public void dashboardPeriodic() {
    gameData2026.periodic();
  }

  private void configueIntaker() {
    // 进球相关按钮也只请求 Superstructure 状态；intaker 开赛后默认保持下放。
    Trigger teleopOnly = RobotModeTriggers.teleop();
    controller.leftTrigger().and(teleopOnly)
        .onTrue(operatorControls.requestPassBallCommand())
        .onFalse(operatorControls.requestIdleCommand());

    controller.leftBumper().and(teleopOnly)
        .onTrue(operatorControls.requestIntakeCommand())
        .onFalse(operatorControls.requestIdleCommand());

    Trigger manualRaiseIntaker = controller.y().and(teleopOnly);
    manualRaiseIntaker
        .onTrue(operatorControls.requestManualCommand())
        .whileTrue(operatorControls.manualRaiseIntakeRotaterCommand(intakeRotaterSubsystem))
        .onFalse(operatorControls.requestIdleCommand());

    Trigger manualLowerIntaker = controller.a().and(teleopOnly);
    manualLowerIntaker
        .onTrue(operatorControls.requestManualCommand())
        .whileTrue(operatorControls.manualLowerIntakeRotaterCommand(intakeRotaterSubsystem))
        .onFalse(operatorControls.requestIdleCommand());
  }
}
