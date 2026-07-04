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
import frc.robot.Constants.RobotStatus;
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

  // 使用两个 Xbox 手柄：0 号给底盘驾驶，1 号给机构操作员。
  private final CommandXboxController controllerlower = new CommandXboxController(0);
  private final CommandXboxController controllerupper = new CommandXboxController(1);

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

  private final RobotStatusManager robotStatusManager = new RobotStatusManager();

  /* ====================== */
  /*     控制封装/调参        */
  /* ====================== */

  public final DriveControls driveControls = new DriveControls(drivetrain, controllerlower, robotStatusManager);

  // 操作员意图层替代旧的射击/进球分离控制类，让复杂动作判断集中留在 Superstructure。
  private final OperatorControls operatorControls = new OperatorControls(superstructure, shooterSubsystem);
  private final DriveGainsTuner driveGainsTuner = new DriveGainsTuner(drivetrain);
  private final GameData2026 gameData2026 = new GameData2026();
  private ConfigTalonFXMotorTuner configFlywheelTuner =
      new ConfigTalonFXMotorTuner(shooterSubsystem.leftFlywheel, "flywheel", Constants.Shooter.flyWheelSlot0Configs);
  private ConfigTalonFXMotorTuner configConveyorTuner =
      new ConfigTalonFXMotorTuner(shooterSubsystem.leftConveyor, "conveyor", Constants.Shooter.conveyorSlot0Configs);
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

    // 左保险：按住刹车。
    controllerlower.leftBumper()
        .onTrue(Commands.runOnce(driveControls::emergencyStop))
        .whileTrue(driveControls.brakeWhileHeld());

    // 右保险：按住开启加速模式，松开后恢复普通速度。
    controllerlower.rightBumper()
        .onTrue(Commands.runOnce(() -> driveControls.setBoostEnabled(true)))
        .onFalse(Commands.runOnce(() -> driveControls.setBoostEnabled(false)));

    // Back 键：重置场地坐标系角度。
    controllerlower.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // Start 键：强行使用 shooter 侧 Limelight 的 MegaTag2 全场定位。
    controllerlower.start().whileTrue(drivetrain.run(drivetrain::forceUsingLimelightmt2));

    controllerlower.a().whileTrue(drivetrain.run(() -> {
      drivetrain.forceUsingLimelightmt2WithllName(Constants.Limelight.LIMELIGHT_NAME_Shooter);
      System.out.println("Try using shooter for metatag2");
    }));
    controllerlower.b().whileTrue(drivetrain.run(() -> {
      drivetrain.forceUsingLimelightmt2WithllName(Constants.Limelight.LIMELIGHT_NAME_Intaker);
      System.out.println("Try using intaker for metatag2");
    }));

    // 预留的左摇杆自动瞄准切换，当前保持注释，避免改变驾驶员已习惯的按键。
    // controllerlower.leftStick().onTrue(robotStatusManager.setStatusCommand(RobotStatus.AutoAimming));
    // controllerlower.leftStick().onFalse(robotStatusManager.setStatusCommand(RobotStatus.AllTelop));

    // 上操作员右保险：按住进入自动瞄准模式，松开回到全手动模式。
    Trigger autoAim = controllerupper.rightBumper(); // 可按需要再并入驾驶员左摇杆。
    autoAim.onTrue(robotStatusManager.setStatusCommand(RobotStatus.AutoAimming));
    autoAim.onFalse(robotStatusManager.setStatusCommand(RobotStatus.AllTelop));

    // 上操作员左保险：按住进入传球模式，松开回到全手动模式。
    Trigger autoAim2 = controllerupper.leftBumper(); // 可按需要再并入驾驶员左摇杆。
    autoAim2.onTrue(robotStatusManager.setStatusCommand(RobotStatus.PassingBall));
    autoAim2.onFalse(robotStatusManager.setStatusCommand(RobotStatus.AllTelop));

    controllerlower.leftTrigger()
        .onTrue(robotStatusManager.setStatusCommand(RobotStatus.CrossingTrench))
        .onFalse(robotStatusManager.setStatusCommand(RobotStatus.AllTelop));

    controllerlower.rightTrigger()
        .onTrue(robotStatusManager.setStatusCommand(RobotStatus.CrossingBump))
        .onFalse(robotStatusManager.setStatusCommand(RobotStatus.AllTelop));
  }

  private void configueShooter() {
    // 射击相关按钮只请求 Superstructure 状态；是否喂球由 Superstructure
    // 根据飞轮速度和底盘瞄准统一判断。
    // 这些命令 require Superstructure，与自动程序共用需求；
    // 用 teleop 门控避免自动赛阶段误触直接取消整个 PathPlanner 自动。
    Trigger teleopOnly = RobotModeTriggers.teleop();
    controllerupper.rightTrigger().and(teleopOnly)
        .onTrue(operatorControls.requestShootCommand())
        .onFalse(operatorControls.requestIdleCommand());
    controllerupper.y().and(teleopOnly)
        .onTrue(operatorControls.requestPrepShootCommand())
        .onFalse(operatorControls.requestIdleCommand());
    controllerupper.b().and(teleopOnly)
        .onTrue(operatorControls.requestEjectCommand())
        .onFalse(operatorControls.requestIdleCommand());
    controllerupper.povLeft().onTrue(Commands.runOnce(() -> operatorControls.adjustFlywheelSpeedOffset(-1)));
    controllerupper.povRight().onTrue(Commands.runOnce(() -> operatorControls.adjustFlywheelSpeedOffset(1)));
    controllerupper.povUp().onTrue(Commands.runOnce(() -> operatorControls.adjustBackboardRateOffset(100)));
    controllerupper.povDown().onTrue(Commands.runOnce(() -> operatorControls.adjustBackboardRateOffset(-100)));
    controllerupper.back().onTrue(Commands.runOnce(() -> operatorControls.resetOffsets()));
    controllerupper.start().onTrue(operatorControls.resetBackboardCommand());
  }

  private void bingdingLED() {
    LedBindings.bindModeIndicators(leds);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void tunerPeriodic() {
    driveGainsTuner.periodic();
    configFlywheelTuner.periodic(() -> shooterSubsystem.applyLeftConfigurationToRight());
    configConveyorTuner.periodic(null);
    configBackboardTuner.periodic();
    shooterSubsystem.setRightFollowLeft();
  }

  public void dashboardPeriodic() {
    gameData2026.periodic();
  }

  private void configueIntaker() {
    // 进球相关按钮也只请求 Superstructure 状态；intaker 开赛后默认保持下放。
    Trigger teleopOnly = RobotModeTriggers.teleop();
    controllerupper.leftTrigger().and(teleopOnly)
        .onTrue(operatorControls.requestIntakeCommand())
        .onFalse(operatorControls.requestIdleCommand());

    Trigger manualRaiseIntaker = controllerupper.x().and(teleopOnly);
    manualRaiseIntaker
        .onTrue(operatorControls.requestManualCommand())
        .whileTrue(operatorControls.manualRaiseIntakeRotaterCommand(intakeRotaterSubsystem))
        .onFalse(operatorControls.requestIdleCommand());

    Trigger manualLowerIntaker = controllerupper.a().and(teleopOnly);
    manualLowerIntaker
        .onTrue(operatorControls.requestManualCommand())
        .whileTrue(operatorControls.manualLowerIntakeRotaterCommand(intakeRotaterSubsystem))
        .onFalse(operatorControls.requestIdleCommand());
  }
}
