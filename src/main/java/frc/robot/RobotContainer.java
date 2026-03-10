package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotStatus;
import frc.robot.controls.DriveControls;
import frc.robot.controls.IntakerControls;
import frc.robot.controls.ShooterControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.tuning.ConfigTalonFXMotorTuner;
import frc.robot.tuning.ConfigTalonFXSMotorTuner;
import frc.robot.tuning.DriveGainsTuner;
import frc.robot.utils.LedBindings;
import frc.robot.controls.ClimberControls;
import frc.robot.controls.Autocommand;
import frc.robot.subsystems.ClimberSubsystem;
public class RobotContainer {

  /* ====================== */
  /*         手柄            */
  /* ====================== */

  // 使用2个 Xbox 手柄（端口 0,1）
  private final CommandXboxController controllerlower = new CommandXboxController(0);
  private final CommandXboxController controllerupper = new CommandXboxController(1);

  /* ====================== */
  /*         子系统           */
  /* ====================== */

  // Phoenix TunerX 生成的底盘
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final IntakerSubsystem IntakerSubsystem = new IntakerSubsystem();

  // LED 子系统
  private final LEDSubsystem leds = new LEDSubsystem(0, 72 ,0); // 总长度=72+72         

  private final RobotStatusManager robotStatusManager = new RobotStatusManager();
  /* ====================== */
  /*     控制封装/日志        */
  /* ====================== */

  private final DriveControls driveControls = new DriveControls(drivetrain, controllerlower, robotStatusManager );
  private final ShooterControls shooterControls = new ShooterControls(shooterSubsystem, controllerupper,robotStatusManager);
  private final ClimberControls climberControls = new ClimberControls(climberSubsystem, controllerupper);
  private final IntakerControls intakerControls = new IntakerControls(IntakerSubsystem, shooterSubsystem, controllerupper);
  private final DriveGainsTuner driveGainsTuner = new DriveGainsTuner(drivetrain);
  private ConfigTalonFXMotorTuner configFlywheelTuner = new ConfigTalonFXMotorTuner(shooterSubsystem.flywheelMotorLeft, "flywheel", Constants.Shooter.flyWheelSlot0Configs);
  private ConfigTalonFXMotorTuner configConveyorTuner = new ConfigTalonFXMotorTuner(shooterSubsystem.conveyorMotor, "conveyor", Constants.Shooter.conveyorSlot0Configs);
  private ConfigTalonFXSMotorTuner configBackboardTuner = new ConfigTalonFXSMotorTuner(shooterSubsystem.backboardMotor, "backboard", Constants.Shooter.backboardSlot0Configs);

  private final SendableChooser<Command> autoChooser;
  
  private final Trigger enableTrigger = new Trigger(DriverStation::isEnabled);

  
  public RobotContainer() {
    Autocommand.preNameCommands(
        climberSubsystem,
        IntakerSubsystem,
        drivetrain,
        shooterSubsystem,
        driveControls,
        shooterControls
    );

    SignalLogger.enableAutoLogging(false);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configueSwerve();
    configueShooter();
    configueClimber();
    bingdingLED();
    configueIntaker();
  }

  /* ====================== */
  /*        底盘配置          */
  /* ====================== */

  private void configueSwerve() {
    enableTrigger.onTrue(driveControls.getAllianceColorCommand());
    Pose2d startingPose = Constants.StartingPoints.Red.Point2;
    drivetrain.resetPose(startingPose);

    // 默认驾驶命令 
    /*
     * LeftX 底盘左右
     * LeftY 底盘前后  Bump/Trench偏移未完成
     * RightX Aiming偏移    AllTelop模式下 底盘旋转
     * 
    */
    drivetrain.setDefaultCommand(driveControls.defaultDriveCommand());

    // Disabled 时进入 idle（防止模块乱动）
    RobotModeTriggers.disabled().whileTrue(driveControls.idleCommand());

    
    //左保险：按住刹车
    controllerlower.leftBumper()
      .onTrue(Commands.runOnce(driveControls::emergencyStop))
      .whileTrue(driveControls.brakeWhileHeld());
    
    // 右保险：按住加速（Boost）
    controllerlower.rightBumper()
        .onTrue(Commands.runOnce(() -> driveControls.setBoostEnabled(true)))
        .onFalse(Commands.runOnce(() -> driveControls.setBoostEnabled(false)));

    // Back：重置场地坐标系角度
    controllerlower.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    
    // Start：强行使用metaTag2全场定位
    controllerlower.start().whileTrue(drivetrain.run(drivetrain::forceUsingLimelightmt2));

    //LM 按下时autoaimming，松开时alltelop
    // controllerlower.leftStick().onTrue(robotStatusManager.setStatusCommand(RobotStatus.AutoAimming));
    // controllerlower.leftStick().onFalse(robotStatusManager.setStatusCommand(RobotStatus.AllTelop));

    //双操均可触发autoaimming模式，松开时alltelop
    Trigger autoAim = controllerupper.rightBumper();//.or(controllerlower.rightStick());
    autoAim.onTrue(robotStatusManager.setStatusCommand(RobotStatus.AutoAimming));
    autoAim.onFalse(robotStatusManager.setStatusCommand(RobotStatus.AllTelop));

    //双操均可触发passingball模式，松开时alltelop
    Trigger autoAim2 = controllerupper.leftBumper();//.or(controllerlower.leftStick());
    autoAim2.onTrue(robotStatusManager.setStatusCommand(RobotStatus.PassingBall));
    autoAim2.onFalse(robotStatusManager.setStatusCommand(RobotStatus.AllTelop));

    controllerlower.leftTrigger()
    .onTrue(robotStatusManager.setStatusCommand(RobotStatus.CrossingTrench))
    .onFalse(robotStatusManager.setStatusCommand(RobotStatus.AllTelop));
    
    controllerlower.rightTrigger()
    .onTrue(robotStatusManager.setStatusCommand(RobotStatus.CrossingBump))
    .onFalse(robotStatusManager.setStatusCommand(RobotStatus.AllTelop));

    controllerlower.a()
    .onTrue(robotStatusManager.setStatusCommand(RobotStatus.Climbing))
    .onFalse(robotStatusManager.setStatusCommand(RobotStatus.AllTelop));
  }

  private void configueShooter(){
    shooterSubsystem.setDefaultCommand(shooterControls.defaultShooterCommand());
    controllerupper.povLeft().onTrue(Commands.runOnce(() -> shooterControls.adjustFlywheelSpeedOffset(-1)));
    controllerupper.povRight().onTrue(Commands.runOnce(() -> shooterControls.adjustFlywheelSpeedOffset(1)));
    controllerupper.povUp().onTrue(Commands.runOnce(() -> shooterControls.adjustBackboardRateOffset(100)));
    controllerupper.povDown().onTrue(Commands.runOnce(() -> shooterControls.adjustBackboardRateOffset(-100)));
    controllerupper.back().onTrue(Commands.runOnce(() -> shooterControls.resetOffsets()));
    controllerupper.start().onTrue(Commands.runOnce(() -> shooterControls.resetBackboardCommand()));
    controllerupper.b().whileTrue(new RunCommand(() -> shooterSubsystem.setConveyorSpeedByRPS(10), shooterSubsystem));
    controllerupper.a().whileTrue(new RunCommand(() -> shooterSubsystem.setConveyorSpeedByRPS(-25), shooterSubsystem));
  }
  /* ====================== */
  /*        LED 绑定          */
  /* ====================== */

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

  private void configueClimber() {
    //enableTrigger.onTrue(climberControls.enableInitCommand());
    climberSubsystem.setDefaultCommand(climberControls.defaultClimberCommand());
    //controllerupper.b().onTrue(climberControls.resetClimberEncoderCommand(climberSubsystem));
  }

  private void configueIntaker() {
    IntakerSubsystem.setDefaultCommand(intakerControls.defaultIntakerCommand());
    controllerupper.a().onTrue(intakerControls.resetIntakerRotaterEncoderCommand(IntakerSubsystem));
  }

}
