package frc.robot.controls;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.RobotStatusManager;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.LimelightSupplier;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.RobotStatus;

    public class Autocommand extends SequentialCommandGroup{

        public static void preNameCommands(
            ClimberSubsystem climber,
            IntakerSubsystem intaker,
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            DriveControls driveControls,
            ShooterControls shooterControls
    ) {
        NamedCommands.registerCommand("climb_up", climb_up(climber));
        //NamedCommands.registerCommand("climb_down", climb_down(climber));
        NamedCommands.registerCommand("start_intake", start_intake(intaker));
        NamedCommands.registerCommand("start_intake_timelimit", start_intake_timelimit(intaker));
        NamedCommands.registerCommand("stop_intake", stop_intake(intaker));
        NamedCommands.registerCommand("intaker_down", intaker_down(intaker));
        NamedCommands.registerCommand("intaker_up", intaker_up(intaker));
        NamedCommands.registerCommand("auto_align_to_climb", autoAlignToClimb(drivetrain));
        NamedCommands.registerCommand("shoot", shoot(shooter));
        // NamedCommands.registerCommand("Shake", Autocommand.shake(driveControls));
        // NamedCommands.registerCommand("AutoAim", Autocommand.autoAim(driveControls));
        // NamedCommands.registerCommand("AutoShoot", Autocommand.autoShoot(shooterControls));

    }

        public static Command climb_up(ClimberSubsystem climber) {
            return new RunCommand(() -> {
                climber.climbUp();  // 控制爬升器上升
            }, climber).withTimeout(1.0).andThen(new InstantCommand(() -> {
                climber.holdPosition();  // 停止并锁定当前爬升器位置
            }, climber));
        }
        // public static Command climb_down(ClimberSubsystem climber) {
        //     return Commands.sequence(
        //         new InstantCommand(climber::markAutoClimbUsedInAuto),
        //         new RunCommand(() -> {
        //             climber.climbDownAuto();
        //         }, climber).withTimeout(2.0),
        //         new InstantCommand(() -> {
        //             climber.holdPosition();
        //         }, climber)
        //     );
        // }

        public static Command start_intake(IntakerSubsystem intaker) {
            return new RunCommand(() -> {
                intaker.setIntakerGetterSpeed(Constants.Intaker.IntakeGetterSpeedforAuto);  // 启动吸球
            }, intaker);
        }
        public static Command start_intake_timelimit(IntakerSubsystem intaker) {
            return new RunCommand(() -> {
                intaker.setIntakerGetterSpeed(Constants.Intaker.IntakeGetterSpeedforAuto);  // 吸球机构下降
            }, intaker).withTimeout(3).andThen(new InstantCommand(() -> {
                intaker.setIntakerGetterSpeed(0);  // 停止旋转
            }, intaker));
        }
        public static Command stop_intake(IntakerSubsystem intaker) {
            return new InstantCommand(() -> {
                intaker.setIntakerGetterSpeed(0);  // 停止吸球
            }, intaker);
        }
        public static Command intaker_down(IntakerSubsystem intaker) {
            return new RunCommand(() -> {
                intaker.IntakerDownNonStop();  // 吸球机构下降
            }, intaker).withTimeout(1.0).andThen(new InstantCommand(() -> {
                intaker.stopIntakerotater();  // 停止旋转
            }, intaker));
        }
        public static Command intaker_up(IntakerSubsystem intaker) {
            return new InstantCommand(() -> {
                intaker.IntakerUpNonStop();  // 吸球机构上升
            }, intaker).withTimeout(0.8).andThen(new InstantCommand(() -> {
                intaker.stopIntakerotater();;  // 停止旋转
            }, intaker));
        }
        public static Command autoAlignToClimb(CommandSwerveDrivetrain drivetrain) {
        return Commands.sequence(
            new InstantCommand(() -> {
                LimelightSupplier.setPipeline(Constants.Limelight.climbPipelineIndex);
            }),
            Commands.runEnd(
                () -> {
                    drivetrain.driveToAprilTag();
                },
                () -> {
                    LimelightSupplier.setPipeline(Constants.Limelight.locatePipelineIndex);
                },
                drivetrain
            ).until(() -> drivetrain.isAlignedToAprilTag()));
        }

        public static Command shoot(ShooterSubsystem shooter) {
            return new RunCommand(() -> {
                shooter.setFlywheelSpeedByRPS(64);
                shooter.setBackboardPosition(175);  // 设置自动射击转速
                if(shooter.isBackboardAtTarget()){
                        shooter.backboardMotor.set(0);
                    }else{
                        shooter.outputBackboard();
                    }  
            }, shooter).withTimeout(1.0).andThen(new RunCommand(() -> {
                shooter.setFlywheelSpeedByRPS(64);
                shooter.setBackboardPosition(175);
                if(shooter.isBackboardAtTarget()){
                        shooter.backboardMotor.set(0);
                    }else{
                        shooter.outputBackboard();
                    }  
                shooter.setConveyorSpeedByRPS(25);
            }, shooter).withTimeout(3.0).andThen(new InstantCommand(() -> {
                shooter.setFlywheelSpeedByRPS(0);  // 停止射击
                //shooter.setBackboardPosition(0);  // 恢复背板位置
                shooter.setConveyorSpeedByRPS(0);  // 停止输送
            }, shooter)));
        }
        // public static Command autoAim(DriveControls driveControls) {
        //     return driveControls.autoAimCommand();
        // }
        // public static Command autoShoot(ShooterControls shooterControl) {
        //     return shooterControl.autoShootToHubCommand();
        // }
        // public static Command shake(DriveControls driveControls) {
        //     return driveControls.shakeCommand();
        // }


    }






