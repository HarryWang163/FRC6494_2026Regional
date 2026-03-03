package frc.robot.controls;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.RobotStatusManager;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.RobotStatus;

    public class Autocommand extends SequentialCommandGroup{
        public static Command climb_up(ClimberSubsystem climber) {
            return new RunCommand(() -> {
                climber.climbUp();  // 控制爬升器上升
            }, climber).withTimeout(1.0)  // 
            .andThen(new InstantCommand(() -> {
                climber.holdPosition();  // 停止并锁定当前爬升器位置
            }, climber));
        }
        public static Command climb_down(ClimberSubsystem climber) {
            return new RunCommand(() -> {
                climber.climbDown();  // 控制爬升器下降
            }, climber).withTimeout(1.0)  //
            .andThen(new InstantCommand(() -> {
                climber.holdPosition();  // 停止并锁定当前爬升器位置
            }, climber));
        }
    }






