package frc.robot.controls;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeRotaterSubsystem;
import frc.robot.subsystems.Superstructure;

/**
 * 注册 PathPlanner 的 NamedCommands。
 *
 * 自动路径只请求机器人级动作，不直接设置 shooter、intake 或 conveyor 电机；
 * 这样自动和手动都会走同一套 Superstructure 状态机。
 */
public class Autocommand extends SequentialCommandGroup {
    public static void preNameCommands(
        Superstructure superstructure,
        IntakeRotaterSubsystem intakeRotater,
        DriveControls driveControls,
        OperatorControls operatorControls
    ) {
        NamedCommands.registerCommand("start_intake", startIntake(superstructure));
        NamedCommands.registerCommand("start_intake_and_conveyor", startIntake(superstructure));
        NamedCommands.registerCommand("stop_intake", idle(superstructure));
        NamedCommands.registerCommand("stop_intake_", idle(superstructure));
        NamedCommands.registerCommand("stop_intake_and_conveyor", idle(superstructure));
        NamedCommands.registerCommand("init", init(superstructure));
        NamedCommands.registerCommand("intaker_down", init(superstructure));
        NamedCommands.registerCommand("intaker_up", raiseIntaker(intakeRotater));
        NamedCommands.registerCommand("shoot_hub", shootHub(superstructure));
        NamedCommands.registerCommand("reset_backboard0", operatorControls.resetBackboard0Command());
        NamedCommands.registerCommand("AutoAimForShoot", autoAimForShoot(driveControls));
        NamedCommands.registerCommand("AutoAim", autoAimForShoot(driveControls));
        NamedCommands.registerCommand("Shake", driveControls.shakeCommand());
    }

    public static Command startIntake(Superstructure superstructure) {
        return superstructure.requestIntakeCommand();
    }

    public static Command idle(Superstructure superstructure) {
        return superstructure.requestIdleCommand();
    }

    public static Command init(Superstructure superstructure) {
        return superstructure.requestInitCommand();
    }

    public static Command raiseIntaker(IntakeRotaterSubsystem intakeRotater) {
        return intakeRotater.raiseForMatchCommand();
    }

    public static Command shootHub(Superstructure superstructure) {
        return Commands.sequence(
            superstructure.requestShootHubCommand(),
            Commands.waitSeconds(Constants.Superstructure.shootTimeoutSeconds),
            superstructure.requestIdleCommand()
        );
    }
    public static Command autoAimForShoot(DriveControls driveControls) {
        return driveControls.autoAimForShootCommand();
    }

}
