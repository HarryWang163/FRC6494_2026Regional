package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeRotaterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;

/**
 * 上操作员手柄的“意图层”。
 *
 * 这里刻意不直接设置任何机构电机。按钮、自动辅助命令和调参快捷键都会
 * 转换成 Superstructure 请求，由 Superstructure 统一处理多机构动作顺序。
 */
public class OperatorControls {
    private final Superstructure superstructure;
    private final ShooterSubsystem shooterSubsystem;

    public OperatorControls(Superstructure superstructure, ShooterSubsystem shooterSubsystem) {
        this.superstructure = superstructure;
        this.shooterSubsystem = shooterSubsystem;
    }

    public Command requestIdleCommand() {
        return superstructure.requestIdleCommand();
    }

    public Command requestIntakeCommand() {
        return superstructure.requestIntakeCommand();
    }

    public Command requestShootHubCommand() {
        return superstructure.requestShootHubCommand();
    }

    public Command requestAimHubCommand() {
        return superstructure.requestAimHubCommand();
    }

    public Command requestPassBallCommand() {
        return superstructure.requestPassBallCommand();
    }

    public Command requestEjectCommand() {
        return superstructure.requestEjectCommand();
    }

    public Command requestManualCommand() {
        return superstructure.requestManualCommand();
    }

    public void adjustFlywheelSpeedOffset(double offset) {
        superstructure.adjustFlywheelSpeedOffset(offset);
    }

    public void adjustBackboardRateOffset(double offset) {
        superstructure.adjustBackboardPositionOffset(offset);
    }

    public void resetOffsets() {
        superstructure.resetOffsets();
    }

    public Command resetBackboardCommand() {
        return Commands.runOnce(superstructure::resetBackboardEncoder, shooterSubsystem);
    }

    public Command manualRaiseIntakeRotaterCommand(IntakeRotaterSubsystem intakeRotater) {
        return Commands.runEnd(intakeRotater::manualRaise, intakeRotater::stop, intakeRotater);
    }

    public Command manualLowerIntakeRotaterCommand(IntakeRotaterSubsystem intakeRotater) {
        return Commands.runEnd(intakeRotater::manualLower, intakeRotater::stop, intakeRotater);
    }

    // PathPlanner 自动射击辅助：先预备到所有射击条件满足，再请求喂球，
    // 最后回到空闲状态。真正的“是否能射”判断仍然放在 Superstructure。
    public Command autoShootToHubCommand() {
        return Commands.sequence(
            superstructure.requestShootHubCommand(),
            Commands.waitSeconds(Constants.Superstructure.shootTimeoutSeconds),
            superstructure.requestIdleCommand()
        );
    }

    // 保留旧背板回零动作，方便调参和现场恢复；
    // 具体电机操作在 ShooterSubsystem 内实现，这里只转发命令。
    public Command resetBackboard0Command() {
        return shooterSubsystem.homeBackboardCommand();
    }
}
