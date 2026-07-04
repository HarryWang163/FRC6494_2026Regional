// 版权所有 (c) FIRST 和其他 WPILib 贡献者。
// 本项目为开源软件；你可以依据项目根目录中的 WPILib BSD license 文件
// 修改和/或分享本代码。

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterSubsystem;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run(); 
        m_robotContainer.tunerPeriodic();
        m_robotContainer.dashboardPeriodic();

    }

    @Override
    public void disabledInit() {
        System.out.println("Robot disabledInit called");
        // 进入 disabled 时强制所有重构后的机构回到安全状态，
        // 即使上一周期还有手动或自动请求也不能继续动作。
        m_robotContainer.superstructure.requestIdle();
        m_robotContainer.shooterSubsystem.stopAll();
        m_robotContainer.intakeRollerSubsystem.stop();
        m_robotContainer.intakeRotaterSubsystem.stop();
        m_robotContainer.conveyorSubsystem.stop();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        m_robotContainer.driveControls.pushDistanceData();
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
