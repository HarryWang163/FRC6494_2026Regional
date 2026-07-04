// 版权所有 (c) FIRST 和其他 WPILib 贡献者。
// 本项目为开源软件；你可以依据项目根目录中的 WPILib BSD license 文件
// 修改和/或分享本代码。

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
