package frc.robot.controls;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberControls {

    private final ClimberSubsystem climber;
    private final CommandXboxController controller;

    private final NetworkTable table;

    public ClimberControls(ClimberSubsystem climber, CommandXboxController controller) {
        this.climber = climber;
        this.controller = controller;
        this.table = NetworkTableInstance.getDefault().getTable("ClimberControls");
    }

    public Command defaultClimberCommand() {
        return Commands.run(()->{
            //climber.climbUp();
                // 优先级：如果同时按，优先上升（你也可以反过来）
                if(controller.y().getAsBoolean()&&controller.x().getAsBoolean()){
                    climber.zeroEncoder();
                }
                else if (controller.y().getAsBoolean()) {
                    climber.climbUp();
                } else if (controller.x().getAsBoolean()) {
                    climber.climbDownNonStop();
                } else {
                    // 松开：记录当前位置 + 锁死
                    climber.captureHoldPosition();
                    climber.holdPosition();
                }
                

                // 调试用
                table.getEntry("xHeld_up").setBoolean(controller.x().getAsBoolean());
                table.getEntry("yHeld_down").setBoolean(controller.y().getAsBoolean());
        }, climber);

    }

    public Command enableInitCommand() {
        return Commands.run(climber::initialize, climber)
                .withTimeout(0.5)
                .andThen(Commands.runOnce(climber::zeroEncoder, climber));
    }
}
