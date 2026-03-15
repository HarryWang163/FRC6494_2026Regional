package frc.robot.controls;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
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
                if(controller.getRightY() < -0.5){
                    climber.climbUpNonStop();
                } else if(controller.getRightY() > 0.5){
                    climber.climbDownNonStop();
                } else if (controller.y().getAsBoolean()) {
                    climber.climbUp();
                } else if (controller.x().getAsBoolean()) {
                    climber.climbDown();
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

    public Command resetClimberEncoderCommand(ClimberSubsystem climber) {
        return Commands.runOnce(climber::zeroEncoder, climber);
    }
    
}
