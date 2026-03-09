package frc.robot.controls;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;


public class IntakerControls {
    private final IntakerSubsystem IntakerSubsystem;
    private final CommandXboxController controller;
    private final ShooterSubsystem ShooterSubsystem;

    //private static final double ROTATER_STEP_PER_CYCLE = 0.00;

    public IntakerControls(IntakerSubsystem Intaker, ShooterSubsystem Shooter, CommandXboxController controller) {
        this.ShooterSubsystem = Shooter;
        this.IntakerSubsystem = Intaker;
        this.controller = controller;
    }

    

        //B.A：按住下降 + getter转；松开后 rotater保持当前位置，getter继续转
        // controller.a()
        //     .whileTrue(
        //         Commands.run(() -> {
        //             // rotater下降：范围会被 subsystem 限制
        //             double cur = intakerSubsystem.getIntakerotaterPosition();
        //             intakerSubsystem.setIntakerRotaterPosition(cur - ROTATER_STEP_PER_CYCLE);

        //             // getter 开
        //             intakerSubsystem.setIntakeGetterOn(true);
        //         }, intakerSubsystem)
        //     )
        //     .onFalse(
        //         Commands.runOnce(() -> {
        //             // 松：把当前位置作为target保持
        //             double cur = intakerSubsystem.getIntakerotaterPosition();
        //             intakerSubsystem.setIntakerRotaterPosition(cur);

        //             // gtter继续转
        //             intakerSubsystem.setIntakeGetterOn(true);
        //         }, intakerSubsystem)
        //     );
        // // === B.B：按住上升+getter停；松开后rotater保持当前位置
        // controller.b()
        //     .whileTrue(
        //         Commands.run(() -> {
        //             // rotater升
        //             double cur = intakerSubsystem.getIntakerotaterPosition();
        //             intakerSubsystem.setIntakerRotaterPosition(cur + ROTATER_STEP_PER_CYCLE);

        //             // getter关
        //             intakerSubsystem.setIntakeGetterOn(false);
        //         }, intakerSubsystem)
        //     )
        //     .onFalse(
        //         Commands.runOnce(() -> {
        //             // 松：保持当前位置
        //             double cur = intakerSubsystem.getIntakerotaterPosition();
        //             intakerSubsystem.setIntakerRotaterPosition(cur);

        //             // getter仍然停止
        //             intakerSubsystem.setIntakeGetterOn(false);
        //         }, intakerSubsystem)
        //     );
    
    public Command defaultIntakerCommand() {
        return Commands.run(() -> {
        double getterspeed = 0;
        double conveyoyspeedforintake = 0;
        double statorCurrent = ShooterSubsystem.getConveyorStatorCurrent();
        
        if (controller.getLeftTriggerAxis() > 0.1) {
            getterspeed = controller.getLeftTriggerAxis() * 0.3 + 0.3;
            ShooterSubsystem.setConveyorSpeedByRPS(conveyoyspeedforintake);
            }

        IntakerSubsystem.setIntakerGetterSpeed(getterspeed);

        if(controller.getLeftY() < -0.5){
                    IntakerSubsystem.IntakerUpNonStop();
            } else if(controller.getLeftY() > 0.5){
                    IntakerSubsystem.IntakerDownNonStop();
                    // double rotaterspeed = -controller.getLeftY() * 0.3 + 0.2;
                    // IntakerSubsystem.setIntakerRotater(rotaterspeed);

            } else {
                    IntakerSubsystem.stopIntakerotater();
            }
        }, IntakerSubsystem);
    }
    public Command resetIntakerRotaterEncoderCommand(IntakerSubsystem intakerSubsystem) {
        return Commands.runOnce(intakerSubsystem::zeroEncoder, intakerSubsystem);
    }
}