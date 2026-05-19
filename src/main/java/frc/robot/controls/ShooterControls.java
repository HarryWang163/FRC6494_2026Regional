package frc.robot.controls;

import java.util.Set;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotStatusManager;
import frc.robot.Constants.Intaker;
import frc.robot.Constants.RobotStatus;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterControls {
    private final ShooterSubsystem shooterSubsystem;
    private final CommandXboxController controller;

    private double flywheelSpeedOffset = 0.0;
    private double backboardPositionOffset = 0.0;
    private NetworkTable shooterControlTable;
    private double conveyorSpeed = 0.0;
    private double flywheelSpeed = 0;
    private double backboardPosition = 0;
    private final RobotStatusManager robotStatusManager;
    public ShooterControls(ShooterSubsystem shooterSubsystem, CommandXboxController controller,RobotStatusManager rsm) {
        this.shooterSubsystem = shooterSubsystem;
        this.controller = controller;
        this.robotStatusManager = rsm;
        shooterControlTable = NetworkTableInstance.getDefault().getTable("ShooterControl");
    }
    
    public Command defaultShooterCommand() {
        return Commands.run(() -> {
            switch (robotStatusManager.getStatus()) {
                case Stopped:
                    shooterSubsystem.stopMotors();
                    break;
                case AllTelop:
                    shooterSubsystem.setBackboardPosition(0); 
                    if(shooterSubsystem.isBackboardAtTarget()){
                        shooterSubsystem.backboardMotor.set(0);
                    }else{
                        shooterSubsystem.outputBackboard();
                    }  
                    shooterSubsystem.setFlywheelSpeedByRPS(0);
                    if(!Constants.DemoMode.ENABLED && controller.a().getAsBoolean()) { shooterSubsystem.setConveyorSpeedByRPS(Constants.Shooter.conveyorSpeed);}
                    else{shooterSubsystem.setConveyorSpeedByRPS(0);}
                    break;
                case PassingBall:
                    var xP = shooterSubsystem.getDistanceToPassball();
                    xP = Math.max(2.5, Math.min(10.0, xP));
                    flywheelSpeed = +0.004960317456971805*xP*xP*xP*xP*xP*xP*xP-0.22361111097424535*xP*xP*xP*xP*xP*xP+4.234722219890569*xP*xP*xP*xP*xP-43.63194442298942*xP*xP*xP*xP+264.05555544032126*xP*xP*xP-938.6444440829168*xP*xP+1818.2047612908214*xP-1415.9999995643104;
                    flywheelSpeed += flywheelSpeedOffset;
                    backboardPosition = +0.10912698402591796*xP*xP*xP*xP*xP*xP*xP-5.277777773586415*xP*xP*xP*xP*xP*xP+105.55555548303796*xP*xP*xP*xP*xP-1125.6944437656143*xP*xP*xP*xP+6865.763885174154*xP*xP*xP-23769.027765888906*xP*xP+43328.57140795847*xP-31599.99998506075;
                    conveyorSpeed = Constants.Shooter.conveyorSpeed;
                    if (controller.getRightTriggerAxis() < 0.1) {
                        flywheelSpeed = 0;
                        conveyorSpeed = 0;
                    }
                    if(shooterSubsystem.flywheelMotorLeft.getVelocity().getValueAsDouble()<flywheelSpeed*0.9){
                        conveyorSpeed = 0;
                    }
                    //if(controller.a().getAsBoolean()) { conveyorSpeed = Constants.Shooter.conveyorSpeed;}
                    shooterControlTable.getEntry("flywheelTargetSpeed").setDouble(flywheelSpeed);
                    shooterControlTable.getEntry("conveyerTargetSpeed").setDouble(conveyorSpeed);
                    shooterControlTable.getEntry("backboardTargetPosition").setDouble(backboardPosition);
                    shooterSubsystem.setFlywheelSpeedByRPS(flywheelSpeed);
                    shooterSubsystem.setConveyorSpeedByRPS(conveyorSpeed);
                    shooterSubsystem.setBackboardPosition(backboardPosition);  
                    if(shooterSubsystem.isBackboardAtTarget()){
                        shooterSubsystem.backboardMotor.set(0);
                    }else{
                        shooterSubsystem.outputBackboard();
                    }  
                    break;
                case AutoAimming:
                    var x = shooterSubsystem.getDistanceToHub();
                    x = Math.max(1.2, Math.min(4.5, x));
                    flywheelSpeed = -1.4964026859575532*x*x*x*x*x*x*x+29.82445640241506*x*x*x*x*x*x-247.71603153159896*x*x*x*x*x+1108.8727302184625*x*x*x*x-2881.517500617481*x*x*x+4332.05363145275*x*x-3467.2508337871454*x+1192.1508062663759;
                    flywheelSpeed += flywheelSpeedOffset;
                    backboardPosition = +34.5286998307899*x*x*x*x*x*x*x*x-788.5788623683884*x*x*x*x*x*x*x+7658.073303693148*x*x*x*x*x*x-41191.69338972519*x*x*x*x*x+133870.54588688645*x*x*x*x-268559.6987062158*x*x*x+324254.6530324262*x*x-215015.48893131423*x+59977.46545382827;
                    backboardPosition += backboardPositionOffset;
                    conveyorSpeed = Constants.Shooter.conveyorSpeed;
                    if (controller.getRightTriggerAxis() < 0.1) {
                        flywheelSpeed = 0;
                        conveyorSpeed = 0;
                    }
                    if(shooterSubsystem.flywheelMotorLeft.getVelocity().getValueAsDouble()<flywheelSpeed*0.9){
                        conveyorSpeed = 0;
                    }
                    //if(controller.a().getAsBoolean()) { conveyorSpeed = Constants.Shooter.conveyorSpeed;}
                    shooterControlTable.getEntry("flywheelTargetSpeed").setDouble(flywheelSpeed);
                    shooterControlTable.getEntry("conveyerTargetSpeed").setDouble(conveyorSpeed);
                    shooterControlTable.getEntry("backboardTargetPosition").setDouble(backboardPosition);
                    shooterSubsystem.setFlywheelSpeedByRPS(flywheelSpeed);
                    shooterSubsystem.setConveyorSpeedByRPS(conveyorSpeed);
                    shooterSubsystem.setBackboardPosition(backboardPosition);  
                    if(shooterSubsystem.isBackboardAtTarget()){
                        shooterSubsystem.backboardMotor.set(0);
                    }else{
                        shooterSubsystem.outputBackboard();
                    }  
                    break;
                case Climbing:
                case CrossingBump:
                case CrossingTrench:
                    if(shooterSubsystem.getBackboardPosition() <= 20)
                        shooterSubsystem.stopMotors();
                    else{
                        shooterSubsystem.setBackboardPosition(0);    
                        shooterSubsystem.outputBackboard();
                    }
                    break;
                    
            }
        }, shooterSubsystem);
    }

    // public Command resetBackboardCommand(){
    //     return 
    //         Commands.runOnce(()->{
    //             shooterSubsystem.backboardMotor.set(-0.1);
    //             shooterSubsystem.currentBackboardInalyzeMode = Mode.Inalyzing;}, shooterSubsystem)
    //         .andThen(
    //             Commands.waitSeconds(0.5))
    //         .andThen(Commands.runOnce(()->{
    //             shooterSubsystem.backboardMotor.set(0);
    //             shooterSubsystem.backboardEncoder.reset();
    //             shooterSubsystem.backboardPID.reset(0);}, shooterSubsystem))
    //         .andThen(
    //             Commands.waitSeconds(1))
    //         .andThen(Commands.runOnce(()->{
    //             shooterSubsystem.currentBackboardInalyzeMode = Mode.Inalyzed;}, shooterSubsystem));
    // }

    public void outputInfo(){
        double currentConveyorSpeed = shooterSubsystem.conveyorMotor.getVelocity().getValueAsDouble();
        double currentFlywheelSpeed = shooterSubsystem.flywheelMotorLeft.getVelocity().getValueAsDouble();
        double currentBackboardRate = shooterSubsystem.getBackboardPosition();
        System.out.println("Conveyor: " + currentConveyorSpeed + "\t Flywheel: " + currentFlywheelSpeed + "\t Backboard:" + currentBackboardRate);
    }
    public Command resetBackboardCommand(){
        return 
            Commands.runOnce(()->{
                shooterSubsystem.backboardEncoder.reset();
                shooterSubsystem.backboardPID.reset(0);}, shooterSubsystem);
    }

    public void adjustFlywheelSpeedOffset(double offset) {
        flywheelSpeedOffset += offset;
    }

    public void adjustBackboardRateOffset(double offset) {
        backboardPositionOffset += offset;
    }
    public void resetOffsets(){
        flywheelSpeedOffset = 0.0;
        backboardPositionOffset = 0.0;
    }

    public Command autoShootToHubCommand() {
        return Commands.defer(() -> {
            double x = shooterSubsystem.getDistanceToHub();
            x = Math.max(1.2, Math.min(4.5, x));

            double flywheelTargetSpeed = -1.4964026859575532*x*x*x*x*x*x*x+29.82445640241506*x*x*x*x*x*x-247.71603153159896*x*x*x*x*x+1108.8727302184625*x*x*x*x-2881.517500617481*x*x*x+4332.05363145275*x*x-3467.2508337871454*x+1192.1508062663759;
            double backboardTargetPosition = +34.5286998307899*x*x*x*x*x*x*x*x-788.5788623683884*x*x*x*x*x*x*x+7658.073303693148*x*x*x*x*x*x-41191.69338972519*x*x*x*x*x+133870.54588688645*x*x*x*x-268559.6987062158*x*x*x+324254.6530324262*x*x-215015.48893131423*x+59977.46545382827;
            double conveyorTargetSpeed = Constants.Shooter.conveyorSpeed;

            return new RunCommand(() -> {
                shooterSubsystem.setFlywheelSpeedByRPS(flywheelTargetSpeed);
                shooterSubsystem.setConveyorSpeedByRPS(0.0);
                shooterSubsystem.setBackboardPosition(backboardTargetPosition);
                if (shooterSubsystem.isBackboardAtTarget()) {
                    shooterSubsystem.backboardMotor.set(0);
                } else {
                    shooterSubsystem.outputBackboard();
                }
            }, shooterSubsystem)
            .until(() ->
                shooterSubsystem.isBackboardAtTarget()
                && shooterSubsystem.flywheelMotorLeft.getVelocity().getValueAsDouble() >= flywheelTargetSpeed * 0.9
            )
            .withTimeout(0.3)
            .andThen(new RunCommand(() -> {
                shooterSubsystem.setFlywheelSpeedByRPS(flywheelTargetSpeed);
                shooterSubsystem.setBackboardPosition(backboardTargetPosition);
                if (shooterSubsystem.isBackboardAtTarget()) {
                    shooterSubsystem.backboardMotor.set(0);
                } else {
                    shooterSubsystem.outputBackboard();
                }
                shooterSubsystem.setConveyorSpeedByRPS(25);
            }, shooterSubsystem).withTimeout(5.0))
            .andThen(new InstantCommand(() -> {
                shooterSubsystem.setFlywheelSpeedByRPS(0.0);
                shooterSubsystem.setConveyorSpeedByRPS(0.0);
                shooterSubsystem.backboardMotor.set(0.0);
            }, shooterSubsystem));
        }, Set.of(shooterSubsystem));
    }
    
    public Command resetBackboard0Command() {
        return new RunCommand(() -> {
            shooterSubsystem.setBackboardPosition(0);

            if (shooterSubsystem.isBackboardAtTarget()) {
                shooterSubsystem.backboardMotor.set(0);
            } else {
                shooterSubsystem.outputBackboard();
            }

        }, shooterSubsystem)
        .until(() -> shooterSubsystem.isBackboardAtTarget())
        .andThen(new InstantCommand(() -> {
            shooterSubsystem.backboardMotor.set(0);
        }));
    }
}

