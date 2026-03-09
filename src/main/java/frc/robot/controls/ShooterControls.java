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
                    break;
                case PassingBall:
                    var xP = shooterSubsystem.getDistanceToPassball();
                    xP = Math.max(1.2, Math.min(4.5, xP));
                    flywheelSpeed = -1.4964026859575532*xP*xP*xP*xP*xP*xP*xP+29.82445640241506*xP*xP*xP*xP*xP*xP-247.71603153159896*xP*xP*xP*xP*xP+1108.8727302184625*xP*xP*xP*xP-2881.517500617481*xP*xP*xP+4332.05363145275*xP*xP-3467.2508337871454*xP+1192.1508062663759;
                    flywheelSpeed += flywheelSpeedOffset;
                    backboardPosition = +34.5286998307899*xP*xP*xP*xP*xP*xP*xP*xP-788.5788623683884*xP*xP*xP*xP*xP*xP*xP+7658.073303693148*xP*xP*xP*xP*xP*xP-41191.69338972519*xP*xP*xP*xP*xP+133870.54588688645*xP*xP*xP*xP-268559.6987062158*xP*xP*xP+324254.6530324262*xP*xP-215015.48893131423*xP+59977.46545382827;
                    backboardPosition += backboardPositionOffset;
                    conveyorSpeed = Constants.Shooter.conveyorSpeed;
                    if (controller.getRightTriggerAxis() < 0.1) {
                        flywheelSpeed = 0;
                        conveyorSpeed = 0;
                    }
                    if(shooterSubsystem.flywheelMotorLeft.getVelocity().getValueAsDouble()<flywheelSpeed*0.9){
                        conveyorSpeed = 0;
                    }
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
            .withTimeout(1.0)
            .andThen(new RunCommand(() -> {
                shooterSubsystem.setFlywheelSpeedByRPS(flywheelTargetSpeed);
                shooterSubsystem.setBackboardPosition(backboardTargetPosition);
                if (shooterSubsystem.isBackboardAtTarget()) {
                    shooterSubsystem.backboardMotor.set(0);
                } else {
                    shooterSubsystem.outputBackboard();
                }
                if (shooterSubsystem.flywheelMotorLeft.getVelocity().getValueAsDouble() >= flywheelTargetSpeed * 0.9) {
                    shooterSubsystem.setConveyorSpeedByRPS(conveyorTargetSpeed);
                } else {
                    shooterSubsystem.setConveyorSpeedByRPS(0.0);
                }
            }, shooterSubsystem).withTimeout(3.0))
            .andThen(new InstantCommand(() -> {
                shooterSubsystem.setFlywheelSpeedByRPS(0.0);
                shooterSubsystem.setConveyorSpeedByRPS(0.0);
                shooterSubsystem.backboardMotor.set(0.0);
            }, shooterSubsystem));
        }, Set.of(shooterSubsystem));
    }
}