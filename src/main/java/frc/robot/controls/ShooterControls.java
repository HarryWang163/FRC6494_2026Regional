package frc.robot.controls;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
                case PassingBall:
                    var xP = shooterSubsystem.getDistanceToPassball();
                    //xP = Math.max(1.2, Math.min(4.5, xP));
                    flywheelSpeed = 0.6956596811733782*xP*xP*xP*xP*xP*xP*xP-14.164408860183267*xP*xP*xP*xP*xP*xP+119.7115375240918*xP*xP*xP*xP*xP-543.0984876677477*xP*xP*xP*xP+1425.1928543240738*xP*xP*xP-2156.199296490315*xP*xP+1736.568139600292*xP-516.097598057351;
                    flywheelSpeed += flywheelSpeedOffset;
                    backboardPosition = xP*xP;//todo:实测传球的时候的背板位置 打表
                    backboardPosition += backboardPositionOffset;
                    conveyorSpeed = Constants.Shooter.conveyorSpeed;
                    if (controller.getRightTriggerAxis() < 0.1) {
                        flywheelSpeed = 0;
                        conveyorSpeed = 0;
                    }
                    if(shooterSubsystem.flywheelMotorLeft.getVelocity().getValueAsDouble()<flywheelSpeed*0.7){
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
                    flywheelSpeed = 0.6956596811733782*x*x*x*x*x*x*x-14.164408860183267*x*x*x*x*x*x+119.7115375240918*x*x*x*x*x-543.0984876677477*x*x*x*x+1425.1928543240738*x*x*x-2156.199296490315*x*x+1736.568139600292*x-516.097598057351;
                    flywheelSpeed += flywheelSpeedOffset;
                    backboardPosition = -122.11889023593778*x*x*x*x*x*x*x+2453.385583841308*x*x*x*x*x*x-20553.05413872581*x*x*x*x*x+92963.33175750206*x*x*x*x-244905.25916147238*x*x*x+375150.1363997868*x*x-308031.51715612336*x+104203.52690842684;
                    backboardPosition += backboardPositionOffset;
                    conveyorSpeed = Constants.Shooter.conveyorSpeed;
                    if (controller.getRightTriggerAxis() < 0.1) {
                        flywheelSpeed = 0;
                        conveyorSpeed = 0;
                    }
                    if(shooterSubsystem.flywheelMotorLeft.getVelocity().getValueAsDouble()<flywheelSpeed*0.95){
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
}