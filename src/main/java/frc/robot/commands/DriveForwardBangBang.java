// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardBangBang extends CommandBase {
  /** Creates a new DriveForwardBangBang. */
  private Drivetrain drivetrainSubsystem;
  private double distance;
  public DriveForwardBangBang(Drivetrain drivetrainSubsystem, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.distance = distance;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drivetrainSubsystem.getAverageDistance() < distance){
      drivetrainSubsystem.setMotrs(0.3, 0.3);
    }
    else if(drivetrainSubsystem.getAverageDistance() == distance){
      drivetrainSubsystem.setMotrs(0, 0);
    }
    else{
      drivetrainSubsystem.setMotrs(-0.3, -0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
