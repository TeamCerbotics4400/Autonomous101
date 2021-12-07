// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardPID extends CommandBase {
  /** Creates a new DriveForwardPID. */
  private Drivetrain drivetrainSubsystem;
  private PIDController pidController;
  double distance; 
  public DriveForwardPID(Drivetrain drivetrainSubsystem, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    pidController = new PIDController(Constants.kP, Constants.kI, Constants.kD);

    pidController.setSetpoint(distance);
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(drivetrainSubsystem.getAverageDistance());
    drivetrainSubsystem.setMotrs(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.setMotrs(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
