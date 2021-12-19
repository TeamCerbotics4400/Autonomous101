// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private Drivetrain drivetrainSubsystem;
  private Supplier<Double> speed, turn;
  public ArcadeDrive(Drivetrain drivetrainSubsystem, Supplier<Double> speed,
  Supplier<Double> turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.speed = speed;
    this.turn = turn;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Arcade Drive Empezado");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.setArcadeDrive(speed.get(), turn.get());
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
