// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  CANSparkMax leftMaster = new CANSparkMax(DriveConstants.mFrontLeft, MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(DriveConstants.mFrontRight, MotorType.kBrushless);
  CANSparkMax leftSlave = new CANSparkMax(DriveConstants.mRearLeft, MotorType.kBrushless);
  CANSparkMax rightSlave = new CANSparkMax(DriveConstants.mRearRight, MotorType.kBrushless);

  ADIS16448_IMU imu = new ADIS16448_IMU();

  CANEncoder leftEncoder = leftMaster.getEncoder();
  CANEncoder rightEncoder = rightMaster.getEncoder();

  private Pose2d mPosition = new Pose2d(0, 0, Rotation2d.fromDegrees(getAngle()));

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(26));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()),
   mPosition);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.kS, 
    DriveConstants.kV, DriveConstants.kA);

  PIDController leftPIDController = new PIDController(DriveConstants.kP, 0, 0);
  PIDController rightPIDController = new PIDController(DriveConstants.kP, 0, 0);

  public static final Field2d field = new Field2d();

  public Drivetrain() {

    leftMaster.restoreFactoryDefaults();
    leftSlave.restoreFactoryDefaults();
    rightMaster.restoreFactoryDefaults();
    rightSlave.restoreFactoryDefaults();

    leftMaster.setSmartCurrentLimit(40);
    leftSlave.setSmartCurrentLimit(40);
    rightMaster.setSmartCurrentLimit(40);
    rightSlave.setSmartCurrentLimit(40);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftMaster.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    
    SmartDashboard.putData(field);

    imu.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mPosition = odometry.update(Rotation2d.fromDegrees(getAngle()),getLeftMeters(), getRightMeters());

    SmartDashboard.putNumber("Odometry X:", mPosition.getX());
    SmartDashboard.putNumber("Odometry Y:", mPosition.getY());
    SmartDashboard.putNumber("Odometry Angle:", mPosition.getRotation().getDegrees());
    SmartDashboard.putNumber("Left Velocity", getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("Right Velocity", getWheelSpeeds().rightMetersPerSecond);

    field.getObject("Robot").setPose(odometry.getPoseMeters());
  }

  public double getLeftEncoderValue(){
    return leftEncoder.getPosition();
  } 

  public double getRightEncoderValue(){
    return rightEncoder.getPosition();
  }
  /*
  public double getHeading() {
    return Math.IEEEremainder(imu.getAngle(), 360) * (DriveConstants.GYRO_REVERSE ? -1.0 : 1.0);
  }
  */

  public double getAngle(){ 
    return -imu.getAngle();
  } 

  public void plotTrajectory(Trajectory trajectory) {
    ArrayList<Pose2d> poses = new ArrayList<>();

    for (Trajectory.State pose : trajectory.getStates()) {
      poses.add(pose.poseMeters);
    }

    field.getObject("foo").setPoses(poses);
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getEncoderVelocityMetersPerSecond(leftEncoder),
        getEncoderVelocityMetersPerSecond(rightEncoder));
  }

  
  public double getEncoderVelocityMetersPerSecond(CANEncoder selectedEncoder) {
    return selectedEncoder.getVelocity() / DriveConstants.kDriveGearing * Math.PI
        * DriveConstants.mWheelDiameter / 60;
  }
  
  public double getAverageEncoderValue(){
    return (getRightEncoderValue() + getLeftEncoderValue()) / 2;
  }

  public double getLeftMeters(){
    double wheelRotations = getLeftEncoderValue() / 12;
    double distance = wheelRotations * (Math.PI * DriveConstants.mWheelDiameter);

    return distance;
  }

  public double getRightMeters(){
    double wheelRotations = getRightEncoderValue() / 12;
    double distance = wheelRotations * (Math.PI * DriveConstants.mWheelDiameter);

    return distance;
  }

  public double getAverageDistance(){
    return (getRightMeters() + getLeftMeters()) / 2;
  }
  public void setArcadeDrive(double speed, double turn){
    
    double left = speed - turn;
    double right = speed + turn;

    leftMaster.set(left);
    rightMaster.set(right);
  }

  public void setMotrs(double rightMotor, double leftMotor){
    leftMaster.set(leftMotor);
    rightMaster.set(rightMotor);
  }

  
 public Pose2d getPose() {
  return odometry.getPoseMeters();
}

public Pose2d getPosition() {
  return mPosition;
}

public Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose) {
  if (initPose) {
    new InstantCommand(() -> {resetOdometry(trajectory.getInitialPose());});
  }
  RamseteCommand rCommand = new RamseteCommand(
    trajectory,
    this::getPosition,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(
        DriveConstants.kS,
        DriveConstants.kV,
        DriveConstants.kA),
    AutoConstants.kDriveKinematics,
    this::getWheelSpeeds,
    leftPIDController,
    rightPIDController,
    // RamseteCommand passes volts to the callback
    this::tankDriveVolts,
    this);
  return rCommand;
}

public void resetOdometry(Pose2d pose) {
  //resetEncoders();
  imu.reset();
  odometry.resetPosition(pose, Rotation2d.fromDegrees(getAngle()));
}

public void tankDriveVolts(double leftVolts, double rightVolts) {
  // izquierdoE.setVoltage(leftVolts);
  // derechoE.setVoltage(rightVolts);
  leftMaster.set(leftVolts / 12);
  rightMaster.set(rightVolts / 12);
}
}
