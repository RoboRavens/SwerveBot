// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.util.Deadband;

public class DrivetrainDefaultCommand extends Command {
  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand() {
    addRequirements(Robot.DRIVETRAIN_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double controllerDirection = Robot.allianceColor == Alliance.Red ? 1 : -1;
    double x = Robot.DRIVE_CONTROLLER.getLeftY() * controllerDirection;
    double y = Robot.DRIVE_CONTROLLER.getLeftX() * controllerDirection;
    double r = Robot.DRIVE_CONTROLLER.getRightX() * -1;

    x = Deadband.adjustValueToZero(x, Constants.JOYSTICK_DEADBAND);
    y = Deadband.adjustValueToZero(y, Constants.JOYSTICK_DEADBAND);
    r = Deadband.adjustValueToZero(r, Constants.JOYSTICK_DEADBAND);

    // The angle of the robot as measured by a gyroscope. The robot's angle is
    // considered to be zero when it is facing directly away from your alliance
    // station wall.
    Rotation2d robotRotation = Robot.DRIVETRAIN_SUBSYSTEM.getOdometryRotation();

    double cutPowerRotation = Robot.cutPower ? 0.5 : 1;
    double cutPowerTranslation = Robot.cutPower ? 0.25 : 1;
    x = x * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * cutPowerTranslation;
    y = y * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * cutPowerTranslation;
    r = r * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND * cutPowerRotation;

    var targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        x, // x translation
        y, // y translation
        r, // rotation
        robotRotation // The angle of the robot as measured by a gyroscope.
    );

    Robot.DRIVETRAIN_SUBSYSTEM.drive(targetChassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
