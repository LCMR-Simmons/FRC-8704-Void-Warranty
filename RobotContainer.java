// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
//import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Lift;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.subsystems.CoralDropper;
import frc.robot.Constants.CoralDropperConstants;


public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Lift m_lift = new Lift();
  private final CoralDropper m_coralDropper = new CoralDropper();
 
 // private final Camera m_camera = new Camera();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRawAxis(2),OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }
    /*
    // Add CoralDropper default command for trigger control
    m_coralDropper.setDefaultCommand(
        new RunCommand(
            () -> m_coralDropper.setMotorsFromTriggers(
                m_driverController.getLeftTriggerAxis(),
                m_driverController.getRightTriggerAxis()),
            m_coralDropper));
  }
*/
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kL1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
   
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
            .onTrue(new RunCommand(() -> m_lift.setGroundPosition(), m_lift));
        
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(new RunCommand(() -> m_lift.setLowPosition(), m_lift));
    
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new RunCommand(() -> m_lift.setMediumPosition(), m_lift));
    
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(
        () -> m_coralDropper.startRotation(CoralDropperConstants.kRotationsToExecute),
        m_coralDropper));
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> {
                // Run coral dropper motors at full speed
                m_coralDropper.fullSpeed();
            },
            m_coralDropper))
        .onFalse(new InstantCommand(
            () -> m_coralDropper.stopMotors(),
            m_coralDropper));
    }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
  
    // Simple trajectory to drive forward 5 feet (1.524 meters)
    Trajectory forwardTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // No interior waypoints
        List.of(),
        // End 5 feet straight ahead, still facing forward
        new Pose2d(1.524, 0, new Rotation2d(0)),
        config);
  
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        forwardTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
  
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);
  
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(forwardTrajectory.getInitialPose());
  
    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
