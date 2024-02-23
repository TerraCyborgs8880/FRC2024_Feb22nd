// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeOutakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    //led maybe
    public static AddressableLED m_led = new AddressableLED(0);
    public static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(150);

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    private final IntakeOutakeSubsystem m_robotIntakeOutake = new IntakeOutakeSubsystem();
    private final ElevatorSubsystem m_robotElevator = new ElevatorSubsystem();
        
    // The driver's controller
    public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    public static XboxController m_commandController = new XboxController(OIConstants.kCommandControllerPort);

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
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true, true),
                m_robotDrive));

        // Configure subsystems

        
        //LED on
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();
        for (var i = 0; i < m_ledBuffer.getLength(); i++) { 
            // Sets the specified LED to the RGB values for red
            if(i % 4 == 1) m_ledBuffer.setRGB(i, 255, 255, 255);
            if(i % 4 == 2) m_ledBuffer.setRGB(i, 255, 0, 0);
            if(i % 4 == 3) m_ledBuffer.setRGB(i, 0, 255, 0);
            
        }
        m_led.setData(m_ledBuffer);
    }

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
        /*
         * DRIVER CONTROLLER COMMANDS
         * SET X = RT
         * SPIN = LT
         * ELEVATOR UP = Y
         * ELEVATOR DOWN = A
         */
        //set wheels to X formation [Driver Controller RT]
        new JoystickButton(m_driverController, Axis.kRightTrigger.value)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.setX(),
                m_robotDrive));
        //spin [Driver Controller LT]
        JoystickButton btnSpin = new JoystickButton(m_driverController, Axis.kLeftTrigger.value);
        btnSpin.whileTrue(new RunCommand(
            () -> m_robotDrive.drive(0, 0, 1, true, true),
            m_robotDrive));
        //elevator up [Driver Controller Y]
        JoystickButton btnElevatorUp = new JoystickButton(m_driverController, Button.kY.value);
        btnElevatorUp.whileTrue(new RunCommand(
            () -> m_robotElevator.setElevator(Constants.DriveConstants.elevatorVolts),
            m_robotElevator));
        btnElevatorUp.whileFalse(new RunCommand(
            () -> m_robotElevator.setElevator(0),
            m_robotElevator));
        //elevator down [Driver Controller A]
        JoystickButton btnElevatorDown = new JoystickButton(m_driverController, Button.kA.value);
        btnElevatorDown.whileTrue(new RunCommand(
            () -> m_robotElevator.setElevator(-Constants.DriveConstants.elevatorVolts),
            m_robotElevator));
        btnElevatorDown.whileFalse(new RunCommand(
            () -> m_robotElevator.setElevator(0),
            m_robotElevator));
        
        
        /*
         * COMMAND CONTROLLER
         * INTAKE IN = RB
         * INTAKE OUT (SHOOT) = LB
         * WRIST UP = Y
         * WRIST DOWN = A
         * ARM UP = RT
         * ARM DOWN = RT
         */
        //intake in [Command Controller RB]
        JoystickButton btnIntakeIn = new JoystickButton(m_commandController, Button.kRightBumper.value);
        btnIntakeIn.whileTrue(new RunCommand(
            () -> m_robotIntakeOutake.intakeOn(Constants.DriveConstants.intakeOutakeVolts * 5),
            m_robotIntakeOutake));
        btnIntakeIn.whileFalse(new RunCommand(
            () -> m_robotIntakeOutake.intakeOn(0),
            m_robotIntakeOutake));
        // [Command Controller LB]
        JoystickButton btnIntakeOut = new JoystickButton(m_commandController, Button.kLeftBumper.value);
        btnIntakeOut.whileTrue(new RunCommand(
            () -> m_robotIntakeOutake.intakeOn(-Constants.DriveConstants.intakeOutakeVolts * 1.5),
            m_robotIntakeOutake));
        btnIntakeOut.whileFalse(new RunCommand(
            () -> m_robotIntakeOutake.intakeOn(0),
            m_robotIntakeOutake));
        //wrist down [Command Controller Y]
        JoystickButton btnWristUp = new JoystickButton(m_commandController, Button.kY.value);
        btnWristUp.whileTrue(new RunCommand(
            () -> m_robotIntakeOutake.setWrist(Constants.DriveConstants.wristVolts),
            m_robotIntakeOutake));
        btnWristUp.whileFalse(new RunCommand(
            () -> m_robotIntakeOutake.setWrist(0),
            m_robotIntakeOutake));
        //wrist up [Command Controller A]
        JoystickButton btnWristDown = new JoystickButton(m_commandController, Button.kA.value);
        btnWristDown.whileTrue(new RunCommand(
            () -> m_robotIntakeOutake.setWrist(-Constants.DriveConstants.wristVolts),
            m_robotIntakeOutake));
        btnWristDown.whileFalse(new RunCommand(
            () -> m_robotIntakeOutake.setWrist(0),
            m_robotIntakeOutake));
        //arm up [Command Controller RT]
        JoystickButton btnArmUp = new JoystickButton(m_commandController, Button.kX.value);
        btnArmUp.whileTrue(new RunCommand(
            () -> m_armSubsystem.setArm(-Constants.DriveConstants.armVolts),
            m_armSubsystem));
        btnArmUp.whileFalse(new RunCommand(
            () -> m_armSubsystem.setArm(0),
            m_armSubsystem));
        //arm down [Command Controller LT]
        JoystickButton btnArmDown = new JoystickButton(m_commandController, Button.kB.value);
        btnArmDown.whileTrue(new RunCommand(
            () -> m_armSubsystem.setArm(Constants.DriveConstants.armVolts),
            m_armSubsystem));
        btnArmDown.whileFalse(new RunCommand(
            () -> m_armSubsystem.setArm(0),
            m_armSubsystem));
        
        

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

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }
}
