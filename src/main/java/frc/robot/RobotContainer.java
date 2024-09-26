// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.JoystickUtil;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.LockMode;
import frc.robot.Constants.PivotConstants.PivotDirection;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.auton.BackUp;
import frc.robot.auton.ShootBackUp;
import frc.robot.auton.TwoShootBasicAuto;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;
import frc.robot.commands.Lock;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.climber.Climber;

/**
 * This class is where the bulk of the robot is declared. Very little robot logic should actually 
 * be handled in the {@link Robot} periodic methods (other than the scheduler calls).  
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) 
 * should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake m_intake = new Intake();
  private final Pivot m_pivot = new Pivot();
  private final Shooter m_shooter = new Shooter();
  private final Feeder m_feeder = new Feeder();
  private final AHRS m_gyro = new AHRS();
  private final Camera m_camera = new Camera("camera");
  public final DriveSubsystem m_robotDrive = new DriveSubsystem(m_gyro, m_camera);
  private final Climber m_climber = new Climber();
  private final ShootBackUp m_shootBackUp = new ShootBackUp(m_robotDrive, m_intake, m_shooter, m_pivot, m_feeder);
  private final TwoShootBasicAuto m_twoShootBasicAuto = new TwoShootBasicAuto(m_robotDrive, m_intake, m_shooter, m_pivot, m_feeder);
  private final BackUp m_backUp = new BackUp(m_robotDrive);
  private final SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(IOConstants.kDriverControllerPort);
  XboxController m_driverControllerSP = new XboxController(IOConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(IOConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    configureDriverBindings();
    configureOperatorBindings();
    launchCommands();
  }

    // set up for driving controls
    DoubleSupplier moveForward = () -> MathUtil.applyDeadband(
        -m_driverController.getLeftY(), Constants.IOConstants.kControllerDeadband);
    DoubleSupplier moveSideways = () -> MathUtil.applyDeadband(
        -m_driverController.getLeftX(), Constants.IOConstants.kControllerDeadband);

    Lock lockMode = new Lock(m_robotDrive, m_pivot, m_camera, moveForward, moveSideways);

  /**
   * Configure the button bindings for the Diver's Controller ({@code m_driverController})
   */
  private void configureDriverBindings() {
    /*
     * ---Driving Controls for the driver
     * The left stick on Xbox controller controls the translation of the robot - 1
     * The right stick controls the rotation of the robot - 12
     */
    m_robotDrive.setDefaultCommand(
        Commands.run(
            () -> m_robotDrive.drive(
                moveForward.getAsDouble(),
                moveSideways.getAsDouble(),
                JoystickUtil.squareAxis(
                    -m_driverController.getRightX()),
                true),
            m_robotDrive));

            

    /*
     * ---Reset button and X mode button
     * right stick button on controller controls the re-zeroing of the heading
     */

    m_driverController
        .rightStick()
        .onTrue(Commands.runOnce(m_robotDrive::zeroHeading, m_robotDrive));

    /*
     * INTAKE and FEEDER controls
     * 
     * LEFT BUMPER: Intake note and feed into shooter
     * RIGHT BUMPER: Spit note and outtake
     */
    m_driverController.leftBumper().whileTrue(m_feeder.feedNote().alongWith(m_intake.intake()));

    m_driverController.rightBumper().whileTrue(m_feeder.spitNote().alongWith(m_intake.outtake()));

    /*
     * SHOOTER PIVOT controls 
     * 
     * LEFT TRIGGER: Pivot up
     * RIGHT TRIGGER: Pivot down
     */
      m_driverController
        .leftTrigger()
        .whileTrue(Commands.runEnd(() -> {
          m_pivot.pivot(PivotDirection.UP);
        }, () -> {
          m_pivot.pivot(PivotDirection.STOP);
        }, m_pivot));

    m_driverController
        .rightTrigger()
        .whileTrue(Commands.runEnd(() -> {
          m_pivot.pivot(PivotDirection.DOWN);
        }, () -> {
          m_pivot.pivot(PivotDirection.STOP);
        }, m_pivot));



    /*
     * PIVOT auto commands
     * 
     * LEFT D-PAD: Pivot to AMP
     * RIGHT D-PAD: Pivot to Speaker
     */
    m_driverController.povLeft().whileTrue(m_pivot.pivotToAMP()).onTrue(Commands.runOnce(()-> m_shooter.setStateSpeaker(ShooterState.IDLE)));
    m_driverController.povRight().whileTrue(m_pivot.pivotToSpeaker()).onTrue(Commands.runOnce(()-> m_shooter.setStateSpeaker(ShooterState.SPEAKER)));


    /*
     * SHOOTING controls
     * 
     * B BUTTON: Auto Aim
     * Y BUTTON: Toggle Shooter State
     * A BUTTON: Shoot
     */
    //m_driverController.b().whileTrue(lockMode);

    m_driverController
        .y()
        .onTrue(Commands.runOnce(() -> {
          m_shooter.toggleState(ShooterState.SPEAKER);
        }, m_shooter).andThen(Commands.runOnce(() -> {
          lockMode.setMode(LockMode.SPEAKER_LOCK_MODE);
        })));

    m_driverController.a().whileTrue(m_feeder.shootNote());


    }

  /**
   * Configure the button bindings for the Operator's Controller ({@code m_operatorController})
   */
  private void configureOperatorBindings() {

    /*
     * CLIMBING controls
     * 
     * LEFT Trigger: Climb up
     * RIGHT Trigger: Climb down
     */
    m_operatorController.leftTrigger().whileTrue(m_climber.climbUp());
    m_operatorController.rightTrigger().whileTrue(m_climber.climbDown());


    /*
     * PIVOT auto commands
     * 
     * LEFT D-PAD: Pivot to AMP
     * RIGHT D-PAD: Pivot to Speaker
     * UP D-PAD: Pivot to Range
     * DOWN D-PAD: Pivot to Rest
     */
    m_operatorController.povLeft().whileTrue(m_pivot.pivotToAMP()).onTrue(Commands.runOnce(()-> m_shooter.setStateSpeaker(ShooterState.IDLE)));
    m_operatorController.povRight().whileTrue(m_pivot.pivotToSpeaker()).onTrue(Commands.runOnce(()-> m_shooter.setStateSpeaker(ShooterState.SPEAKER)));
    m_operatorController.povUp().whileTrue(m_pivot.pivotToRange()).onTrue(Commands.runOnce(()-> m_shooter.setStateSpeaker(ShooterState.SPEAKER)));
    m_operatorController.povDown().whileTrue(m_pivot.pivotToRest()).onTrue(Commands.runOnce(()-> m_shooter.setStateSpeaker(ShooterState.IDLE)));

     /*
     * SHOOTING controls
     * 
     * Y BUTTON: Toggle Shooter State
     */
    m_operatorController
        .y()
        .onTrue(Commands.runOnce(() -> {
          m_shooter.toggleState(ShooterState.SPEAKER);
        }, m_shooter).andThen(Commands.runOnce(() -> {
          lockMode.setMode(LockMode.SPEAKER_LOCK_MODE);
        })));

  }

  public void getRumble(){
    if(m_feeder.hasNote()){
      m_driverControllerSP.setRumble(RumbleType.kLeftRumble, 0.1);
    }
    else if (!m_feeder.hasNote()){
      m_driverControllerSP.setRumble(RumbleType.kLeftRumble, 0);
    }

    if (m_shooter.getSpeed() > 0.5){
      m_driverControllerSP.setRumble(RumbleType.kRightRumble, 0.2);
    }

    else if (m_shooter.getSpeed() < 0.5){
      m_driverControllerSP.setRumble(RumbleType.kRightRumble, 0);
    }
  }

  public void setUpAutonomousDashboard(){
    this.autonomousChooser.setDefaultOption("Shoot back up Basic Auto", m_shootBackUp);
    this.autonomousChooser.addOption("Two Shoot Basic Auto", m_twoShootBasicAuto);
    this.autonomousChooser.addOption("Back Up", m_backUp);
    SmartDashboard.putData("Autonomous Chooser", this.autonomousChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // System.out.println("Auton Selected" + AutoChooser.getAuton().getName());
    // return AutoChooser.getAuton();
    // PathPlannerPath path = PathPlannerPath.fromPathFile("example");
    //return autonomousChooser.getSelected();

    /* Shoot and back up */
    // return new SequentialCommandGroup(m_pivot.pivotToSpeaker().withTimeout(2.5).alongWith(Commands.runOnce(() -> 
    //   m_shooter.toggleState(ShooterState.SPEAKER))).andThen(new WaitCommand(0.9)).andThen(m_feeder.shootNote().withTimeout(1.0)).andThen(Commands.runOnce(()-> m_shooter.toggleState(ShooterState.IDLE))).andThen(Commands.run(()-> m_robotDrive.drive(-1,0, 0, false), m_robotDrive).alongWith(m_intake.intake()).withTimeout(1.0)));

    return new PathPlannerAuto("Test Auto #2");
    //PathPlannerPath path = PathPlannerPath.fromPathFile("1 meter forward");

    //Pose2d initialPose = path.getPathPoses().get(0);

    //SmartDashboard.putString("initialPose", initialPose.toString());
    
    //m_robotDrive.resetOdometry(initialPose);

    //return AutoBuilder.followPath(path);
  }

  /**
   * Use this method to pass anything to the dashboard
   * 
   * Reduces multi method use to Shuffleboard
   */
  public void putDashboard() {
    // m_robotDrive.putNumber();
    // SmartDashboard.putNumber("filtered PoseX", m_robotDrive.getPose().getX());
    // SmartDashboard.putNumber("filtered PoseY", m_robotDrive.getPose().getY());
    
  }

  /** Run a function during autonomous to get run time of autonomous. */
  public void autonPeriodic() {
    SmartDashboard.putNumber("Auton Time", Timer.getFPGATimestamp());

  }

  public void launchCommands() {
  }
  
}