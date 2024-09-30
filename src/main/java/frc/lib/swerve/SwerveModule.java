package frc.lib.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;

public interface SwerveModule extends Sendable {
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  SwerveModuleState getState();

  /**
   * Returns the last set desired state of the module. Can me useful for keeping the wheels set to a
   * specific orientation when no demand is given, instead of using some default.
   *
   * @return The last set desired state of the module.
   */
  SwerveModuleState getDesiredState();

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  void setDesiredState(SwerveModuleState desiredState);

  /** Reset the drive encoder to zero. */
  void resetEncoders();

  /**
   * Periodic funcion runs at the rate of the swerve drive. Used in the case that a PID controller
   * must be run continuously or similar.
   */
  default void periodic() {}

  // Set up simulation???
  //   public default void simulationPeriodic() {}

  default void testPeriodic() {}

  double getDriveDistanceMeters();

  SwerveModulePosition getPosition();
}