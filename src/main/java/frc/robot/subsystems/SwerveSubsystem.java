package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveSubsystem extends SubsystemBase {

    /**
     * The object representing the swerve drive
     */
    private SwerveDrive swerveDrive;

    /**
     * Constructor for the Swerve Subsystem 
     */
    public SwerveSubsystem () {
        try {
          swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(Constants.MAX_SPEED);
        } catch (IOException e) {
          e.printStackTrace();
        }     
    }

    /**
     * Gets the {@link SwerveDrive} object.
     * 
     * @return the {@link SwerveDrive} object
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
    }

    /**
     * Gets the field relative velocity of the robot.
     * 
     * @return A {@link ChassisSpeeds} object of the robot's field velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current pose of the robot (position and rotation) as reported by odometry
     * 
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading()
    {
        return getPose().getRotation();
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object
     * 
     * @return The {@link SwerveDriveConfiguration} for the current drive
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
    * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
    * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
    * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
    *
    * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
    *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
    *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
    *                      (field North) and positive y is torwards the left wall when looking through the driver station
    *                      glass (field West).
    * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
    *                      relativity.
    * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
    */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }
}
