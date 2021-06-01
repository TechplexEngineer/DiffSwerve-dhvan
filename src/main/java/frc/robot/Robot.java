/*
 * Team 4909, Bionics
 * Billerica Memorial High School
 *
 * Copyright:
 *   2021 Bionics
 *
 * License:
 *   MIT: https://opensource.org/licenses/MIT
 *   See the LICENSE file in the project's top-level directory for details.
 */

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.bionic.TrajectoryFollow;
import frc.bionic.UserInterfaceElement;
import frc.bionic.swerve.AbstractDrivetrain;
import frc.bionic.swerve.Vision;
import frc.bionic.swerve.debug.DebugDash;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class Robot extends TimedRobot {
  private AbstractDrivetrain drivetrain;

  private ShooterSubsystem shooter;
  private Vision vision;

  public static DebugDash debugDash = null;

  // Shuffleboard-related
  NetworkTableEntry sb_robot_type;

  public Robot() {
    super();

    // uncomment one or the other
    // drivetrain = new frc.peyton.Drivetrain();
    drivetrain = new frc.team4909.Drivetrain();
    // UserInterface.registerObject("Drivetrain", new UserInterfaceElement<AbstractDrivetrain>(drivetrain));

    // UserInterface.createDefaultUI();
    UserInterface.createUIJoystick0(drivetrain);
    debugDash = new DebugDash(drivetrain);

  }

  @Override
  public void robotInit() {
    shooter = new ShooterSubsystem();
    // TODO: change values based on Limelight mounting
    vision = new Vision(0, 0, 0, 0, 0);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void robotPeriodic() {
    // System.out.println("RobotPerodic");
    CommandScheduler.getInstance().run();

    debugDash.periodic();
  }

  @Override
  public void teleopInit() {

  }

  // @Override
  // public void autonomousInit() {
  //   // Starts the process of following a trajectory
  //   new TrajectoryFollow().getTrajectoryCommand(drivetrain, "paths/flower.json").schedule();
  //   new TrajectoryFollow().getTrajectoryCommand(drivetrain, "paths/line.json").schedule();

  // }
}
