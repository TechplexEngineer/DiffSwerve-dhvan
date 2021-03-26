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

package frc.peyton;

import frc.bionic.swerve.*;

public class PeytonSwerveModule extends frc.bionic.swerve.SwerveModule
{
  // Gear ratio of first two pairs of gears.
  // Yaw does not depend on the third pair.
  private static final double       GEAR_RATIO_YAW          =  (10.0/80.0) * (34.0/90.0);

  // Gear ratio of all three pairs of gears.
  // Wheel speed depends on all three.
  private static final double       GEAR_RATIO_WHEEL_SPEED  =  (10.0/80.0) * (34.0/90.0) * (82.0/21.0);

  // Maximum yaw speed in RPM
  private static final double       MAX_YAW_SPEED_RPM       = 200;

  // The wheel diameter, in meters
  private static final double      WHEEL_DIAMETER_METERS    = frc.bionic.Conversion.inchesToMeters(3.0);

  public PeytonSwerveModule(int pwmMotorChannelA, int pwmMotorChannelB,
                            int dioEncoderChannelA, int dioEncoderChannelB,
                            String name, String shuffleboardTabName)
  {
    super(name, shuffleboardTabName);

    // Instantiate the two swerve motors our swerve module incorporates
    IMotor motorA = new MotorCANSparkMaxNeo(pwmMotorChannelA, name + " A", shuffleboardTabName);
    IMotor motorB = new MotorCANSparkMaxNeo(pwmMotorChannelB, name + " B", shuffleboardTabName);

    // Instantiate the yaw encoder our swerve module incorporates
    IYawEncoder encoder = new YawEncoderGrayhill63R128(dioEncoderChannelA, dioEncoderChannelB, 
                                           name + " Grayhill", shuffleboardTabName);

    initialize(GEAR_RATIO_YAW, GEAR_RATIO_WHEEL_SPEED, MAX_YAW_SPEED_RPM, 
               motorA, motorB, encoder, WHEEL_DIAMETER_METERS);
  }
}
