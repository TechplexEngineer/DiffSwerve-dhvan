package frc.drivetrain;

import frc.bionic.Conversion;

public class Constants {
    public static class ModuleConstants{
        public static final double MOTOR_kP = 0.05;
        public static final double MOTOR_kI = 0d;
        public static final double MOTOR_kD = 0d;

        public static final double YAW_kP = 0.05;
        public static final double YAW_kI = 0d;
        public static final double YAW_kD = 0d;

        public static final double GEAR_RATIO_YAW = (13.0 / 82.0);
        public static final double GEAR_RATIO_WHEEL_SPEED = (13.0 / 82.0) * (45.0 / 15.0) * (28.0 / 36.0);

        public static final double WHEEL_DIAMETER_METERS = Conversion.inchesToMeters(3d);
    }

    public class MotorConstants{
        public static final double TICKS_PER_100ms = 2048d / 600d;
    }
}
