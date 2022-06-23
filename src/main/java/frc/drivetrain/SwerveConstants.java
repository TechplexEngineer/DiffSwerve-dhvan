package frc.drivetrain;

import frc.robot.Conversion;

public class SwerveConstants {

    public static class DrivetrainConstants{
        public static final int FRONT_LEFT_MOTOR_A = 1;
        public static final int FRONT_LEFT_MOTOR_B = 2;
        public static final int FRONT_LEFT_ENCODER = 9;

        public static final int FRONT_RIGHT_MOTOR_A = 3;
        public static final int FRONT_RIGHT_MOTOR_B = 4;
        public static final int FRONT_RIGHT_ENCODER = 10;

        public static final int BACK_LEFT_MOTOR_A = 5;
        public static final int BACK_LEFT_MOTOR_B = 6;
        public static final int BACK_LEFT_ENCODER = 11;

        public static final int BACK_RIGHT_MOTOR_A = 7;
        public static final int BACK_RIGHT_MOTOR_B = 8;
        public static final int BACK_RIGHT_ENCODER = 12;

        public static final int PIGEON = 13;

        // TODO Check CAD / Ask Matt
        public static final double TRACKWIDTH_METERS = Conversion.inchesToMeters(30);
        public static final double WHEELBASE_METERS = Conversion.inchesToMeters(30);

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 10;
    }

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

    public static class MotorConstants{
        public static final double TICKS_PER_100ms = 2048d / 600d;
    }
}
