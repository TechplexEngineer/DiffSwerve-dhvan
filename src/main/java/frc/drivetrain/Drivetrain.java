package frc.drivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.drivetrain.SwerveConstants.DrivetrainConstants;
import frc.drivetrain.module.Module;

public class Drivetrain extends SubsystemBase{
    // All modules present in the drivetrain
    private Module frontLeftModule;
    private Module frontRightModule;
    private Module backLeftModule;
    private Module backRightModule;

    // Gyroscope being used to keep track of the drivetrain orentation
    private PigeonIMU m_pigeon;

    // ChassisSpeeds object to keep track of all the individual module speeds, and do kinematics
    private ChassisSpeeds m_chassisSpeeds;

    // Instance of the Drivetrain class, used so that only one instance is being passed around different classes
    private static Drivetrain m_instance;

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DrivetrainConstants.TRACKWIDTH_METERS / 2.0, DrivetrainConstants.WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DrivetrainConstants.TRACKWIDTH_METERS / 2.0, -DrivetrainConstants.WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DrivetrainConstants.TRACKWIDTH_METERS / 2.0, DrivetrainConstants. WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DrivetrainConstants.TRACKWIDTH_METERS / 2.0, -DrivetrainConstants.WHEELBASE_METERS / 2.0)
    );

    private final SwerveDriveOdometry m_odometry;

    private Drivetrain(){
        frontLeftModule = new Module(
            DrivetrainConstants.FRONT_LEFT_MOTOR_A, 
            DrivetrainConstants.FRONT_LEFT_MOTOR_B, 
            DrivetrainConstants.FRONT_LEFT_ENCODER,
            DrivetrainConstants.FRONT_LEFT_OFFSET,
            "FRONT_LEFT"
        );

        frontRightModule = new Module(DrivetrainConstants.FRONT_RIGHT_MOTOR_A, 
            DrivetrainConstants.FRONT_RIGHT_MOTOR_B, 
            DrivetrainConstants.FRONT_RIGHT_ENCODER,
            DrivetrainConstants.FRONT_RIGHT_OFFSET,
            "FRONT_RIGHT"
        );

        backLeftModule = new Module(DrivetrainConstants.BACK_LEFT_MOTOR_A, 
            DrivetrainConstants.BACK_LEFT_MOTOR_B, 
            DrivetrainConstants.BACK_LEFT_ENCODER,
            DrivetrainConstants.BACK_LEFT_OFFSET,
            "BACK_LEFT"
        );

        backRightModule = new Module(DrivetrainConstants.BACK_RIGHT_MOTOR_A, 
            DrivetrainConstants.BACK_RIGHT_MOTOR_B, 
            DrivetrainConstants.BACK_RIGHT_ENCODER,
            DrivetrainConstants.BACK_RIGHT_OFFSET,
            "BACK_RIGHT"
        );

        m_pigeon = new PigeonIMU(DrivetrainConstants.PIGEON);

        m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());
    }

    @Override
    public void periodic() {
        // Creates a SwerveModuleStates array from the current chassisSpeeds object
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        // Normalizes the SwerveModuleStates array based off of the MMAX_VELOCITY_METERS_PER_SECOND constant
        SwerveDriveKinematics.normalizeWheelSpeeds(states, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

        // Sets the state of each module
        actuateModules(states);

        // Updates the odometry with the current swerve module states and gyroscope rotation
        m_odometry.update(getGyroscopeRotation(), states);
    }

    /*
     * Sets the chassisSpeeds object from the given x, y, and theta suppliers (Joystick Left X, Left Y, Right X)
     */
    public void drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier){
        m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSupplier.getAsDouble(),
            ySupplier.getAsDouble(),
            thetaSupplier.getAsDouble(),
            Rotation2d.fromDegrees(m_pigeon.getFusedHeading())
        );
    }

    /**
     * Sets the current chassisSpeeds object to the passed in chassisSpeeds object
     * @param chassisSpeeds The chassisSpeeds object to set
     */
    public void drive(ChassisSpeeds chassisSpeeds){
        m_chassisSpeeds = chassisSpeeds;
    }

    /*
     * Sets the states from the SwerveModuleStates array to each individual module
     */
    private void actuateModules(SwerveModuleState[] states){
        frontLeftModule.setModuleState(states[0]);
        frontRightModule.setModuleState(states[1]);
        backLeftModule.setModuleState(states[2]);
        backRightModule.setModuleState(states[3]);
    }


    /**
     * Returns the current gyroscope heading as a Rotation2D object
     */
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
    }

    /**
     * Sets the zero point based off of the current gyroscope heading
     */
    public void zeroGyroscope(){
        m_pigeon.setFusedHeading(0);
    }

    /**
     * Returns the current instance of the drivetrain
     */
    public static Drivetrain getInstance(){
        return m_instance == null ? new Drivetrain() : m_instance;
    }
}
