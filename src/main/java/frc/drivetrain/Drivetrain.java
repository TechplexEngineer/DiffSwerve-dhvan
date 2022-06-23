package frc.drivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.drivetrain.SwerveConstants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase{
    private Module frontLeftModule;
    private Module frontRightModule;
    private Module backLeftModule;
    private Module backRightModule;

    private PigeonIMU m_pigeon;

    private ChassisSpeeds m_chassisSpeeds;

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

    private Drivetrain(){
        frontLeftModule = new Module(DrivetrainConstants.FRONT_LEFT_MOTOR_A, DrivetrainConstants.FRONT_LEFT_MOTOR_B, DrivetrainConstants.FRONT_LEFT_ENCODER, ModuleType.FRONT_LEFT);
        frontRightModule = new Module(DrivetrainConstants.FRONT_RIGHT_MOTOR_A, DrivetrainConstants.FRONT_RIGHT_MOTOR_B, DrivetrainConstants.FRONT_RIGHT_ENCODER, ModuleType.FRONT_RIGHT);
        backLeftModule = new Module(DrivetrainConstants.BACK_LEFT_MOTOR_A, DrivetrainConstants.BACK_LEFT_MOTOR_B, DrivetrainConstants.BACK_LEFT_ENCODER, ModuleType.BACK_LEFT);
        backRightModule = new Module(DrivetrainConstants.BACK_RIGHT_MOTOR_A, DrivetrainConstants.BACK_RIGHT_MOTOR_B, DrivetrainConstants.BACK_RIGHT_ENCODER, ModuleType.BACK_RIGHT);

        m_pigeon = new PigeonIMU(DrivetrainConstants.PIGEON);
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

        actuateModules(states);
    }

    public void drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier){
        m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSupplier.getAsDouble(),
            ySupplier.getAsDouble(),
            thetaSupplier.getAsDouble(),
            Rotation2d.fromDegrees(m_pigeon.getFusedHeading())
        );
    }

    private void actuateModules(SwerveModuleState[] states){
        frontLeftModule.setModuleState(states[0]);
        frontRightModule.setModuleState(states[1]);
        backLeftModule.setModuleState(states[2]);
        backRightModule.setModuleState(states[3]);
    }

    public void zeroGyroscope(){
        m_pigeon.setFusedHeading(0);
    }
}
