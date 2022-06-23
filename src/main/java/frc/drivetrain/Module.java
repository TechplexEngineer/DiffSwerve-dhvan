package frc.drivetrain;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Conversion;
import frc.drivetrain.SwerveConstants.ModuleConstants;
import frc.drivetrain.SwerveConstants.MotorConstants;

public class Module {
    private static Module m_instance;
    private static TalonFX m_motorA;
    private static TalonFX m_motorB;
    private static RevHexEncoder m_yawEncoder;

    private static PIDController m_yawPID;

    public Module(int motorA, int motorB, int encoder, ModuleType type){
        m_motorA = new TalonFX(motorA);
        m_motorB = new TalonFX(motorB);
        m_yawEncoder = new RevHexEncoder(encoder);

        m_yawPID = new PIDController(0.05, 0, 0);
        m_yawPID.enableContinuousInput(-180d, 180d);

        initializeHardware();
    }

    private void initializeHardware(){
        m_motorA.config_kP(0, ModuleConstants.MOTOR_kP);
        m_motorA.config_kI(0, ModuleConstants.MOTOR_kI);
        m_motorA.config_kD(0, ModuleConstants.MOTOR_kD);

        m_motorB.config_kP(0, ModuleConstants.MOTOR_kP);
        m_motorB.config_kI(0, ModuleConstants.MOTOR_kI);
        m_motorB.config_kD(0, ModuleConstants.MOTOR_kD);
    }

    public void setModuleState(SwerveModuleState state){
        double desiredTRPM = Conversion.mpsToRpm(state.speedMetersPerSecond, ModuleConstants.WHEEL_DIAMETER_METERS);
        double desiredYRPM = m_yawPID.calculate(m_yawEncoder.getDistanceDegrees(), state.angle.getDegrees());

        double aRPM = (desiredTRPM / ModuleConstants.GEAR_RATIO_WHEEL_SPEED) - (desiredYRPM / ModuleConstants.GEAR_RATIO_YAW);
        double bRPM = (desiredTRPM / ModuleConstants.GEAR_RATIO_WHEEL_SPEED) + (desiredYRPM / ModuleConstants.GEAR_RATIO_YAW);

        m_motorA.set(TalonFXControlMode.Velocity, aRPM * MotorConstants.TICKS_PER_100ms);
        m_motorB.set(TalonFXControlMode.Velocity, bRPM * MotorConstants.TICKS_PER_100ms);

    }
    
    public  Module getInstance() throws Exception{
        if(m_instance == null) throw new NullPointerException("Drivetrain instance does not exist");
        else return m_instance;
    }
}
