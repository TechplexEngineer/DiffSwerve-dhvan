package frc.drivetrain.module;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Conversion;
import frc.drivetrain.SwerveConstants.ModuleConstants;
import frc.drivetrain.SwerveConstants.MotorConstants;

public class Module extends SubsystemBase{
    // Instance of the Module class, used so that only one instance is being passed around different classes
    private static Module m_instance;

    // Motors in the swerve module
    private TalonFX m_motorA;
    private TalonFX m_motorB;

    // RevHex Encoder in the swerve module
    private RevHexEncoder m_yawEncoder;

    // PID for the wheel orientation
    private PIDController m_yawPID;

    // Shuffleboard name for the module
    private String m_sb_Name;

    // Shuffleboard tab for the module
    private ShuffleboardTab m_tab;

    // Shuffleboard layout for the module
    private ShuffleboardLayout m_layout;
    private ShuffleboardLayout m_motorALayout;
    private ShuffleboardLayout m_motorBLayout;

    private NetworkTableEntry sb_moduleAngle;
    private NetworkTableEntry sb_moduleOffset;

    private NetworkTableEntry sb_motorATargetRPM;
    private NetworkTableEntry sb_motorACurrentRPM;

    private NetworkTableEntry sb_motorBTargetRPM;
    private NetworkTableEntry sb_motorBCurrentRPM;


    public Module(int motorA, int motorB, int encoder, double offset, String sb_Name){
        m_motorA = new TalonFX(motorA);
        m_motorB = new TalonFX(motorB);
        m_yawEncoder = new RevHexEncoder(encoder, offset);

        m_sb_Name = sb_Name;
        initializeShuffleboard();

        m_yawPID = new PIDController(ModuleConstants.YAW_kP, ModuleConstants.YAW_kI, ModuleConstants.YAW_kD);
        m_yawPID.enableContinuousInput(-180d, 180d);

        initializeHardware();
    }

    /**
     * Sets all of the initial settings of the motors / encoder in the module
     */
    private void initializeHardware(){
        m_motorA.config_kP(0, ModuleConstants.MOTOR_kP);
        m_motorA.config_kI(0, ModuleConstants.MOTOR_kI);
        m_motorA.config_kD(0, ModuleConstants.MOTOR_kD);

        m_motorB.config_kP(0, ModuleConstants.MOTOR_kP);
        m_motorB.config_kI(0, ModuleConstants.MOTOR_kI);
        m_motorB.config_kD(0, ModuleConstants.MOTOR_kD);
    }

    /**
     * Takes a SwerveModuleState, and sets RPM values to the motor based off of the Yaw and Translation RPMS
     */
    public void setModuleState(SwerveModuleState state){
        // The desired translation RPM from the SwerveModuleState
        double desiredTRPM = Conversion.mpsToRpm(state.speedMetersPerSecond, ModuleConstants.WHEEL_DIAMETER_METERS);
        // The desired yaw RPM from the SwerveModuleState
        double desiredYRPM = m_yawPID.calculate(m_yawEncoder.getDistanceDegrees(), state.angle.getDegrees());

        // The RPM is derived from adding or subtracting (depending on the orientation of the motor) the desired RPMS divided by their gear reductions
        double aRPM = (desiredTRPM / ModuleConstants.GEAR_RATIO_WHEEL_SPEED) - (desiredYRPM / ModuleConstants.GEAR_RATIO_YAW);
        double bRPM = (desiredTRPM / ModuleConstants.GEAR_RATIO_WHEEL_SPEED) + (desiredYRPM / ModuleConstants.GEAR_RATIO_YAW);

        sb_motorATargetRPM.setValue(aRPM);
        sb_motorBTargetRPM.setValue(bRPM);

        // Sets the respective motor rpms, multipled by a constant so that it's in ticks per 100ms, and not revolutions per second
        m_motorA.set(TalonFXControlMode.Velocity, aRPM * MotorConstants.TICKS_PER_100ms);
        m_motorB.set(TalonFXControlMode.Velocity, bRPM * MotorConstants.TICKS_PER_100ms);

    }

    @Override
    public void periodic() {
        
    }
    
    /**
     * Returns the instance of the current module
     * @throws Exception if the instance does not exist yet
     */
    public Module getInstance() throws Exception{
        if(m_instance == null) throw new NullPointerException("Drivetrain instance does not exist");
        else return m_instance;
    }

    // Shuffleboard
    private void initializeShuffleboard(){
        m_tab = Shuffleboard.getTab(m_sb_Name);

        m_layout = m_tab.getLayout(m_sb_Name, BuiltInLayouts.kList);
        m_motorALayout = m_tab.getLayout("Motor A", BuiltInLayouts.kList);
        m_motorBLayout = m_tab.getLayout("Motor B", BuiltInLayouts.kList);

        sb_moduleAngle = m_layout.add("Angle", 0.0).getEntry();
        sb_moduleOffset = m_layout.add("Offset", 0.0).getEntry();

        sb_motorATargetRPM = m_motorALayout.add("Desired RPM", 0.0).getEntry();
        sb_motorACurrentRPM = m_motorALayout.add("Current RPM", 0.0).getEntry();

        sb_motorBTargetRPM = m_motorBLayout.add("Desired RPM", 0.0).getEntry();
        sb_motorBCurrentRPM = m_motorBLayout.add("Current RPM", 0.0).getEntry();
    }

    private void updateShuffleBoard(){
        sb_motorACurrentRPM.setValue(m_motorA.getSelectedSensorVelocity())
    }
}
