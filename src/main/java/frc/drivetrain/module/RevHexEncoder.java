package frc.drivetrain.module;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.drivetrain.SwerveConstants.ModuleConstants;


public class RevHexEncoder {
    // DutyCycleEncoder that represents the RevHexEncoder
    private DutyCycleEncoder encoder;

    // PID for the rotation of the encoder
    private PIDController pid;

    // Offset of the encoder, can be set in the constructor to insure that the encoder is always at 0 when it initializes
    private double encoderOffset;

    // idk
    private LinearFilter yawDistanceAverage;

    /**
     * Constructs a RevHexEncoder object
     * @param offset value when the encoder is at the desired 0 location
     */
    public RevHexEncoder(int dioChannel, double offset) {
        this.encoderOffset = offset;
        this.yawDistanceAverage = LinearFilter.movingAverage(10);

        encoder = new DutyCycleEncoder(dioChannel);
        encoder.setDistancePerRotation(360.0);

        pid = new PIDController(ModuleConstants.YAW_kP, ModuleConstants.YAW_kI, ModuleConstants.YAW_kD);
        pid.enableContinuousInput(-180.0, 180.0 );
    }

    /**
     * Returns the degrees traveled by the encoder
     */
    public double getDistanceDegrees() {
        double currentYawDist = this.yawDistanceAverage.calculate(encoder.getDistance());
        return frc.robot.Conversion.normalize(currentYawDist + encoderOffset, -180, 180);
    }

    /**
     * Set the setpoint of the pid to goalDegrees
     */
    public void setGoalDegrees(double goalDegrees) {
        pid.setSetpoint(goalDegrees);
    }

    /**
     * Returns the current pid setpoint
     */
    private double getGoalDegrees() {
        return pid.getSetpoint();
    }

    /**
     * Returns the calulcated output of the pid controller
     */
    public double getOutputSignedPercent() {
        double pidCalc = -pid.calculate(getDistanceDegrees(), getGoalDegrees());
        return pidCalc / 180.0;
    }
}
