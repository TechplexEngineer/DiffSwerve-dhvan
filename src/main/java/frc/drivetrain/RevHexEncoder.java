package frc.drivetrain;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.drivetrain.SwerveConstants.ModuleConstants;


public class RevHexEncoder {
    private DutyCycleEncoder encoder;
    private PIDController pid;

    private double encoderOffset = 0.0;
    private LinearFilter yawDistanceAverage;

    public RevHexEncoder(int dioChannel) {
        this.encoderOffset = 0.0;
        this.yawDistanceAverage = LinearFilter.movingAverage(10);

        encoder = new DutyCycleEncoder(dioChannel);
        encoder.setDistancePerRotation(360.0);

        pid = new PIDController(ModuleConstants.YAW_kP, ModuleConstants.YAW_kI, ModuleConstants.YAW_kD);
        pid.enableContinuousInput(-180.0, 180.0 );
    }

    public double getDistanceDegrees() {
        double currentYawDist = this.yawDistanceAverage.calculate(encoder.getDistance());
        return frc.robot.Conversion.normalize(currentYawDist + encoderOffset, -180, 180);
    }

    public void setGoalDegrees(double goalDegrees) {
        pid.setSetpoint(goalDegrees);
    }

    private double getGoalDegrees() {
        return pid.getSetpoint();
    }

    public double getOutputSignedPercent() {
        double pidCalc = -pid.calculate(getDistanceDegrees(), getGoalDegrees());
        return pidCalc / 180.0;
    }
}
