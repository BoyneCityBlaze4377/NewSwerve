package frc.Lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.Lib.SCurveProfile.SCurveState;

/** Add your docs here. */
public class SCurveProfiledPIDController {
    private final PIDController controller;
    private final SCurveProfile profile;

    private double minimumInput, maximumInput;

    public SCurveProfiledPIDController(PIDController controller, SCurveProfile profile) {
        this.controller = controller;
        this.profile = profile;
    }

    public SCurveProfiledPIDController(double kP, double kI, double kD, SCurveProfile profile) {
        this(new PIDController(kP, kI, kD), profile);
    }

    public void setTolerance(double tolerance) {
        controller.setTolerance(tolerance);
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        controller.enableContinuousInput(minimumInput, maximumInput);
        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
    }

    public boolean atSetpoint() {
        return controller.atSetpoint() && Math.abs(controller.getSetpoint() - profile.getGoalState().position) < 1e-6;
    }

    public double calculate(double time, SCurveState currentState) {
        if (controller.isContinuousInputEnabled()) {
            // Get error which is the smallest distance between goal and measurement
            double errorBound = (maximumInput - minimumInput) / 2.0;
            double goalMinDistance = MathUtil.inputModulus(profile.getGoalState().position - currentState.position, 
                                                           -errorBound, errorBound);
            double setpointMinDistance = MathUtil.inputModulus(controller.getSetpoint() - currentState.position, 
                                                               -errorBound, errorBound);

            // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
            // may be outside the input range after this operation, but that's OK because the controller
            // will still go there and report an error of zero. In other words, the setpoint only needs to
            // be offset from the measurement by the input range modulus; they don't need to be equal.
            profile.setGoalState(goalMinDistance + currentState.position);
            controller.setSetpoint(setpointMinDistance + currentState.position);
        }

        SCurveState desiredState = profile.calculate(time, currentState);
        return controller.calculate(currentState.position, desiredState.position);
    }
}
