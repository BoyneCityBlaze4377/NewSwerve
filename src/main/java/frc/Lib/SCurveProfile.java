package frc.Lib;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class SCurveProfile {
    private SCurveConstraints m_constraints;

    private SCurveState m_startingState, m_goalState, m_lastState;

    private double Tj, Ta, Td, Tc, totalTime, 
                   targetDistance, accelerationDistance, decelerationDistance, cruiseDistance,
                   phase1Time, phase2Time, phase3Time, phase4Time, phase5Time, phase6Time;

    private int direction;

    private static double sigChangePosThreshold;
    private static double sigChangeVelThreshold;
    private static double sigChangeAccelThreshold;

    public static class SCurveConstraints {
        public final double maxVelocity;
        public final double maxAcceleration;
        public final double maxJerk;
        public final double period;

        public SCurveConstraints(double maxVelocity, double maxAcceleration, double maxJerk, double period) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxJerk = maxJerk;
            this.period = period;
        }
    }

    public static class SCurveState {
        public double position;
        public double velocity;
        public double acceleration;

        public SCurveState(double position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }

        public SCurveState(double position, double velocity) {
            this(position, velocity, 0);
        }

        public SCurveState(double position) {
            this(position, 0, 0);
        }

        public SCurveState () {
            this(0, 0, 0);
        }
    }

    public static class TimedSCurveState extends SCurveState{
        public double time;

        public TimedSCurveState(SCurveState state, double time) {
            super(state.position, state.velocity, state.acceleration);
            this.time = time;
        }

        public TimedSCurveState(double position, double velocity, double acceleration, double time) {
            this(new SCurveState(position, velocity, acceleration), time);
        }

        public TimedSCurveState (double position, double velocity, double time) {
            this(position, velocity, 0, time);
        }

        public TimedSCurveState(double position) {
            this(position, 0, 0, 0);
        }
    }

    public SCurveProfile(SCurveConstraints constraints, SCurveState startingState, SCurveState goalState) {
        m_constraints = constraints;
        m_goalState = goalState;
        m_startingState = startingState;
        setStartState(startingState);
        setGoalState(goalState);
        configureProfile();
    }

    public SCurveProfile(SCurveConstraints constraints) {
        this(constraints, new SCurveState(), new SCurveState());
    }

    public void setConstraints(SCurveConstraints constraints) {
        m_constraints = constraints;
        configureProfile();
    }

    public void setStartState(SCurveState startState) {
        m_startingState = new SCurveState(configureState(startState).position, configureState(startState).velocity, configureState(startState).acceleration);
        configureProfile();
    }

    public void setGoalState(SCurveState goalState) {
        m_goalState = new SCurveState(configureState(goalState).position, configureState(goalState).velocity, 0);
        configureProfile();
    }

    public void setGoalState(double position, double velocity, double acceleration) {
        setGoalState(new SCurveState(position, velocity, acceleration));
    }

    public void setGoalState(double position) {
        setGoalState(new SCurveState(position));
    }

    private boolean setCurrentState(SCurveState currentState) {
        boolean reconfigured = false;
        if (isSignificantStateChange(currentState)) {
            setStartState(currentState);
            reconfigured = true;
        }
        m_lastState = currentState;
        return reconfigured;
    }

    public SCurveState getGoalState() {
        return m_goalState;
    }

    public double getTotalTime() {
        return totalTime;
    }

    public void configureProfile() {
        sigChangePosThreshold = .5 * Math.pow(m_constraints.maxVelocity, 2) + .01;
        sigChangeVelThreshold = .5 * Math.pow(m_constraints.maxAcceleration, 2) + .01;
        sigChangeAccelThreshold = .5 * Math.pow(m_constraints.maxJerk, 2) + .01;

        Tj = m_constraints.maxAcceleration / m_constraints.maxJerk;

        // Compute acceleration time
        if ((m_constraints.maxVelocity - m_startingState.velocity) < m_constraints.maxAcceleration * Tj) {
            Ta = Math.sqrt((m_constraints.maxVelocity - m_startingState.velocity) / m_constraints.maxJerk) + Tj;
        } else {
            double Tconst = (m_constraints.maxVelocity - m_startingState.velocity - m_constraints.maxAcceleration * Tj) / m_constraints.maxAcceleration;
            Ta = 2 * Tj + Tconst;
        }

        // Compute deceleration time
        if (Math.abs((m_constraints.maxVelocity - m_goalState.velocity)) < m_constraints.maxAcceleration * Tj) {
            Td = Math.sqrt((Math.abs(-(m_constraints.maxVelocity - m_goalState.velocity))) / m_constraints.maxJerk) + Tj;
        } else {
            double Tconst = (Math.abs(-(m_constraints.maxVelocity - m_goalState.velocity)) - m_constraints.maxAcceleration * Tj) / m_constraints.maxAcceleration;
            Td = 2 * Tj + Tconst;
        }
        
        // Distances
        accelerationDistance = (m_startingState.velocity + m_constraints.maxVelocity) / 2.0 * Ta;
        decelerationDistance = (m_constraints.maxVelocity + m_goalState.velocity) / 2.0 * Td;
        targetDistance = Math.abs(m_goalState.position - m_startingState.position);
        cruiseDistance = targetDistance - accelerationDistance - decelerationDistance;

        // Check if there's a cruise phase
        if (cruiseDistance < 0) {
            // No cruise phase, adjust acceleration times to fit within distance
            Tj = Math.cbrt((3 * targetDistance) / (7 * m_constraints.maxJerk));
            Ta = 2 * Tj;
            Tc = 0;

            accelerationDistance = 0.5 * (m_constraints.maxVelocity + m_startingState.velocity) * Ta;
            decelerationDistance = accelerationDistance;
            cruiseDistance = 0;
        } else {
            Tc = cruiseDistance / m_constraints.maxVelocity;
        }

        totalTime = Ta + Tc + Td;

        // Time at the end of each phase of the curve
        phase1Time = Tj;
        phase2Time = Ta - Tj;
        phase3Time = Ta;
        phase4Time = Ta + Tc;
        phase5Time = phase4Time + Tj;
        phase6Time = phase4Time + Td - Tj;

        m_lastState = m_startingState;
    }

    public SCurveState calculate(double time, SCurveState currentState) {
        double t = 0.0, 
               a = m_startingState.acceleration, 
               v = m_startingState.velocity, 
               x = m_startingState.position;
        time = MathUtil.clamp(time, 0, totalTime);

        while (t < time) {
            double jerk;

            if (t < phase1Time) {
                jerk = m_constraints.maxJerk;
            } else if (t < phase2Time) {
                jerk = 0;
            } else if (t < phase3Time) {
                jerk = -m_constraints.maxJerk;
            } else if (t < phase4Time) {
                jerk = 0;
                a = 0;
            } else if (t < phase5Time) {
                jerk = -m_constraints.maxJerk;
            } else if (t < phase6Time) {
                jerk = 0;
            } else if (t <= totalTime) {
                jerk = m_constraints.maxJerk;
            } else {
                jerk = 0;
            }

            double aMidpoint = a + jerk * (m_constraints.period / 2.0);
            double vMidpoint = v + aMidpoint * (m_constraints.period / 2.0);
            
            // Now integrate to the next time step
            a += jerk * m_constraints.period;
            v += aMidpoint * m_constraints.period;
            x += vMidpoint * m_constraints.period;

            t += m_constraints.period;

            if (t >= (Ta + Tc + Td) - m_constraints.period) {
                v = m_goalState.velocity;
                a = m_goalState.acceleration;
                x = m_goalState.position;
            }
        }

        setCurrentState(currentState);
        if (setCurrentState(currentState)) {
            t = 0.0; 
            a = currentState.acceleration; 
            v = currentState.velocity;
            x = currentState.position;
        }

        return new SCurveState(x * direction, v * direction, a * direction);
    }

    public ArrayList<TimedSCurveState> getProfAsList(double time) {
        ArrayList<TimedSCurveState> prof = new ArrayList<TimedSCurveState>();
        double t = 0.0, 
               a = m_startingState.acceleration, 
               v = m_startingState.velocity, 
               x = m_startingState.position;
        time = MathUtil.clamp(time, 0, totalTime);

        int timesRun = 0;

        while (t < time) {
            double jerk;

            if (t < phase1Time) {
                jerk = m_constraints.maxJerk;
            } else if (t < phase2Time) {
                jerk = 0;
            } else if (t < phase3Time) {
                jerk = -m_constraints.maxJerk;
            } else if (t < phase4Time) {
                jerk = 0;
                a = 0;
            } else if (t < phase5Time) {
                jerk = -m_constraints.maxJerk;
            } else if (t < phase6Time) {
                jerk = 0;
            } else if (t <= totalTime) {
                jerk = m_constraints.maxJerk;
            } else {
                jerk = 0;
            }

            // Integrate
            a += jerk * m_constraints.period;
            v += a * m_constraints.period;
            x += v * m_constraints.period;

            if (t >= (Ta + Tc + Td) - m_constraints.period) {
                v = m_goalState.velocity;
                a = m_goalState.acceleration;
                x = m_goalState.position;
            }

            prof.add(new TimedSCurveState(configureState(new SCurveState(x, v, a)), t));

            t += m_constraints.period;
            timesRun++;
            if (timesRun >= 1) {
                setCurrentState(prof.get(timesRun - 1));
                if (setCurrentState(prof.get(timesRun - 1))) {
                    t = 0;
                    a = prof.get(timesRun - 1).acceleration; 
                    v = prof.get(timesRun - 1).velocity;
                    x = prof.get(timesRun - 1).position;
                }
            }
        }

        return prof;
    }

    public static boolean isSignificantStateChange(SCurveState state1, SCurveState state2,
                                                   double positionThreshold, double velocityThreshold,
                                                   double accelerationThreshold) {
        return (Math.abs(state1.position - state2.position) > positionThreshold ||
                Math.abs(state1.velocity - state2.velocity) > velocityThreshold ||
                Math.abs(state1.acceleration - state2.acceleration) > accelerationThreshold);
    }

    public static boolean isSignificantStateChange(SCurveState state1, SCurveState state2) {
        return isSignificantStateChange(state1, state2, sigChangePosThreshold, sigChangeVelThreshold, sigChangeAccelThreshold);
    }

    public boolean isSignificantStateChange(SCurveState currentState) {
        return isSignificantStateChange(currentState, m_lastState, sigChangePosThreshold, sigChangeVelThreshold, sigChangeAccelThreshold);
    }

    public SCurveState configureState(SCurveState in) {
        direction = (int) Math.signum(m_goalState.position - m_startingState.position);
        double position = in.position * direction;
        double velocity = MathUtil.clamp(in.velocity, -m_constraints.maxVelocity, m_constraints.maxVelocity) * direction;
        double acceleration = MathUtil.clamp(in.acceleration, -m_constraints.maxAcceleration, m_constraints.maxAcceleration) * direction;

        return new SCurveState(position, velocity, acceleration);
    }
}
