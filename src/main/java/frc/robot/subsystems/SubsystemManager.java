package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class SubsystemManager {
    private final DriveTrain m_driveTrain;
    private final AutoFactory m_factory;

    public SubsystemManager(AutoFactory factory, DriveTrain driveTrain) {
        m_driveTrain = driveTrain;
        m_factory = factory;
    }

    public AutoRoutine example() {
        m_factory.bind("IntakeOrSmth", new Command() {
            //Run Intake
        });

        AutoRoutine routine = m_factory.newRoutine("urmom");
        AutoTrajectory traj = routine.trajectory("AHHHHHH");

        routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));

        Trigger trigger = new Trigger(m_driveTrain::getInRange);
        trigger.and(new Trigger(() -> {return true;})).whileTrue(Commands.run(() -> {/* Smth */}, null));

        return routine;
    }

    public AutoRoutine TEST() {
        AutoRoutine routine = m_factory.newRoutine("TEST");
        AutoTrajectory traj = routine.trajectory("TEST");

        routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
        return routine;
    }
}
