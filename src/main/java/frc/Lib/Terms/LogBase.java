package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class LogBase extends Term {
    private final double m_base;

    public LogBase(Term coefficient, Term imbedded, double base) {
        super(TermType.log, coefficient, imbedded);
        m_base = base;
    }

    public LogBase(double coefficient, Term imbedded, double base) {
        this(new Constant(coefficient), imbedded, base);
    }

    public LogBase(Term imbedded, double base) {
        this(NoCoefficient, imbedded, base);
    }

    @Override
    public double eval(double x) {
        return Math.log(m_imbedded.eval(x)) / Math.log(m_base);
    }
}
