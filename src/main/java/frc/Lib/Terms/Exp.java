package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Exp extends Term {
    private final double m_base;

    public Exp(Term coefficient, Term imbedded, double base) {
        super(TermType.exp, coefficient, imbedded);
        m_base = base;
    }

    public Exp(double coefficient, Term imbedded, double base) {
        this(new Constant(coefficient), imbedded, base);
    }

    public Exp(Term imbedded, double base) {
        this(NoCoefficient, imbedded, base);
    }

    @Override
    public double eval(double x) {
        return Math.pow(m_base, m_imbedded.eval(x));
    }
}
