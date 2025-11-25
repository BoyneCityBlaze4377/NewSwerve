package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Pow extends Term {
    private final double m_exp;

    public Pow(Term coefficient, Term imbedded, double exponent) {
        super(TermType.pow, coefficient, imbedded);
        m_exp = exponent;
    }

    public Pow(double coefficient, Term imbedded, double exponent) {
        super(TermType.pow, coefficient, imbedded);
        m_exp = exponent;
    }

    public Pow(Term imbedded, double exponent) {
        this(NoCoefficient, imbedded, exponent);
    }

    @Override
    public double eval(double x) {
        return Math.pow(m_imbedded.eval(x), m_exp);
    }
}
