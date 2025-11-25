package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Ln extends LogBase {
    public Ln(Term coefficient, Term imbedded) {
        super(coefficient, imbedded, Math.E);
        m_imbedded = imbedded;
    }

    public Ln(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public Ln(Term imbedded) {
        this(NoCoefficient, imbedded);
    }
}
