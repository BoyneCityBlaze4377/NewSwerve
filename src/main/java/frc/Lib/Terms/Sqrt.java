package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Sqrt extends Root {
    public Sqrt(Term coefficient, Term imbedded) {
        super(coefficient, imbedded, 2);
    }

    public Sqrt(double coefficient, Term imbedded) {
        this(new Constant(coefficient), m_imbedded);
    }

    public Sqrt(Term imbedded) {
        this(NoCoefficient, imbedded);
    }
}
