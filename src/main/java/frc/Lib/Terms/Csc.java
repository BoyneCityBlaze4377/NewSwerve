package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Csc extends Term {
    public Csc(Term coefficient, Term imbedded) {
        super(TermType.csc, coefficient, imbedded);
    }

    public Csc(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public Csc(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return 1 / Math.sin(x);
    }
}
