package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Tan extends Term {
    public Tan(Term coefficient, Term imbedded) {
        super(TermType.tan, coefficient, imbedded);
    }

    public Tan(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public Tan(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return Math.tan(x);
    }
}
