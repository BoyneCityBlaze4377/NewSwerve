package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Sec extends Term {
    public Sec(Term coefficient, Term imbedded) {
        super(TermType.sec, coefficient, imbedded);
    }

    public Sec(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public Sec(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return 1 / Math.cos(x);
    }
}
