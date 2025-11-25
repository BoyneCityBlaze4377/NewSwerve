package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Abs extends Term {
    public Abs(Term coefficient, Term imbedded) {
        super(TermType.abs, coefficient, imbedded);
    }

    public Abs(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public Abs(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return Math.abs(x);
    }
}
