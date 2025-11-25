package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Sin extends Term {
    public Sin(Term coefficient, Term imbedded) {
        super(TermType.sin, coefficient, imbedded);
    }

    public Sin(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public Sin(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return Math.sin(x);
    }
}
