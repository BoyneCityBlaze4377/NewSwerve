package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Cos extends Term {
    public Cos(Term coefficient, Term imbedded) {
        super(TermType.cos, coefficient, imbedded);
    }

    public Cos(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public Cos(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return Math.cos(x);
    }
}
