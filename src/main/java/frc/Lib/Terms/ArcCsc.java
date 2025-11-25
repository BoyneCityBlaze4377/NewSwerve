package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class ArcCsc extends Term {
    public ArcCsc(Term coefficient, Term imbedded) {
        super(TermType.aCsc, coefficient, imbedded);
    }

    public ArcCsc(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public ArcCsc(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return 1 / Math.asin(x);
    }
}
