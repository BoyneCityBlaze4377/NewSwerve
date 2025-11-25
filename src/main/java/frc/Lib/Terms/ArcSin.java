package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class ArcSin extends Term {
    public ArcSin(Term coefficient, Term imbedded) {
        super(TermType.aSin, coefficient, imbedded);
    }

    public ArcSin(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public ArcSin(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return Math.asin(x);
    }
}
