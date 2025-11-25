package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class ArcSec extends Term {
    public ArcSec(Term coefficient, Term imbedded) {
        super(TermType.aSec, coefficient, imbedded);
    }

    public ArcSec(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public ArcSec(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return 1 / Math.acos(x);
    }
}
