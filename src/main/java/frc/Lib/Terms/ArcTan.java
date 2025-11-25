package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class ArcTan extends Term {
    public ArcTan(Term coefficient, Term imbedded) {
        super(TermType.aTan, coefficient, imbedded);
    }

    public ArcTan(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public ArcTan(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return Math.atan(x);
    }
}
