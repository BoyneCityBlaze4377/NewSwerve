package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class ArcCot extends Term {
    public ArcCot(Term coefficient, Term imbedded) {
        super(TermType.aCot, coefficient, imbedded);
    }

    public ArcCot(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public ArcCot(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return 1 / Math.atan(x);
    }
}
