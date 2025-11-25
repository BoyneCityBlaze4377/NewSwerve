package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class ArcCos extends Term {
    public ArcCos(Term coefficient, Term imbedded) {
        super(TermType.aCos, coefficient, imbedded);
    }

    public ArcCos(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public ArcCos(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return Math.acos(x);
    }
}
