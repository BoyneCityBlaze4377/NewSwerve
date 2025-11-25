package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Cot extends Term {
    public Cot(Term coefficient, Term imbedded) {
        super(TermType.cot, coefficient, imbedded);
    }

    public Cot(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public Cot(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return 1 / Math.tan(x);
    }
}
