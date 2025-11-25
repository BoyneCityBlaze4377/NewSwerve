package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class EExp extends Exp {
    public EExp(Term coefficient, Term imbedded) {
        super(coefficient, imbedded, Math.E);
    }

    public EExp(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public EExp(Term imbedded) {
        this(NoCoefficient, imbedded);
    }
}
