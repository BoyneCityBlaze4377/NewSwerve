package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Variable extends Term {
    public Variable(Term coefficient) {
        super(TermType.var, coefficient, null);
    }

    public Variable(double coefficient) {
        this(new Constant(coefficient));
    }

    public Variable() {
        this(NoCoefficient);
    }

    @Override
    public double eval(double x) {
        return x;
    }
}
