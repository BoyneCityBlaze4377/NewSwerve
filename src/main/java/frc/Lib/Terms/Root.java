package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Root extends Term {
    private final double m_root;

    public Root(Term coefficient, Term imbedded, double root) {
        super(TermType.root, coefficient, imbedded);
        m_root = root;
    }

    public Root(double coefficient, Term imbedded, double root) {
        this(new Constant(coefficient), imbedded, root);
    }

    public Root(Term imbedded, double root) {
        this(NoCoefficient, imbedded, root);
    }

    @Override
    public double eval(double x) {
        return Math.pow(m_imbedded.eval(x), 1/m_root);
    }
}
