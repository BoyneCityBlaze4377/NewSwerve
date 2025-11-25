package frc.Lib;

import frc.Lib.Terms.Constant;
import frc.Lib.Terms.Variable;

/** Add your docs here. */
public class Term {
    public enum TermType {pow, log, exp, constant, root, factorial, var, abs, 
                          sin, cos, tan, sec, csc, cot, aSin, aCos, aTan, aSec, aCsc, aCot};

    public static TermType termType;
    public static Term m_coefficient, m_imbedded;

    public static final Term NoCoefficient = new Constant(1);
    public static final Term Variable = new Variable();

    public Term(TermType type, Term coefficient, Term imbedded) {
        termType = type;
        m_coefficient = coefficient;
        m_imbedded = imbedded;
    }

    public Term(TermType type, double coefficient, Term imbedded) {
        this(type, new Constant(coefficient), imbedded);
    }

    public Term(TermType type, Term imbedded) {
        this(type, NoCoefficient, imbedded);
    }

    public double evaluate(double x) {
        return m_coefficient.eval(x) * this.eval(x);
    };

    public native double eval(double x);
}
