package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Factorial extends Term {
    public Factorial(Term coefficient, Term imbedded) {
        super(TermType.factorial, coefficient, imbedded);
    }

    public Factorial(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public Factorial(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    /**
     * Calculate the factorial of a given integer
     * 
     * @param k The integer to calculate the factorial of
     * @return The calculated factorial
     */
    public static double factorial(int k) {
        long m_k = 1;
        for (int i = 1; i <= k; i++) {
            m_k *= i;
        }
        return m_k;
    }

    @Override
    public double eval(double x) {
        return factorial((int) m_imbedded.eval(x));
    }
}
