package frc.Lib;

import java.util.ArrayList;

import frc.Lib.Terms.Factorial;

/** Add your docs here. */
public class BezierPath {
    private final AdvancedPose2D[] m_points;

    /**
     * Creates a new {@link BezierPath}
     * 
     * @param points The control points, including starting and ending points, of the Bezier curve
     */
    public BezierPath(AdvancedPose2D... points) {
        m_points = points;
    }

    /** @return The control points of the Bezier curve */
    public AdvancedPose2D[] getControlPoints() {
        return m_points;
    }

    /**
     * Calculate the coefficient of the polynomial term, similar to using pascals triangle
     * 
     * @param n The degree of the polynomial
     * @param i The index (like an array) of the term to calculate the coefficient of
     * 
     * @return The calculated coefficient
     */
    public static double binomialCoefficient(int n, int i) {
        return Factorial.factorial(n) / (Factorial.factorial(i) * Factorial.factorial(n - i));
    }

    /**
     * Calculate a point at a given value of t along the Bezier curve
     * 
     * @param t The desired value of t [0, 1]
     * @return The {@link AdvancedPose2D} of the calculated point (excluding heading)
     */
    public AdvancedPose2D outputPoint(double t) {
        double x = 0;
        double y = 0;
        int n = m_points.length - 1;

        for (int i = 0; i <= n; i++) {
            x += binomialCoefficient(n, i) * Math.pow((1 - t), (n - i)) * Math.pow(t, i) * m_points[i].getX();
            y += binomialCoefficient(n, i) * Math.pow((1 - t), (n - i)) * Math.pow(t, i) * m_points[i].getY();
        }

        return new AdvancedPose2D(x, y);
    }

    /**
     * Calculate the Bezier curve and output it as an {@link ArrayList} of {@link AdvancedPose2D}s
     * 
     * @param numIntermediatePoints The desired number of points, excluding starting and ending points, of the path
     * @return The {@link ArrayList} containing the {@link BezierPath}
     */
    public ArrayList<AdvancedPose2D> outputPath(double numIntermediatePoints) {
        ArrayList<AdvancedPose2D> path = new ArrayList<AdvancedPose2D>();
        double interval = 1 / (numIntermediatePoints + 1);

        for (double t = 0; t <= 1; t += interval) {
            path.add(outputPoint(t));
        }

        return path;
    }


    // BezierPath bezier = new BezierPath();
    // ArrayList<AdvancedPose2D> path = bezier.outputPath(50);

    // for (int i = 0; i < path.toArray().length; i++) {
    //   estimateField.getObject("Point" + i).setPose(path.get(i));
    // }

    // for (int k = 0; k < bezier.getControlPoints().length; k++) {
    //   estimateField.getObject("Control" + k).setPose(bezier.getControlPoints()[k]);
    // }
}
