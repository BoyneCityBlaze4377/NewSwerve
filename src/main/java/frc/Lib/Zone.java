package frc.Lib;

/** Add your docs here. */
public class Zone {
    private final AdvancedPose2D m_BL, m_TR;

    public Zone(AdvancedPose2D BottomLeftPoint, AdvancedPose2D TopRightPoint) {
        m_BL = BottomLeftPoint;
        m_TR = TopRightPoint;
    }

    public double getMinX() {
        return m_BL.getX();
    }

    public double getMaxX() {
        return m_TR.getX();
    }

    public double getMinY() {
        return m_BL.getY();
    }

    public double getMaxY() {
        return m_TR.getY();
    }

    public double getLength() {
        return Math.abs(m_TR.getX() - m_BL.getX());
    }

    public double getWidth() {
        return Math.abs(m_TR.getY() - m_BL.getY());
    }

    public AdvancedPose2D getMidpoint() {
        return m_TR.getMidpoint(m_BL);
    }

    public boolean pointInZone(AdvancedPose2D point, boolean inclusive) {
        return (inclusive ? point.getX() >= getMinX() && point.getX() <= getMaxX() &&
                            point.getY() >= getMinY() && point.getY() <= getMaxY() 
                          : point.getX() > getMinX() && point.getX() < getMaxX() &&
                            point.getY() > getMinY() && point.getY() < getMaxY());
    }

    public boolean pointInZone(AdvancedPose2D point) {
        return pointInZone(point, true);
    }

    public static boolean pointInZone(AdvancedPose2D point, Zone zone, boolean inclusive) {
        return zone.pointInZone(point, inclusive);
    }

    public static boolean pointInZone(AdvancedPose2D point, Zone zone) {
        return zone.pointInZone(point);
    }
}
