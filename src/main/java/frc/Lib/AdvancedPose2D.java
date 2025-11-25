package frc.Lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Constants.FieldConstants;

public class AdvancedPose2D extends Pose2d {
    public AdvancedPose2D() {
        super();
    }

    public AdvancedPose2D(Translation2d translation, Rotation2d rotation){
        super(translation, rotation);
    }

    public AdvancedPose2D(double x, double y) {
        super(x, y, new Rotation2d());
    }

    public AdvancedPose2D(double x, double y, Rotation2d rotation){
        super(new Translation2d(x, y), rotation);
    }

    public AdvancedPose2D(double x, double y, double degrees) {
        super(new Translation2d(x, y), Rotation2d.fromDegrees(degrees));
    }

    public AdvancedPose2D(Pose2d pose2d) {
        super(pose2d.getTranslation().getX(), pose2d.getTranslation().getY(), pose2d.getRotation());
    }

    public AdvancedPose2D rotateBy(Rotation2d rotation) {
        return new AdvancedPose2D(this.getTranslation(), Rotation2d.fromDegrees(MathUtil.inputModulus(
                                                            this.getHeadingDegrees() + rotation.getDegrees(), -180, 180)));
    }

    public double getHeadingDegrees() {
        return this.getRotation().getDegrees();
    }

    public AdvancedPose2D horizontallyFlip(){
        return new AdvancedPose2D(new Translation2d(FieldConstants.fieldLength - this.getTranslation().getX(),
                                                    this.getTranslation().getY()),
                                  Rotation2d.fromDegrees((this.getRotation().getDegrees() > 0) ?
                                                          180 - this.getRotation().getDegrees() :
                                                        -(180 + this.getRotation().getDegrees())));
    }

    public AdvancedPose2D verticallyFlip() {
        return new AdvancedPose2D(new Translation2d(this.getTranslation().getX(),
                                                    FieldConstants.fieldWidth - this.getTranslation().getY()), 
                                                    Rotation2d.fromDegrees(-this.getRotation().getDegrees()));
    }

    public AdvancedPose2D flipBoth() {
        return new AdvancedPose2D(new Translation2d(FieldConstants.fieldLength - this.getTranslation().getX(),
                                                    FieldConstants.fieldWidth - this.getTranslation().getY()),
                                  Rotation2d.fromDegrees((this.getRotation().getDegrees() > 0) ?
                                                          this.getRotation().getDegrees() - 180 :
                                                          this.getRotation().getDegrees() + 180));
    }

    /**
     * Apply a rotated transfromation to the {@link AdvancedPose2D} object
     * 
     * @param direction the direction of the translation
     * @param translation Y positive goes front and X positive goes Right
     * @param desiredHeading the heading of processed pose
     * @return the transformed {@link AdvancedPose2D} object
     */
    public AdvancedPose2D withVector(Rotation2d direction, Translation2d translation, Rotation2d desiredHeading) {
        double x = translation.getY() * direction.getCos() + translation.getX() * direction.getSin() + this.getX();
        double y = translation.getY() * direction.getSin() - translation.getX() * direction.getCos() + this.getY();
        return new AdvancedPose2D(x, y, desiredHeading);
    }

    /**
     * apply a robot-relative transformation to the {@link AdvancedPose2D} object
     * @param transformation Y positive goes front and X positive goes Right
     * @return the transformed {@link AdvancedPose2D} object
     */
    public AdvancedPose2D withRobotRelativeTransformation(Translation2d transformation) {
        return this.withVector(this.getRotation(), transformation, this.getRotation());
    }

    public AdvancedPose2D average(AdvancedPose2D other, boolean averageHeading) {
        return new AdvancedPose2D((this.getX() + other.getX()) / 2, (this.getY() + other.getY()) / 2, 
                                  averageHeading ? Rotation2d.fromDegrees((this.getHeadingDegrees() + other.getHeadingDegrees()) / 2)
                                                 : this.getRotation());
    }

    public AdvancedPose2D getMidpoint(AdvancedPose2D other) {
        return this.average(other, false);
    }

    public double getDistance(AdvancedPose2D other) {
        return this.getTranslation().getDistance(other.getTranslation());
    }

    public double getXDistance(AdvancedPose2D other) {
        return Math.abs(other.getX() - this.getX());
    }

    public double getYDistance(AdvancedPose2D other) {
        return Math.abs(other.getY() - this.getY());
    }

    public Rotation2d getAngleTowards(AdvancedPose2D other) {
        double angle = this.getRotation().getRadians();
        if (!(Math.abs(this.getYDistance(other)) < 1e-6 && Math.abs(this.getXDistance(other)) < 1e-6)) {
            if (this.getXDistance(other) < 0) {
                angle = Math.copySign(Math.PI, this.getYDistance(other)) + 
                        Math.atan(this.getYDistance(other) / this.getXDistance(other));
            } else {
                angle = Math.atan(this.getYDistance(other) / this.getXDistance(other));
            }
        }

        return Rotation2d.fromRadians(angle);
    }
}