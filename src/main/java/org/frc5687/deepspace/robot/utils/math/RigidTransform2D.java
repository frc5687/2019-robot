package org.frc5687.deepspace.robot.utils.math;

public class RigidTransform2D implements Interpolable<RigidTransform2D> {

    public Rotation2D rotationMat;
    public Translation2D translationMat;

    public RigidTransform2D() {
        rotationMat = new Rotation2D();
        translationMat = new Translation2D();
    }

    public RigidTransform2D(Translation2D translation, Rotation2D rotation) {
        rotationMat = rotation;
        translationMat = translation;
    }

    @Override
    public RigidTransform2D interpolate(RigidTransform2D other, double percentage) {
        return new RigidTransform2D(this.translationMat.interpolate(other.translationMat, percentage), this.rotationMat.interpolate(other.rotationMat, percentage));
    }

    public RigidTransform2D transform(RigidTransform2D delta) {
        return new RigidTransform2D(translationMat.translateBy(delta.translationMat.rotateBy(rotationMat)), rotationMat.rotateBy(delta.rotationMat));
    }
}
