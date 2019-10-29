package org.frc5687.deepspace.robot.utils.math;

public class RigidTranform2D implements Interpolable<RigidTranform2D> {

    private Translation2D _translation;
    private Rotation2D _rotation;

    public RigidTranform2D() {
        _translation = new Translation2D();
        _rotation = new Rotation2D();
    }

    public RigidTranform2D(Translation2D translation, Rotation2D rotation) {
        _translation = translation;
        _rotation = rotation;
    }

    public RigidTranform2D(RigidTranform2D other) {
        _translation = new Translation2D(other._translation);
        _rotation = new Rotation2D(other._rotation);
    }
    public Translation2D getTranslation() {
        return _translation;
    }

    public void setTranslation(Translation2D translation) {
        _translation = translation;
    }

    public Rotation2D getRotation() {
        return _rotation;
    }

    public void setRotation(Rotation2D rotation) {
        _rotation = rotation;
    }

    public RigidTranform2D transformBy(RigidTranform2D other){
        return new RigidTranform2D(_translation.translateBy(other._translation.rotateBy(_rotation)), _rotation.rotateBy(other._rotation));
    }

    @Override
    public RigidTranform2D interpolate(RigidTranform2D other, double x) {
        return new RigidTranform2D(_translation.interpolate(other._translation, x), _rotation.interpolate(other._rotation, x));
    }
}
