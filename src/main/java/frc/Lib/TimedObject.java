package frc.Lib;

/** Add your docs here. */
public class TimedObject<ObjectType> {
    public ObjectType object;
    public double time;

    public TimedObject(ObjectType object, double time) {
        this.object = object;
        this.time = time;
    }

    public void setObject(ObjectType object) {
        this.object = object;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public void setValues(ObjectType object, double time) {
        this.object = object;
        this.time = time;
    }

    public ObjectType getObject() {
        return this.object;
    }

    public double getTime() {
        return this.time;
    }
}
