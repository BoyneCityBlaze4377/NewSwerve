package frc.Lib;

/** Add your docs here. */
public class TimedValue extends TimedObject<Double> {

    public TimedValue(double value, double time) {
        super(value, time);
    }

    public void setValue(double value) {
        this.setObject(value);
    }

    public void setValueAndTime(double value, double time) {
        this.setValues(value, time);
    }

    public double getValue() {
        return this.object;
    }

    public double getAverage(TimedValue other) {
        return (this.object + other.object) / 2;
    }

    public double getRateOfChange(TimedValue other) {
        return (this.object - other.object) / (this.time - other.time);
    }
}
