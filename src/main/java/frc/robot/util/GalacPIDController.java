package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

public class GalacPIDController {

    public PIDController innerController;
    private double setpoint;
    private Supplier<Double> measurementSupplier;
    private double minEffort;

    public PIDController getInnerController() {
        return this.innerController;
    }

    public void setInnerController(PIDController innerController) {
        this.innerController = innerController;
    }

    public void setPositionTolerance(double tolerance) {
        innerController.setTolerance(tolerance);
    }
    public void setTolerance(double tolerancePos, double toleranceVel) {
        innerController.setTolerance(tolerancePos, toleranceVel);
    }

    public double getSetpoint() {
        return this.setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public Supplier<Double> getMeasurementSupplier() {
        return this.measurementSupplier;
    }

    public void setMeasurementSupplier(Supplier<Double> measurementSupplier) {
        this.measurementSupplier = measurementSupplier;
    }

    public double getMinEffort() {
        return this.minEffort;
    }

    public void setMinEffort(double minEffort) {
        this.minEffort = minEffort;
    }
    
    /**
     * 
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     * @param minEffort Minimum effort to be applied to motor
     * @param measurement Supplier of reading from feedback device (ex: encoder value)
     * @param initialSetpoint Goal measurement reading, the controller aims to make current measurement reach this value
     * @param positionTolerance Absolute acceptable positional error (velocity tolerance may also be set)
     */
    public GalacPIDController(double p, double i, double d, double minEffort, Supplier<Double> measurement,
            double initialSetpoint, double positionTolerance) {
        innerController = new PIDController(p,i,d);
        this.setpoint = initialSetpoint;
        this.measurementSupplier = measurement;
        this.minEffort = minEffort;
        innerController.setTolerance(positionTolerance);
    }

    public double getEffort() {

        double mez = measurementSupplier.get();
        double effort = innerController.calculate(mez, setpoint);

        if(!isFinished()) {
            return (minEffort * (-Math.signum(mez) )) + effort;

        } else {
            return 0; 
        }

    }

    public boolean isFinished() {
        return innerController.atSetpoint();
    }
}