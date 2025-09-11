package org.firstinspires.ftc.teamcode.pedroPathing.PIDFController;

import java.util.function.DoubleSupplier;

/**
 * An example class demonstrating the usage of the PIDF controllers.
 * This class is for demonstration purposes and is not part of the core controller logic.
 * It shows how to instantiate, configure, and use the controllers in a simulated loop.
 */
public class UsageExample {

    public static void main(String[] args) {
        System.out.println("--- PIDF Controller Usage Example ---");

        // --- Linear PIDF Controller Example ---
        System.out.println("\n--- Testing LinearPIDFController ---");
        LinearPIDFController linearController = new LinearPIDFController(0.05, 0.01, 0.001, 0.02, 0.01);

        // Mock battery voltage supplier
        DoubleSupplier mockBattery = () -> 12.5 - Math.random() * 0.5; // Simulate voltage drop
        linearController.enableVoltageCompensation(12.0, mockBattery);
        linearController.setOutputLimits(-0.8, 0.8);
        linearController.setIntegralLimits(-0.2, 0.2);
        linearController.setDerivativeCutoffHz(15, 1.0/50.0); // 15Hz cutoff, 50Hz loop

        double linearTarget = 100.0;
        double currentPosition = 0.0;
        linearController.setTarget(linearTarget);

        System.out.println("Simulating linear slide movement to target: " + linearTarget);
        for (int i = 0; i < 100; i++) {
            double output = linearController.compute(currentPosition, linearTarget);
            // In a real robot, you would apply 'output' to a motor
            // Here, we simulate the effect of the motor on the position
            currentPosition += output * 20; // Simulate movement

            System.out.printf("Step %d: Pos=%.2f, Vel=%.2f, Err=%.2f, Out=%.2f\n",
                i, currentPosition, linearController.getLastVelocity(), linearController.getLastError(), output);

            if (Math.abs(linearController.getLastError()) < 1.0) {
                System.out.println("Target reached!");
                break;
            }
        }

        // --- Rotational PIDF Controller Example ---
        System.out.println("\n--- Testing RotationalPIDFController ---");
        RotationalPIDFController rotationalController = new RotationalPIDFController(0.1, 0.02, 0.005, 0.03, 0.015, 0.05);

        rotationalController.enableVoltageCompensation(12.0, mockBattery);
        rotationalController.setOutputLimits(-1.0, 1.0);

        double rotationalTarget = Math.PI / 2; // 90 degrees
        double currentAngle = 0.0;
        rotationalController.setTarget(rotationalTarget);

        System.out.println("Simulating arm movement to target: " + rotationalTarget + " rad");
        for (int i = 0; i < 150; i++) {
            double output = rotationalController.compute(currentAngle, rotationalTarget, currentAngle);
            currentAngle += output * 0.1; // Simulate arm rotation

            System.out.printf("Step %d: Angle=%.2f, Vel=%.2f, Err=%.2f, Out=%.2f\n",
                i, currentAngle, rotationalController.getLastVelocity(), rotationalController.getLastError(), output);

            if (Math.abs(rotationalController.getLastError()) < 0.05) {
                System.out.println("Target reached!");
                break;
            }
        }

        // --- Velocity PIDF Controller Example ---
        System.out.println("\n--- Testing VelocityPIDFController ---");
        VelocityPIDFController velocityController = new VelocityPIDFController(0.01, 0.005, 0.0001, 0.02, 0.01);

        velocityController.enableVoltageCompensation(12.0, mockBattery);

        double targetVelocity = 50.0; // e.g., ticks per second
        double currentVelocity = 0.0;
        velocityController.setTarget(targetVelocity);

        System.out.println("Simulating motor spin-up to target velocity: " + targetVelocity);
        for (int i = 0; i < 100; i++) {
            double output = velocityController.compute(currentVelocity, targetVelocity);
            // Simulate motor physics: output affects acceleration -> velocity
            double acceleration = output * 10 - 0.5 * currentVelocity; // Simplified physics
            currentVelocity += acceleration * 0.02; // Assuming 50Hz loop (0.02s dt)

            System.out.printf("Step %d: Vel=%.2f, Err=%.2f, Out=%.2f\n",
                i, currentVelocity, velocityController.getLastError(), output);

            if (Math.abs(velocityController.getLastError()) < 2.0) {
                System.out.println("Target velocity reached!");
                break;
            }
        }
    }
}
