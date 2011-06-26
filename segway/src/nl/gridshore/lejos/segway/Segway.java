package nl.gridshore.lejos.segway;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.addon.GyroSensor;
import lejos.nxt.comm.RConsole;
import lejos.robotics.Gyroscope;
import lejos.util.Delay;
import lejos.util.Timer;
import lejos.util.TimerListener;

/**
 * @author Jettro Coenradie
 * @author Anton Stevense
 */
public class Segway {
    private final static  int DELAY_SAMPLE_IN_MILLISECONDS = 5;
    private final static int DEFAULT_OFFSET = 597;

    private Timer sampler;
    private NXTMotor motorA;
    private NXTMotor motorC;

    public void start() {
        final Gyroscope gyroscope = new GyroSensor(SensorPort.S1, DEFAULT_OFFSET);
        gyroscope.recalibrateOffset();
        sampler = new Timer(DELAY_SAMPLE_IN_MILLISECONDS, new TimerListener() {
            public void timedOut() {
                RConsole.println(Float.toString(gyroscope.getAngularVelocity()));
            }
        });
        sampler.start();
    }

    public void shutDown() {
        sampler.stop();
    }

    /**
     * This function returns a suitable initial gyro offset.  It takes 100 gyro samples over a time of 1/2
     * second and averages them to get the offset.  It also check the max and min during that time
     * and if the difference is larger than one it rejects the data and gets another set of samples.
     *
     * @return The calculated offset in a float
     */
    public float calculateOffset() {
        int numberOffsetSamples = 100;

        GyroSensor gyroSensor = new GyroSensor(SensorPort.S1);
        float sumOfValues;
        int lowestValue, highestValue;

        motorA.setPower(20);
        motorC.setPower(20);

        int numberOffsetMeasurement = 0;
        do {
            numberOffsetMeasurement++;
            RConsole.println("Attempt to calculate offset Gyro number: " + numberOffsetMeasurement);
            sumOfValues = 0.0f;
            lowestValue = 1000;
            highestValue = -1000;

            for (int i = 0; i < numberOffsetSamples; i++) {
                int measuredValue = gyroSensor.readValue();

                if (measuredValue > highestValue) {
                    highestValue = measuredValue;
                }
                if (measuredValue < lowestValue) {
                    lowestValue = measuredValue;
                }

                sumOfValues += measuredValue;
                Delay.msDelay(5);
            }
            RConsole.println("Difference between high and low: " + (highestValue - lowestValue));
        } while ((highestValue - lowestValue) > 1);   // Reject and sample again if range too large

        motorA.setPower(0);
        motorC.setPower(0);

        // Even with motor controller active, the initial offset appears to
        // be off from the actual needed offset to keep robot from wondering.
        // This +1 helps keep robot from wondering when it first starts to
        // balance.
        return (sumOfValues / numberOffsetSamples) + 1;
    }

    public void initializeMotors() {
        motorA = new NXTMotor(MotorPort.A);
        motorC = new NXTMotor(MotorPort.C);
    }

    public static void main(String[] args) {
        RConsole.openUSB(10000);

        Segway segway = new Segway();
        segway.initializeMotors();

//        float offset = segway.calculateOffset();
//        RConsole.println("Calculated offset for the Gyro: " + offset);

        segway.start();
        Button.ENTER.waitForPressAndRelease();
        segway.shutDown();

        RConsole.close();
    }
}
