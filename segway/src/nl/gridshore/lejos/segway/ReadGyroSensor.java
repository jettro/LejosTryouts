package nl.gridshore.lejos.segway;

import lejos.nxt.Button;
import lejos.nxt.SensorPort;
import lejos.nxt.addon.GyroSensor;
import lejos.nxt.comm.RConsole;
import lejos.util.Delay;

public class ReadGyroSensor extends Thread {
    private static final int OFFSET_SENSOR = 597;
    private static final float INTEGRATION_DRIFT_OFFSET = 0.0005f;

    private final Segway segway;
    private final GyroSensor gyroSensor;

    private float offset;
    private float gyroAngle;

    private static final int OFFSET_SAMPLES = 400;


    public ReadGyroSensor(Segway segway) {
        super();
        this.segway = segway;
        this.gyroSensor = new GyroSensor(SensorPort.S1, OFFSET_SENSOR);
    }

    public float calculateOffset() {
        float gSum;
        int gMin, gMax;

        do {
            gSum = 0.0f;
            gMin = 1000;
            gMax = -1000;

            for (int i = 0; i < OFFSET_SAMPLES; i++) {
                int g = gyroSensor.readValue();

                if (g > gMax){
                    gMax = g;
                }
                if (g < gMin) {
                    gMin = g;
                }

                gSum += g;
                Delay.msDelay(5);
            }
        } while ((gMax - gMin) > 1);   // Reject and sample again if range too large

        //Average the sum of the samples.
        offset = gSum / OFFSET_SAMPLES;
        return offset;
    }

    @Override
    public void run() {

        while (!Button.ESCAPE.isPressed()) {
            int rawValue = gyroSensor.readValue();
            offset = rawValue * INTEGRATION_DRIFT_OFFSET + (1f - INTEGRATION_DRIFT_OFFSET) * offset;
            float angleVelocity = rawValue - offset;
            gyroAngle += angleVelocity * getSampleTime();

            RConsole.println("Angle velocity | angle : " + (int)angleVelocity + " | " + (int)gyroAngle);
        }
    }

    private float getSampleTime() {
        return 0.0055f;
    }

    private String format(int z) {
        String s = Integer.toString(z);
        while (s.length() < 3 && s.charAt(0) != '-') {
            s = "0" + s;
        }
        return s;
    }


}
