package nl.gridshore.lejos.sensortest;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.SensorPort;
import lejos.nxt.addon.AccelHTSensor;
import lejos.nxt.addon.GyroSensor;
import lejos.nxt.comm.RConsole;
import lejos.robotics.Accelerometer;
import lejos.robotics.Gyroscope;
import lejos.util.Timer;
import lejos.util.TimerListener;

public class SensorTest {
    public static void main(String[] args) throws InterruptedException {
        RConsole.openUSB(10000);

        LCD.drawString("Free RAM:", 0, 0);
        LCD.drawInt((int) System.getRuntime().freeMemory(), 6, 9, 0);
        Thread.sleep(2000);
        LCD.clear();

        Timer gyroTimer = gyroSensor();
        Timer accelTimer = accelSensor();

        gyroTimer.start();
        accelTimer.start();

        Button.ENTER.waitForPressAndRelease();
        gyroTimer.stop();
        accelTimer.stop();

        RConsole.close();
    }

    private static Timer gyroSensor() {
        final Gyroscope gyroSensor = new GyroSensor(SensorPort.S1, 595);
        gyroSensor.recalibrateOffset();

        return new Timer(100, new TimerListener() {
            public void timedOut() {
                String angularVelocity = Float.toString(gyroSensor.getAngularVelocity());
                LCD.drawString(angularVelocity, 0, 0);
                RConsole.println("Angular velocity: " + angularVelocity);
            }
        });
    }

    private static Timer accelSensor() {
        final Accelerometer accelerometer = new AccelHTSensor(SensorPort.S4);

        return new Timer(100, new TimerListener() {
            int x, y, z;

            public void timedOut() {
                x = accelerometer.getXTilt();
                y = accelerometer.getYTilt();
                z = accelerometer.getZTilt();
                LCD.drawString("accel x : " + x, 0, 1);
                LCD.drawString("accel y : " + y, 0, 2);
                LCD.drawString("accel z : " + z, 0, 3);
                RConsole.println("Acceleration: x,y,z (" + x + "," + y + "," + z + ")");
            }
        });
    }

}
