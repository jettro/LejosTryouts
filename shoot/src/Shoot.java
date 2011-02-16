import lejos.nxt.*;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.Delay;

import java.io.IOException;

public class Shoot {
    private final static float degreeFactor = 5.05f;
    private final static float centimeterFactor = 40;
    private int currentColor = -1;

    // Sensors
    private final ColorSensor lightSensor;
    private final UltrasonicSensor ultrasonicSensor;
    private final DifferentialPilot pilot;
    private final TouchSensor touchSensor1;

    public static void main(String[] args) throws InterruptedException, IOException {
        Shoot shoot = new Shoot();
        shoot.driveTillBlocked();
        shoot.playSound();
//        shoot.rotate(180);
    }

    public Shoot() {
        lightSensor = new ColorSensor(SensorPort.S3, ColorSensor.TYPE_COLORFULL);
        ultrasonicSensor = new UltrasonicSensor(SensorPort.S4);
        ultrasonicSensor.continuous();
        pilot = new DifferentialPilot(2.9f, 15.2f, Motor.C, Motor.B);
        touchSensor1 = new TouchSensor(SensorPort.S1);
        burstGreenColor();
    }

    public void driveTillBlocked() {
        burstGreenColor();
        pilot.setTravelSpeed(15);
        pilot.forward();

        int i;
        do {
            i = ultrasonicSensor.getDistance();
            Delay.msDelay(100);
            LCD.drawString("distance : " + i, 0, 1);
        } while (i > 20 || i == 255);

        pilot.stop();
        burstBlueColor();

        pilot.setTravelSpeed(5);
        pilot.forward();
        do {
            Delay.msDelay(10);
        } while (!touchSensor1.isPressed());
        pilot.stop();
        burstRedColor();
    }

    public void usePilot() {
        DifferentialPilot pilot = new DifferentialPilot(2.9f, 15.2f, Motor.C, Motor.B);
        pilot.setTravelSpeed(5);
        pilot.travel(10);
        pilot.setTravelSpeed(25);
        pilot.travel(-10);
        pilot.setTravelSpeed(5);
        while (!Button.ENTER.isPressed()) {
            pilot.arcForward(20);
        }
//        pilot.rotate(90);
    }

    public void writeDistance() {
        int i = ultrasonicSensor.getDistance();
        LCD.drawString("distance : " + i, 0, 1);
    }

    public void playSound() {
        Sound.buzz();
    }

    private void burstRedColor() {
        lightSensor.setFloodlight(ColorSensor.Color.RED);
    }

    private void burstGreenColor() {
        lightSensor.setFloodlight(ColorSensor.Color.GREEN);
    }

    private void burstBlueColor() {
        lightSensor.setFloodlight(ColorSensor.Color.BLUE);
    }


    private void scanForColor() {
        while (!Button.ESCAPE.isPressed()) {
            ColorSensor lightSensor = new ColorSensor(SensorPort.S3, ColorSensor.TYPE_COLORFULL);
            ColorSensor.Color readColor = lightSensor.getColor();
            this.writeColor(readColor.getColor());
        }
    }

    public void shoot() {
        burstRedColor();
        Motor.A.resetTachoCount();
        Motor.A.setSpeed(400);
        Motor.A.forward();
        int count = 0;
        while (count < 360) count = Motor.A.getTachoCount();
        Motor.A.stop();
        burstGreenColor();
    }

    public void writeColor(int color) {
        if (currentColor != color && color > ColorSensor.Color.NONE) {
            currentColor = color;
            LCD.clear();
            LCD.drawString("Read color : ", 0, 0);
            LCD.drawString("color : " + color, 0, 1);
        }
    }

    public void turn() {
        rotate(180);
    }

    public void rotate(int degrees) {
        Motor.B.setSpeed(200);
        Motor.C.setSpeed(200);
        int inDegrees = Math.round(degrees * degreeFactor);
        Motor.B.rotate(inDegrees, true);
        Motor.C.rotate(-inDegrees, false);
    }

    public void drive(int centimeter) {
        Motor.B.setSpeed(200);
        Motor.C.setSpeed(200);

        int inCycles = Math.round(centimeter * centimeterFactor);
        Motor.B.rotate(inCycles, true);
        Motor.C.rotate(inCycles, false);
    }
}