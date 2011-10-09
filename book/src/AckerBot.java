import lejos.nxt.Button;
import lejos.nxt.Motor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.SteeringPilot;

/**
 * This class is coming from the book Intelligence Unleashed by Brian Bagnall
 * isbn: 978-0-9868322-0-8
 */
public class AckerBot {
    static RegulatedMotor motor;
    static int left = 0;
    static int right = 0;
    static int center = 0;

    public static void main(String[] args) {
        AckerBot.calibrate(MotorPort.C);
        AckerBot.recenter(Motor.C);
        Motor.C.setAcceleration(200);

        double MINTURN_RADIUS = 33.75;
        SteeringPilot pilot =
                new SteeringPilot(SteeringPilot.WHEEL_SIZE_NXT2,Motor.B,false, Motor.C, MINTURN_RADIUS, 48, -42);
//        pilot.arc(MINTURN_RADIUS, 360);
        Navigator navigator = new Navigator(pilot);
        navigator.goTo(40,50,90);
        navigator.goTo(0,0,0);
        Button.ENTER.waitForPressAndRelease();
    }

    public static void calibrate(MotorPort port) {
        NXTMotor m = new NXTMotor(port);
        m.setPower(20);
        m.backward();
        int old = -999999;
        while (m.getTachoCount() != old) {
            old = m.getTachoCount();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
            }
        }
        right = m.getTachoCount();
        center = (59 + right);
        left = center + 59;
    }

    public static void recenter(RegulatedMotor steering) {
        motor = steering;
        motor.setSpeed(100);
        motor.rotateTo(center);
        motor.flt();
        motor.resetTachoCount();
    }
}
