package org.lejos.example;

import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.RangeReading;
import lejos.robotics.objectdetection.*;
import lejos.nxt.Motor;

/**
 * Example leJOS Project with an ant build file
 */
public class App {

    private static final float DISTANCE_BETWEEN_SENSORS_CM = 5f;
    public static final int CLOSE_DISTANCE_CM = 40;

    private static float currentRotation = 0f;

    public static void main(String[] args) throws Exception {
        final long interval = 60000;
        final long startTime = System.currentTimeMillis();

        DirectionDetector detector = new DirectionDetector(
                ultrasonicDetectorForPort(SensorPort.S1),
                ultrasonicDetectorForPort(SensorPort.S4),
                DISTANCE_BETWEEN_SENSORS_CM
        );
        final HackPilot p = new HackPilot(5f, Motor.A, Motor.B, 41, -50, 50);

        while (System.currentTimeMillis() - startTime < interval) {
            Feature feature = detector.scan();
            if (feature == null) {
                Motor.C.stop();
                p.travelArc(-currentRotation, 20);
                currentRotation = 0f;
                continue;
            }

            RangeReading reading = feature.getRangeReading();
            System.out.println(reading.getAngle() + " " + reading.getRange());
            if (reading.getAngle() < 0) {
                Motor.C.forward();

                currentRotation = (currentRotation + 30f) % 30;

                if (reading.getRange() < CLOSE_DISTANCE_CM) {
                    p.travelArc(-30, -40);
                    p.travelArc(60, 40);
                } else {
                    p.travelArc(30, 40);
                }

            } else {
                Motor.C.forward();

                currentRotation = (currentRotation - 30f) % 30;

                if (reading.getRange() < CLOSE_DISTANCE_CM) {
                    p.travelArc(30, -40);
                    p.travelArc(-60, 40);
                } else {
                    p.travelArc(-30, 40);
                }
            }
        }
    }

    private static RangeFeatureDetector ultrasonicDetectorForPort(SensorPort port) {
        return new RangeFeatureDetector(new UltrasonicSensor(port), 40f, 250);
    }
}
