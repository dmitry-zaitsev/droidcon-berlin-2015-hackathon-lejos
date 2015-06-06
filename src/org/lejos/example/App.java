package org.lejos.example;

import lejos.nxt.SensorPort;
import lejos.nxt.SoundSensor;

/**
 * Example leJOS Project with an ant build file
 */
public class App {

    public static void main(String[] args) throws Exception {
        SoundSensor soundSensor = new SoundSensor(SensorPort.S4);

        final long interval = 30000;
        final long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < interval) {
            System.out.println("Sound: " + soundSensor.readValue());
        }
    }
}
