package com.sroy.stepcounter;

public class StepDetector {

    private static final int ACCELERATOR_RING_SIZE = 50;
    private static final int VEL_RING_SIZE = 10;

    private static final float STEP_THRESHOLD = 50f;

    private static final int STEP_DELAY_NS = 250000000;

    private int acceleratorRingCounter = 0;
    private float[] accelRingX = new float[ACCELERATOR_RING_SIZE];
    private float[] accelRingY = new float[ACCELERATOR_RING_SIZE];
    private float[] accelRingZ = new float[ACCELERATOR_RING_SIZE];
    private int velRingCounter = 0;
    private float[] velRing = new float[VEL_RING_SIZE];
    private long lastStepTimeNs = 0;
    private float oldVelocityEstimate = 0;

    private StepListener listener;

    public void registerListener(StepListener listener) {
        this.listener = listener;
    }


    public void updateAccel(long timeNs, float x, float y, float z) {
        float[] currentAccelerator = new float[3];
        currentAccelerator[0] = x;
        currentAccelerator[1] = y;
        currentAccelerator[2] = z;

        acceleratorRingCounter++;
        accelRingX[acceleratorRingCounter % ACCELERATOR_RING_SIZE] = currentAccelerator[0];
        accelRingY[acceleratorRingCounter % ACCELERATOR_RING_SIZE] = currentAccelerator[1];
        accelRingZ[acceleratorRingCounter % ACCELERATOR_RING_SIZE] = currentAccelerator[2];

        float[] valueX = new float[3];
        valueX[0] = SensorFilter.sum(accelRingX) / Math.min(acceleratorRingCounter, ACCELERATOR_RING_SIZE);
        valueX[1] = SensorFilter.sum(accelRingY) / Math.min(acceleratorRingCounter, ACCELERATOR_RING_SIZE);
        valueX[2] = SensorFilter.sum(accelRingZ) / Math.min(acceleratorRingCounter, ACCELERATOR_RING_SIZE);

        float normalization_factor = SensorFilter.norm(valueX);

        valueX[0] = valueX[0] / normalization_factor;
        valueX[1] = valueX[1] / normalization_factor;
        valueX[2] = valueX[2] / normalization_factor;

        float currentX = SensorFilter.dot(valueX, currentAccelerator) - normalization_factor;
        velRingCounter++;
        velRing[velRingCounter % VEL_RING_SIZE] = currentX;

        float velocityEstimate = SensorFilter.sum(velRing);

        if (velocityEstimate > STEP_THRESHOLD && oldVelocityEstimate <= STEP_THRESHOLD
                && (timeNs - lastStepTimeNs > STEP_DELAY_NS)) {
            listener.step(timeNs);
            lastStepTimeNs = timeNs;
        }
        oldVelocityEstimate = velocityEstimate;
    }
}
