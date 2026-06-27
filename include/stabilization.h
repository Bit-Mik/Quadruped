#pragma once

void stabilizationInit();

void updateBodyCompensation();

void stabilizationSetConfig(float kpRoll,
                            float kiRoll,
                            float kdRoll,
                            float kpPitch,
                            float kiPitch,
                            float kdPitch,
                            float rollDeadband,
                            float pitchDeadband,
                            float maxRollCorr,
                            float maxPitchCorr);

float getRollCompLeft();
float getRollCompRight();

float getPitchCompFront();
float getPitchCompRear();

void calibrateBodyPose();
