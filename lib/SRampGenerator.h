#ifndef SRAMPGENERATOR_H
#define SRAMPGENERATOR_H

#include <iostream>
#include <vector>
#include <cmath>

class SRampGenerator
{
private:
    float *commandV = NULL;
    float lastV = 0;
    uint16_t index = 0;
    uint16_t totalTimeFrames = 0;

public:
    SRampGenerator(/* args */);
    ~SRampGenerator();
    void generateVelocityProfile(const int16_t startV, const int16_t targetV, const uint16_t timeFrames);
    void generateVelocityProfile(const int16_t targetV, const uint16_t timeFrames);
    float getVf(void);
    int16_t getV(void);
    uint16_t getTotalTimeFrames(void);
};

#endif