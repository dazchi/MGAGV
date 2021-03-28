#include "SRampGenerator.h"

SRampGenerator::SRampGenerator(/* args */)
{
    commandV = new float[1];
}

SRampGenerator::~SRampGenerator()
{
    delete[] commandV;
}

void SRampGenerator::generateVelocityProfile(const int16_t startV, const int16_t targetV, const uint16_t timeFrames)
{
    float absJ;
    float *A, *J;

    if ((timeFrames % 3) != 0)
    {
        totalTimeFrames = timeFrames - (timeFrames % 3);
    }
    else
    {
        totalTimeFrames = timeFrames;
    }

    absJ = (9.0f / 2.0f) * (targetV - startV) / powf(totalTimeFrames, 2);
    delete[] commandV;
    commandV = new float[totalTimeFrames + 1];

    A = new float[totalTimeFrames];
    J = new float[totalTimeFrames];

    for (size_t i = 0; i < totalTimeFrames / 3; i++)
    {
        J[i] = absJ;
    }
    for (size_t i = (totalTimeFrames / 3); i < 2 * totalTimeFrames / 3; i++)
    {
        J[i] = 0;
    }
    for (size_t i = (2 * totalTimeFrames / 3); i < totalTimeFrames; i++)
    {
        J[i] = -absJ;
    }

    commandV[0] = startV;
    A[0] = 0.0f;
    for (size_t i = 1; i < totalTimeFrames; i++)
    {
        A[i] = A[i - 1] + J[i - 1];
        commandV[i] = commandV[i - 1] + A[i - 1] + 0.5f * J[i - 1];
    }
    commandV[totalTimeFrames] = targetV;

    // printf("T\tV\t\tA\t\tJ\t\t\n");
    // for (size_t i = 0; i < totalTimeFrames + 1; i++)
    // {
    //     printf("%d\t%f\t%f\t%f\n", i, (*commandV)[i], A[i], J[i]);
    // }
    delete[] A;
    delete[] J;
    index = 0;
}

void SRampGenerator::generateVelocityProfile(const int16_t targetV, const uint16_t timeFrames)
{
   float absJ;
    float *A, *J;

    if ((timeFrames % 3) != 0)
    {
        totalTimeFrames = timeFrames - (timeFrames % 3);
    }
    else
    {
        totalTimeFrames = timeFrames;
    }

    absJ = (9.0f / 2.0f) * (targetV - lastV) / powf(totalTimeFrames, 2);
    delete[] commandV;
    commandV = new float[totalTimeFrames + 1];

    A = new float[totalTimeFrames];
    J = new float[totalTimeFrames];

    for (size_t i = 0; i < totalTimeFrames / 3; i++)
    {
        J[i] = absJ;
    }
    for (size_t i = (totalTimeFrames / 3); i < 2 * totalTimeFrames / 3; i++)
    {
        J[i] = 0;
    }
    for (size_t i = (2 * totalTimeFrames / 3); i < totalTimeFrames; i++)
    {
        J[i] = -absJ;
    }

    commandV[0] = lastV;
    A[0] = 0.0f;
    for (size_t i = 1; i < totalTimeFrames; i++)
    {
        A[i] = A[i - 1] + J[i - 1];
        commandV[i] = commandV[i - 1] + A[i - 1] + 0.5f * J[i - 1];
    }
    commandV[totalTimeFrames] = targetV;

    // printf("T\tV\t\tA\t\tJ\t\t\n");
    // for (size_t i = 0; i < totalTimeFrames + 1; i++)
    // {
    //     printf("%d\t%f\t%f\t%f\n", i, (*commandV)[i], A[i], J[i]);
    // }
    delete[] A;
    delete[] J;
    index = 0;
}

float SRampGenerator::getVf(void)
{
    if (index <= totalTimeFrames)
    {
        lastV = commandV[index++];
        return lastV;
    }
    return commandV[totalTimeFrames];
}

int16_t SRampGenerator::getV(void)
{
    if (index <= totalTimeFrames)
    {
        lastV = commandV[index++];
        return (int16_t)lastV;
    }
    return (int16_t)commandV[totalTimeFrames];
}

uint16_t SRampGenerator::getTotalTimeFrames(void)
{
    return totalTimeFrames;
}