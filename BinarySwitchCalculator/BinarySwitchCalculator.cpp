#include <iostream>
#include <vector>
#include <cstddef>  // for std::size_t
#include <cmath>    // for std::pow
#include <thread>   // for std::thread
#include <mutex>    // for std::mutex
#include <chrono>   // for std::chrono
#include <Windows.h> // for OutputDebugString
#include <iomanip>  // for std::setprecision
#include <algorithm>  // for std::sort

struct ThreadIndexes
{
    std::size_t startIndex;
    std::size_t endIndex;
};

struct PhysicsParams
{
    std::vector<float> targetOutputs;
    float feedbackVoltage;
    bool use5PercentResistors;
    bool use1PercentResistors;
    bool use01PercentResistors;
    float allowableError;
    float minimumCurrent;
    float maximumCurrent;
};

struct Result
{
    float R;
    float R1;
    float R2;
    float R3;
    float R4;
    float currentState00;
    float currentState01;
    float currentState10;
    float currentState11;
    float voltageError00;
    float voltageError01;
    float voltageError10;
    float voltageError11;
};

// Thread-safe container for results
class ThreadSafeResults {
private:
    std::vector<Result> results;
    long long rollingResultsCount;
    std::mutex mutex;

public:
    void AppendResults(const std::vector<Result>& resultsToAppend)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if(resultsToAppend.size() == 0) {return;}
        results.resize(results.size() + resultsToAppend.size());
        memcpy(&results[results.size() - resultsToAppend.size()], &resultsToAppend[0], resultsToAppend.size() * sizeof(Result));
    }

    void TransferResults(std::vector<Result>& resultsToTransfer)
    {
        std::lock_guard<std::mutex> lock(mutex);
        resultsToTransfer = results;
        results.clear();
    }

    void UpdateRollingResultsCount(long long addThisManyResults)
    {
        std::lock_guard<std::mutex> lock(mutex);
        rollingResultsCount += addThisManyResults;
    }

    long long GetRollingResultsCount()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return rollingResultsCount;
    }

};

// Worker function that each thread will execute
void calculateResistorCombinations(
    PhysicsParams localPhysicsParams, 
    ThreadIndexes localThreadIndexes,
    const std::vector<float>& resistorValues,
    ThreadSafeResults& sharedResults
) {
    std::vector<Result> localResults;  // Store results locally first
    localResults.reserve(localThreadIndexes.endIndex - localThreadIndexes.startIndex + 1);
    Result tempResult{0, 0, 0, 0, 0, 0, 0, 0, 0};
    long long iterations = 0; // total iterations of resistor combinations

    /*
    The switches conduct current to 2 resistors at a time. The mapping of resistor pairs to outputs is as follows:
    R1 and R2 -> 00, targetOutputs[0]
    R1 and R4 -> 01, targetOutputs[1]
    R2 and R3 -> 10, targetOutputs[2]
    R2 and R4 -> 11, targetOutputs[3]
    00, 01, 10, 11 indicate distinct ON-ON states, but these don't inherently map to specific switch pins.
    Assign state 0 and 1 to each of the output pins of both switches to get the physical mapping you want.

    The algorithm was a huge number of iterations. The loops are:
    1. loop through R (the low side resistor) for this thread's block of resistor values
    2. Inside that loop, iterate through all values of R1
    3. Inside that loop, iterate through all values of R2
    4. Inside that loop, iterate through all values of R3
    5. Inside that loop, iterate through all values of R4
    6. Calculate the voltage error and current for each of the 4 states
    7. If the voltage error and current are acceptable, save it to the local results vector
    */

    for (std::size_t i = localThreadIndexes.startIndex; i <= localThreadIndexes.endIndex; i++)
    {
        // R loop
        for (std::size_t j = 0; j < resistorValues.size(); j++)
        {
            // R1 loop
            for (std::size_t k = 0; k < resistorValues.size(); k++)
            {
                // R2 loop
                for (std::size_t l = 0; l < resistorValues.size(); l++)
                {
                    // R3 loop
                    for (std::size_t m = 0; m < resistorValues.size(); m++)
                    {
                        // R4 loop

                        // iterations counter
                        iterations++;
                        if (iterations % static_cast<long long>(pow(10, 7)) == 0)
                        {
                            sharedResults.UpdateRollingResultsCount(static_cast<long long>(pow(10, 7)));
                        }

                        // assign resistor values
                        tempResult.R = resistorValues[i];
                        tempResult.R1 = resistorValues[j];
                        tempResult.R2 = resistorValues[k];
                        tempResult.R3 = resistorValues[l];
                        tempResult.R4 = resistorValues[m];

                        // calculate current and check if it's too high or too low
                        tempResult.currentState00 = localPhysicsParams.feedbackVoltage / (tempResult.R + 1 / (1 / tempResult.R1 + 1 / tempResult.R3));
                        if (tempResult.currentState00 < localPhysicsParams.minimumCurrent || tempResult.currentState00 > localPhysicsParams.maximumCurrent) {continue;}

                        tempResult.currentState01 = localPhysicsParams.feedbackVoltage / (tempResult.R + 1 / (1 / tempResult.R1 + 1 / tempResult.R4));
                        if (tempResult.currentState01 < localPhysicsParams.minimumCurrent || tempResult.currentState01 > localPhysicsParams.maximumCurrent) {continue;}

                        tempResult.currentState10 = localPhysicsParams.feedbackVoltage / (tempResult.R + 1 / (1 / tempResult.R2 + 1 / tempResult.R3));
                        if (tempResult.currentState10 < localPhysicsParams.minimumCurrent || tempResult.currentState10 > localPhysicsParams.maximumCurrent) {continue;}

                        tempResult.currentState11 = localPhysicsParams.feedbackVoltage / (tempResult.R + 1 / (1 / tempResult.R2 + 1 / tempResult.R4));
                        if (tempResult.currentState11 < localPhysicsParams.minimumCurrent || tempResult.currentState11 > localPhysicsParams.maximumCurrent) {continue;}

                        // I will use voltage error to initially hold the voltage itself
                        tempResult.voltageError00 = localPhysicsParams.feedbackVoltage * (1 + 1 / (tempResult.R * (1 / tempResult.R1 + 1 / tempResult.R3)));
                        tempResult.voltageError00 = abs(localPhysicsParams.targetOutputs[0] - tempResult.voltageError00)/localPhysicsParams.targetOutputs[0];
                        if (tempResult.voltageError00 > localPhysicsParams.allowableError) {continue;}

                        tempResult.voltageError01 = localPhysicsParams.feedbackVoltage * (1 + 1 / (tempResult.R * (1 / tempResult.R1 + 1 / tempResult.R4)));
                        tempResult.voltageError01 = abs(localPhysicsParams.targetOutputs[1] - tempResult.voltageError01)/localPhysicsParams.targetOutputs[1];
                        if (tempResult.voltageError01 > localPhysicsParams.allowableError) {continue;}

                        tempResult.voltageError10 = localPhysicsParams.feedbackVoltage * (1 + 1 / (tempResult.R * (1 / tempResult.R2 + 1 / tempResult.R3)));
                        tempResult.voltageError10 = abs(localPhysicsParams.targetOutputs[2] - tempResult.voltageError10)/localPhysicsParams.targetOutputs[2];
                        if (tempResult.voltageError10 > localPhysicsParams.allowableError) {continue;}

                        tempResult.voltageError11 = localPhysicsParams.feedbackVoltage * (1 + 1 / (tempResult.R * (1 / tempResult.R2 + 1 / tempResult.R4)));
                        tempResult.voltageError11 = abs(localPhysicsParams.targetOutputs[3] - tempResult.voltageError11)/localPhysicsParams.targetOutputs[3];
                        if (tempResult.voltageError11 > localPhysicsParams.allowableError) {continue;}

                        // result meets requirements, keep it
                        localResults.push_back(tempResult);
                        
                    }
                }
            }
        }

       
    }

    sharedResults.AppendResults(localResults);
}

void outputProgress(ThreadSafeResults& sharedResults, const bool& monitoring, const long long totalCombinations) 
{
    int periodMilliseconds = 5000;
    double periodMinutes = periodMilliseconds / 60000;
    long long previousCombinations = 0;
    double velocity = 1;
    double remainingTime = 0;
    double percentProgress = 0;
    bool firstTime = true;
    
    while (monitoring) 
    {
        if (firstTime)
        {
            firstTime = false;
            previousCombinations = sharedResults.GetRollingResultsCount();
            std::this_thread::sleep_for(std::chrono::milliseconds(periodMilliseconds));
        }
        else
        {
            long long currentCombinations = sharedResults.GetRollingResultsCount();
            velocity = static_cast<double>(currentCombinations - previousCombinations) / static_cast<double>(periodMilliseconds);
            remainingTime = ((static_cast<double>(totalCombinations) - static_cast<double>(currentCombinations)) / velocity) / 60000;
            percentProgress = static_cast<double>(currentCombinations) / static_cast<double>(totalCombinations) * 100;
            previousCombinations = currentCombinations;

            std::cout << "Count: " << currentCombinations << "\t" 
                    << std::fixed << std::setprecision(2) << percentProgress << "%\t"
                    << std::fixed << std::setprecision(0) << velocity << "/ms\t" 
                    << std::fixed << std::setprecision(1) << remainingTime << "min\n";

            std::this_thread::sleep_for(std::chrono::milliseconds(periodMilliseconds));
        }


        
    }
}

bool compareResultsByVoltageError(const Result& a, const Result& b) 
{
    // average the voltage errors
    float aAverage = (a.voltageError00 + a.voltageError01 + a.voltageError10 + a.voltageError11) / 4.0f;
    float bAverage = (b.voltageError00 + b.voltageError01 + b.voltageError10 + b.voltageError11) / 4.0f;
    return aAverage < bAverage;
}

int main()
{
    // controlling inputs
    PhysicsParams mainInputs;
    mainInputs.targetOutputs = {3.3f, 5.0f, 12.0f, 24.0f}; // output of the DC voltage converter
    mainInputs.feedbackVoltage = 2.5f; // this is the value required at the output of the resistor divider
    mainInputs.use5PercentResistors = true;
    mainInputs.use1PercentResistors = true;
    mainInputs.use01PercentResistors = true;
    mainInputs.allowableError = 0.18f; // specified as a fraction of targetOutputs
    mainInputs.minimumCurrent = 0.0000001f; // in amps
    mainInputs.maximumCurrent = 0.1f; // in amps

    std::size_t workerThreads = 24;

    // input checks
    if (workerThreads <= 0)
    {
        std::cout << "Worker threads must be greater than 0" << std::endl;
        return 1;
    }

    // standard 5% decade values (E24 series)
    std::vector<float> decade5Percent = {10.0f, 11.0f, 12.0f, 13.0f, 15.0f, 16.0f, 18.0f, 20.0f, 22.0f, 24.0f, 27.0f, 30.0f,
                                        33.0f, 36.0f, 39.0f, 43.0f, 47.0f, 51.0f, 56.0f, 62.0f, 68.0f, 75.0f, 82.0f, 91.0f};

    // standard 1% decade values (E96 series)
    std::vector<float> decade1Percent = {10.0f, 10.2f, 10.5f, 10.7f, 11.0f, 11.3f, 11.5f, 11.8f, 12.1f, 12.4f, 12.7f, 13.0f, 
                                        13.3f, 13.7f, 14.0f, 14.3f, 14.7f, 15.0f, 15.4f, 15.8f, 16.2f, 16.5f, 16.9f, 17.4f,
                                        17.8f, 18.2f, 18.7f, 19.1f, 19.6f, 20.0f, 20.5f, 21.0f, 21.5f, 22.1f, 22.6f, 23.2f,
                                        23.7f, 24.3f, 24.9f, 25.5f, 26.1f, 26.7f, 27.4f, 28.0f, 28.7f, 29.4f, 30.1f, 30.9f,
                                        31.6f, 32.4f, 33.2f, 34.0f, 34.8f, 35.7f, 36.5f, 37.4f, 38.3f, 39.2f, 40.2f, 41.2f,
                                        42.2f, 43.2f, 44.2f, 45.3f, 46.4f, 47.5f, 48.7f, 49.9f, 51.1f, 52.3f, 53.6f, 54.9f,
                                        56.2f, 57.6f, 59.0f, 60.4f, 61.9f, 63.4f, 64.9f, 66.5f, 68.1f, 69.8f, 71.5f, 73.2f,
                                        75.0f, 76.8f, 78.7f, 80.6f, 82.5f, 84.5f, 86.6f, 88.7f, 90.9f, 93.1f, 95.3f, 97.6f};

    // standard 0.1% decade values (E192 series)
    std::vector<float> decade01Percent = {10.0f, 10.1f, 10.2f, 10.4f, 10.5f, 10.6f, 10.8f, 10.9f, 11.0f, 11.1f, 11.3f, 11.4f,
                                        11.5f, 11.7f, 11.8f, 12.0f, 12.1f, 12.3f, 12.4f, 12.6f, 12.7f, 12.9f, 13.0f, 13.2f,
                                        13.3f, 13.5f, 13.7f, 13.8f, 14.0f, 14.2f, 14.3f, 14.5f, 14.7f, 14.9f, 15.0f, 15.2f,
                                        15.4f, 15.6f, 15.8f, 16.0f, 16.2f, 16.4f, 16.7f, 16.9f, 17.2f, 17.4f, 17.7f, 17.9f,
                                        18.2f, 18.4f, 18.7f, 19.0f, 19.3f, 19.6f, 19.9f, 20.2f, 20.5f, 20.8f, 21.1f, 21.5f, 
                                        21.8f, 22.2f, 22.5f, 22.9f, 23.3f, 23.7f, 24.1f, 24.6f, 25.0f, 25.5f, 26.0f, 26.5f,
                                        27.0f, 27.5f, 28.0f, 28.5f, 29.1f, 29.6f, 30.2f, 30.8f, 31.4f, 32.0f, 32.7f, 33.4f,
                                        34.1f, 34.8f, 35.5f, 36.2f, 37.0f, 37.7f, 38.5f, 39.2f, 40.0f, 40.8f, 41.6f, 42.4f,
                                        43.2f, 44.0f, 44.9f, 45.7f, 46.6f, 47.5f, 48.4f, 49.3f, 50.2f, 51.1f, 52.0f, 53.0f,
                                        54.0f, 55.0f, 56.0f, 57.0f, 58.0f, 59.0f, 60.0f, 61.0f, 62.0f, 63.0f, 64.0f, 65.0f,
                                        66.5f, 68.1f, 69.8f, 71.5f, 73.2f, 75.0f, 76.8f, 78.7f, 80.6f, 82.5f, 84.5f, 86.6f,
                                        88.7f, 90.9f, 93.1f, 95.3f, 97.6f};

    // generating precalculated resistor values that will be referenced later
    std::vector<float> resistorValues;

    if (mainInputs.use5PercentResistors)
    {
        for (std::size_t i = 4; i <= 4; i++)
        {
            for (std::size_t j = 0; j < decade5Percent.size(); j++)
            {
                resistorValues.push_back(decade5Percent[j] * std::pow(10.0f, static_cast<float>(i - 1)));
            }
        }
    }

    if (mainInputs.use1PercentResistors)
    {
        for (std::size_t i = 3; i <= 3; i++)
        {
            for (std::size_t j = 0; j < decade1Percent.size(); j++)
            {
                resistorValues.push_back(decade1Percent[j] * std::pow(10.0f, static_cast<float>(i - 1)));
            }
        }
    }

    if (mainInputs.use01PercentResistors)
    {
        for (std::size_t i = 2; i <= 2; i++)
        {
            for (std::size_t j = 0; j < decade01Percent.size(); j++)
            {
                resistorValues.push_back(decade01Percent[j] * std::pow(10.0f, static_cast<float>(i - 1)));
            }
        }
    }
    
    // calculate the total number of combinations for reporting purposes
    long long totalCombinations = static_cast<long long>(pow(resistorValues.size(), 5));
    
    // work is divided between threads according with which selection of resistor values will be attached to bit index 0
    // generate the indexes that each thread will use to determine the bit index 0 resistor
    std::vector<ThreadIndexes> threadIndexes;
    threadIndexes.reserve(workerThreads);

    // note: I could just use a float division and add 1 to a single thread, but this functions well enough for now
    std::vector<float>::size_type blockSize = resistorValues.size() / workerThreads;
    std::vector<float>::size_type remainder = resistorValues.size() % workerThreads;
    blockSize += remainder / workerThreads;
    remainder = remainder % workerThreads;

    for (std::size_t i = 0; i < workerThreads; i++)
    {
        threadIndexes.push_back(ThreadIndexes{0, 0});
        threadIndexes[i].startIndex = i * blockSize;
        if (i > 0)
        {
            threadIndexes[i].startIndex += 1;
        }
        threadIndexes[i].endIndex = (i + 1) * blockSize;
    }
    threadIndexes[threadIndexes.size() - 1].endIndex += (remainder - 1);

    // Output thread index assignments
    std::cout << "Thread assignments:\n";
    for (std::size_t i = 0; i < threadIndexes.size(); i++) {
        std::cout << "Thread " << i << ": Start index = " << threadIndexes[i].startIndex 
                  << ", End index = " << threadIndexes[i].endIndex << "\n";
    }
    std::cout << "\nPress Enter to continue...\n";
    std::cin.get();

    // Create shared results container
    ThreadSafeResults sharedResults;

    // Create and launch threads
    std::vector<std::thread> threads;
    threads.reserve(workerThreads);

    const long long startTimeMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

    // Create progress monitoring thread
    bool monitoring = true;
    std::thread progress_thread(
        outputProgress,
        std::ref(sharedResults),
        std::ref(monitoring),
        totalCombinations
    );

    for (std::size_t i = 0; i < workerThreads; ++i) {
        threads.emplace_back(
            calculateResistorCombinations, 
            mainInputs, 
            threadIndexes[i],
            std::ref(resistorValues),
            std::ref(sharedResults)
        );
    }

    // Wait for all threads to complete
    for (auto& thread : threads) {
        thread.join();
    }

    const long long endTimeMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    const double durationSeconds = (endTimeMilliseconds - startTimeMilliseconds) / 1000.0;

    // Stop the monitoring thread and wait for it to finish
    monitoring = false;
    progress_thread.join();

    std::cout << "Total combinations processed: " << totalCombinations << "\n";
    std::cout << "Total time taken: " << std::fixed << std::setprecision(3) << durationSeconds << " seconds\n";

    std::vector<Result> finalResults;
    sharedResults.TransferResults(finalResults);

    std::cout << "Results found: " << finalResults.size() << "\n";
    
    // sort the results by voltage error
    std::sort(finalResults.begin(), finalResults.end(), compareResultsByVoltageError);

    std::cout << "\nPress Enter to view first 100 results, or Ctrl+C to exit";
    std::cin.get();
    
    // Print header
    std::cout << "R\tR1\tR2\tR3\tR4\t"
              << "Current_00(uA)\tCurrent_01(uA)\tCurrent_10(uA)\tCurrent_11(uA)\t"
              << "VoltageError_00(%)\tVoltageError_01(%)\tVoltageError_10(%)\tVoltageError_11(%)\n";
    
    // Print first 100 results
    for (size_t i = 0; i < finalResults.size(); i++) {
        if (i == 100) {break;}
        const Result& result = finalResults[i];
        std::cout << result.R << "\t"
                 << result.R1 << "\t"
                 << result.R2 << "\t"
                 << result.R3 << "\t"
                 << result.R4 << "\t"
                 << result.currentState00 * 1000000.0f << "\t"
                 << result.currentState01 * 1000000.0f << "\t"
                 << result.currentState10 * 1000000.0f << "\t"
                 << result.currentState11 * 1000000.0f << "\t"
                 << result.voltageError00 * 100 << "\t"
                 << result.voltageError01 * 100 << "\t"
                 << result.voltageError10 * 100 << "\t"
                 << result.voltageError11 * 100 << "\n";
    }

    std::cin.get();

    return 0;
}
