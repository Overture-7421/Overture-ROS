//
// Created by ajahueym on 1/14/24.
//

#ifndef SRC_LOW_PASS_FILTER_H
#define SRC_LOW_PASS_FILTER_H
class LowPassFilter {
public:
    explicit LowPassFilter(double cutoffFrequency, double period);
    double Update(double input);
private:
    double cutoffFrequency = 0;
    double period = 0;
    double beta = 0;
    double lastRes = 0;
};
#endif //SRC_LOW_PASS_FILTER_H
