//
// Created by ajahueym on 1/14/24.
//
#include "overture_filters/low_pass_filter/low_pass_filter.h"
#include <cmath>

LowPassFilter::LowPassFilter(double cutoffFrequency, double period) {
    this->cutoffFrequency = cutoffFrequency;
    this->period = period;

    beta = std::exp(-1.0 * cutoffFrequency * period);
}

double LowPassFilter::Update(double input) {
    double res = beta * lastRes + (1 - beta) * input;
    lastRes = res;
    return res;
}
