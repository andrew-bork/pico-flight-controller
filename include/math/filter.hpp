#pragma once

namespace filter {
    struct filter {
        float a1=0, a2=0, b0=1, b1=0, b2=0;
        float x1=0, x2=0, y1=0, y2=0;
        float operator [](float inp);

        void copy_coeffs(filter& other);
    };
    
    filter low_pass(float samp_rate, float w0);
    filter band_pass(float samp_rate, float wL, float wU);
    filter high_pass(float samp_rate, float w0);
    filter none();
    // filter complimentary(float samp_rate);
};