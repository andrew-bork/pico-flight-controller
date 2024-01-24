#include <math/filter.hpp>
#include <cmath>

#define sqrt2 1.41421356237 
float filter::filter::operator[] (float x0){
    float y0 = b0 * x0 + b1 * x1 + b2 * x2 + a1 * y1 + a2 * y2;
    x2 = x1;
    x1 = x0;
    y2 = y1;
    y1 = y0;
    return y0;
}

filter::filter filter::low_pass(float sample_rate, float w0) {
    // https://stackoverflow.com/questions/20924868/calculate-coefficients-of-2nd-order-butterworth-low-pass-filter
    filter out;
    float ff = w0 / sample_rate;
    const float ita =1.0/ tan(M_PI*ff);
    const float q=sqrt(2.0);
    out.b0 = 1.0 / (1.0 + q*ita + ita*ita);
    out.b1= 2*out.b0;
    out.b2= out.b0;
    out.a1 = 2.0 * (ita*ita - 1.0) * out.b0;
    out.a2 = -(1.0 - q*ita + ita*ita) * out.b0;
    return out;
}

filter::filter filter::band_pass(float sample_rate, float wL, float wH){
    // https://github.com/dimtass/DSP-Cpp-filters/blob/master/lib/so_bpf.h
    filter out;
    float fs = sample_rate;
    float fc = (wL + wH) / 2;
    float Q = 2 / ((wH - wL) * sample_rate);

    float w = 2.0 * M_PI * fc / fs;
    float b = 0.5*((1.0 - tan(w / (2.0*Q))) / (1.0 + tan(w / (2.0*Q))));
    float g = (0.5 + b)*cos(w);
    out.b0 = 0.5 - b;
    out.b1 = 0.0;
    out.b2 = -(0.5 - b);
    out.a1 = -2.0 * g;
    out.a2 = 2.0 * b;
    return out;
}


filter::filter filter::none(){
    filter out;
    out.b0 = 1;
    return out;
}

filter::filter filter::high_pass(float sample_rate, float w0){
    // https://github.com/dimtass/DSP-Cpp-filters/blob/master/lib/so_hpf.h
    // filter out;
    // float c = tan(M_PI*w0 / samp_rate);
    // out.b0 = 1.0 / (1.0 + sqrt2*c + pow(c, 2.0));
    // out.b1 = -2.0 * out.b0;
    // out.b2 = out.b0;
    // out.a1 = 2.0 * out.b0*(pow(c, 2.0) - 1.0);
    // out.a2 = out.b0 * (1.0 - sqrt2*c + pow(c, 2.0));
    // return out;
    // https://stackoverflow.com/questions/20924868/calculate-coefficients-of-2nd-order-butterworth-low-pass-filter
    filter out;
    float ff = w0 / sample_rate;
    const float ita =1.0/ tan(M_PI*ff);
    const float q=sqrt(2.0);
    out.b0 = 1.0 / (1.0 + q*ita + ita*ita);
    out.b1= 2*out.b0;
    out.b2= out.b0;
    out.a1 = 2.0 * (ita*ita - 1.0) * out.b0;
    out.a2 = -(1.0 - q*ita + ita*ita) * out.b0;

    out.b0 = out.b0*ita*ita;
    out.b1 = -out.b1*ita*ita;
    out.b2 = out.b2*ita*ita;

    return out;
}



void filter::filter::copy_coeffs(filter& other) {
    a1 = other.a1;
    a2 = other.a2;
    b0 = other.b0;
    b1 = other.b1;
    b2 = other.b2;
}