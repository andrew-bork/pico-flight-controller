#pragma once
namespace math{
    struct vector{
        float x,y,z;
        /**
         * @brief Construct a zero vector
         * 
         */
        vector();
        /**
         * @brief Construct a vector with an x component
         * 
         * @param x 
         */
        vector(float x);
        /**
         * @brief Construct a vector with an x and y component
         * 
         * @param x 
         * @param y 
         */
        vector(float x, float y);
        /**
         * @brief Construct a new vector with an x, y, and z component
         * 
         * @param x 
         * @param y 
         * @param z 
         */
        vector(float x, float y, float z);
        
        /**
         * @brief Vector addition
         * 
         * @param r 
         * @return vector 
         */
        vector operator+ (const vector& r) const;

        /**
         * @brief Vector scaling
         * 
         * @param s 
         * @return vector 
         */
        vector operator* (const float& s) const;

        /**
         * @brief Vector subtraction
         * 
         * @param r 
         * @return vector 
         */
        vector operator- (const vector& r) const;

        /**
         * @brief Dot product
         * 
         * @param r 
         * @return double 
         */
        float dot(const vector& r) const;
    };

    float length(const vector& n);
};