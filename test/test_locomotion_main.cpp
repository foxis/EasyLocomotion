#if  defined(UNIT_TEST)

#define REAL_T float

#if defined(ARDUINO)
#include <Arduino.h>

namespace Locomotion{
template<typename T, size_t N, size_t M> class _Matrix;
template<typename T, size_t N> class _Vector;
}

template<typename T, size_t N, size_t M> void print_matrix(const Locomotion::_Matrix<T, N, M>& m, const char * name) {
    Serial.print("matrix ");
    Serial.println(name);
    Serial.println("    {");
    for (int row = 0; row < N; row++) {
        Serial.print("        {");
        for (int col=0;col<M;col++) {
            if (col != 0) 
                Serial.print(", ");
            Serial.print(m.val(row, col), 6);
        }
        if (row != N - 1)
            Serial.println("},");
        else
            Serial.println("}");
    }
    Serial.println("    }");
}

template<typename T, size_t N> void print_vector(const Locomotion::_Vector<T, N> & v, const char * name) {
    Serial.print("vector ");
    Serial.println(name);
    Serial.print("    {");
    for (int row = 0; row < N; row++) {
        if (row != 0) 
            Serial.print(", ");
        Serial.print(v.val(row), 6);
    }
    Serial.println("}");
}
template<typename T, size_t N> void print_arr(const T * v, const char * name) {
    Serial.print("Array ");
    Serial.println(name);
    Serial.print("    {");
    for (int row = 0; row < N; row++) {
        if (row != 0) 
            Serial.print(", ");
        Serial.print(v[row], 6);
    }
    Serial.println("}");
}

template<typename T> void print_val(const T a, const char * name) {
    Serial.print("value ");
    Serial.print(name);
    Serial.print(": ");
    Serial.println(a, 6);
}

void print_text(const char * text) {
    Serial.print("Text: ");
    Serial.println(text);
}

void print_checkpoint_(int line) {
    Serial.print("Checkpoint line: ");
    Serial.println(line);
}

void print_duration_(int line, unsigned long duration) {
    Serial.print("Duration line: ");
    Serial.print(line);
    Serial.print(" ");
    Serial.print(duration);
    Serial.println(" us");
}

#define PP_CAT(a, b) PP_CAT_I(a, b)
#define PP_CAT_I(a, b) PP_CAT_II(~, a ## b)
#define PP_CAT_II(p, res) res

#define print_checkpoint print_checkpoint_(__LINE__);
#define print_duration(expression) print_duration__(expression, PP_CAT(now, __LINE__))
#define print_duration__(expression, now) \
    unsigned long now = micros(); \
    expression; \
    print_duration_(__LINE__, micros() - now);

#else
#include <string.h>
#include <limits.h>
typedef unsigned char byte;

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#include <unity.h>

#include "math_utils.h"

using namespace Locomotion;


#include "math_utils/test_math_utils_main.hpp"
#include "kinematics/test_kinematics_main.hpp"

#define _RUN_TEST(a) \
    clean_stack(); \
    RUN_TEST(a);

void clean_stack() {
    byte arr[2048];
    memset(arr, 0xAA, sizeof(arr));
}

#if defined(ARDUINO)
void setup() {
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    UNITY_BEGIN();

    MATH_UTILS_TESTS;
    KINEMATICS_TESTS;

    UNITY_END();

}

void loop() {
}
#else

int main(int argc, char **argv) {
    UNITY_BEGIN();

    MATH_UTILS_TESTS;
    KINEMATICS_TESTS;

    UNITY_END();
}

#endif

#endif