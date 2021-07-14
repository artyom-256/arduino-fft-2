
/**
 * Copyright (c) 2021 Artem Hlumov <artyom.altair@gmail.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Helper structure that represents a complex number.
 * As there is no coprocessor in Arduino we have to work with integer arithmetic.
 * Numbers are represented as a mantissa. Exponent is defined by SCALE value.
 * For example, if SCALE is 1024 and sizeof(short) is 16 we have 10 bites representing
 * values after the floating points and 6 bits - before.
 */
struct complex
{
    /**
     * Real part of the complex number.
     */
    short real;
    /**
     * Imagine part of the complex number.
     */
    short imag;
};

/**
 * Define how many bits are dedicated for the numbers after floating point.
 * So this is precision of our fake float numbers. The rest of bits are dedicated for
 * the integer part.
 * The scale MUST be a power of 2 in order to perform quick division via binary shift.
 */
#define SCALE 1024
/**
 * Binary logarithm of the SCALE variable.
 */
#define LOG_SCALE 10

/**
 * Make sure the value does not exceed limits for short.
 * If overflow happens we simply stick to the lower or upper border.
 * This is fine in case of small overflow and does not make our numbers
 * suddenly turn into negative. Assuming sizeof(short) is 2 bytes.
 * @param v Value to trim.
 * @return Value that fits short integer.
 */
#define TRIM(v) constrain((v), -32768, +32767)

/**
 * Add two complex numbers and return their sum.
 * Use long for intermediate calculations to prevent overflow.
 * @param a First number.
 * @param b Second number.
 * @return Sum of numbers.
 */
#define C_ADD(a, b) {        \
  TRIM(long((a).real) + long((b).real)), \
  TRIM(long((a).imag) + long((b).imag))  \
}
/**
 * Substract two complex numbers.
 * Use long for intermediate calculations to prevent overflow.
 * @param a First number.
 * @param b Second number.
 * @return Substruction of numbers.
 */
#define C_SUB(a, b) {        \
  TRIM(long((a).real) - long((b).real)), \
  TRIM(long((a).imag) - long((b).imag))  \
}
/**
 * Multiply two complex numbers.
 * Use long for intermediate calculations to prevent overflow.
 * If we multiply two floating values represented by mantissa, we should divide the product by SCALE
 * in order to get mantissa of the product. To do this faster we use binary shift.
 * @param a First number.
 * @param b Second number.
 * @return Product of tow numbers.
 */
#define C_MUL(a, b) {                                                                                    \
  TRIM((long((a).real) * long((b).real) >> LOG_SCALE) - (long((a).imag) * long((b).imag) >> LOG_SCALE)), \
  TRIM((long((a).real) * long((b).imag) >> LOG_SCALE) + (long((a).imag) * long((b).real) >> LOG_SCALE))  \
}

/**
 * Reverse bits of the given value assuming it is in range [0, SIZE).
 * The function is required by the FFT algorithm.
 * Here we use this trick https://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith32Bits
 * to reverse bits of a byte (no division, no 64-bit values) and after shift the outcome by 8 - LOG_SIZE
 * to get a result like we had only LOG_SIZE bits in byte.
 * @param value Value to bit-reverse.
 * @return Reversed bits value.
 */
#define REVERSE_BITS(value) (                                                                                                                 \
  (unsigned char)(                                                                                                                            \
    (((((unsigned char)value) * 0x0802LU & 0x22110LU) | (((unsigned char)value) * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16) >> (8 - LOG_SIZE) \
  )                                                                                                                                           \
)

/**
 * Define all SIZE complex roots of unity of power SIZE which are used in the FFT algorithm.
 * Complexity of the FFT depends on how we calculate roots of unity.
 * Direct calculation using exp(-2*PI*k*i/N) formula is too expensive and is not considered for Arduino.
 * We use predefined table of all roots used in the calculation, so computation complexity is decreased, but we need more memory.
 * Another way might be to keep only primitive roots of powers 2, 4, 8, 16, 32, 64, 128 and calculate others via complex multiplication.
 * This would reduce memory consumption, but increases computation complexity. Also there might be some hybid solution were we
 * have only a couple of these numbers in the table and calculate the rest on fly.
 */
complex roots_of_unity[] = {
  {1024, 0}, {1022, 50}, {1019, 100}, {1012, 150}, {1004, 199}, {993, 248}, {979, 297}, {964, 344}, {946, 391}, {925, 437}, {903, 482}, {878, 526},
  {851, 568}, {822, 609}, {791, 649}, {758, 687}, {724, 724}, {687, 758}, {649, 791}, {609, 822}, {568, 851}, {526, 878}, {482, 903}, {437, 925},
  {391, 946}, {344, 964}, {297, 979}, {248, 993}, {199, 1004}, {150, 1012}, {100, 1019}, {50, 1022}, {0, 1024}, {-50, 1022}, {-100, 1019}, {-150, 1012},
  {-199, 1004}, {-248, 993}, {-297, 979}, {-344, 964}, {-391, 946}, {-437, 925}, {-482, 903}, {-526, 878}, {-568, 851}, {-609, 822}, {-649, 791},
  {-687, 758}, {-724, 724}, {-758, 687}, {-791, 649}, {-822, 609}, {-851, 568}, {-878, 526}, {-903, 482}, {-925, 437}, {-946, 391}, {-964, 344},
  {-979, 297}, {-993, 248}, {-1004, 199}, {-1012, 150}, {-1019, 100}, {-1022, 50}, {-1024, 0}, {-1022, -50}, {-1019, -100}, {-1012, -150}, {-1004, -199},
  {-993, -248}, {-979, -297}, {-964, -344}, {-946, -391}, {-925, -437}, {-903, -482}, {-878, -526}, {-851, -568}, {-822, -609}, {-791, -649}, {-758, -687},
  {-724, -724}, {-687, -758}, {-649, -791}, {-609, -822}, {-568, -851}, {-526, -878}, {-482, -903}, {-437, -925}, {-391, -946}, {-344, -964}, {-297, -979},
  {-248, -993}, {-199, -1004}, {-150, -1012}, {-100, -1019}, {-50, -1022}, {0, -1024}, {50, -1022}, {100, -1019}, {150, -1012}, {199, -1004}, {248, -993},
  {297, -979}, {344, -964}, {391, -946}, {437, -925}, {482, -903}, {526, -878}, {568, -851}, {609, -822}, {649, -791}, {687, -758}, {724, -724}, {758, -687},
  {791, -649}, {822, -609}, {851, -568}, {878, -526}, {903, -482}, {925, -437}, {946, -391}, {964, -344}, {979, -297}, {993, -248}, {1004, -199},
  {1012, -150}, {1019, -100}, {1022, -50}
};
/**
 * Define pow2(n) values for all values in range [0, LOG_SIZE] to not calculate them on fly.
 */
const int pow2[] = {1, 2, 4, 8, 16, 32, 64, 128};

/**
 * Size of the FFT input vector.
 * In order to make the algorithm smaller we only focus on this value and adjust all data structures accordingly.
 * If we want to change the input vector size it would require recalcultion of roots_of_unity and pow2 arrays.
 */
#define SIZE 128
/**
 * Binary logarighm of SIZE, keep it here to prevent calculations on fly.
 */
#define LOG_SIZE 7

/**
 * Maximal frequency we want to detect (Hz).
 */
#define MAX_FREQUENCY 4000
/**
 * Delay for reading a value via analogRead() function.
 * Calculated empirically to align sampling.
 */
#define READ_DELAY 112

/**
 * Perform Fast Fourier Transformation of the samples vector and return a frequency having major amplitude.
 * Use Cooley–Tukey algorithm: https://en.wikipedia.org/wiki/Cooley%E2%80%93Tukey_FFT_algorithm
 * The transformation is performed in place, so the vector will represent frequencies at the end.
 * Although the input vector will contain result of transformation, it is not finished yet.
 * To get the output values, all elements of the array should be casted to float and divided by (SCALE * sqrt(SIZE)).
 * This is omitted because we do not need to go so far.
 * @param input Input array of exactly SIZE complex numbers to transform.
 * @param minFrequency Minimal frequency value to consider.
 * @param maxFrequency Maximal frequency value to consider.
 * @return Value of the major frequency in the given range.
 * Only frequencies in the given range are processed even if there are some bigger spike outside the range,
 * it won't be considered.
 */
short take_major_frequency(complex* input, short minFrequency, short maxFrequency) {
  // Step 1: Reorder numbers in reverse-bit order.

  for (int i = 0; i < SIZE; i++) {
    int index = REVERSE_BITS(i);
    // This prevents double-swapping problem.
    if (i < index) {
      // Swap two numbers if they are not in correct place.
      complex temp = input[index];
      input[index] = input[i];
      input[i] = temp;
    }
  }
  
  // Step 2: Do LOG_SIZE steps of butterfly transformation.

  for (int i = 1; i <= LOG_SIZE; i++) {
    // Stride value.
    int m = pow2[i];
    // For each chunk of size m.
    for (int j = 0; j < SIZE; j += m) {
      // Go through the first half of the chunk and apply butterfly with numbers of the second chunk.
      for (int k = 0; k < m / 2; k++) {
        // Butterfly step.
        int u_index = j + k + m / 2;
        int v_index = j + k;
        complex u = C_MUL(roots_of_unity[SIZE / m * k], input[u_index]);
        complex v = input[v_index];
        input[v_index] = C_ADD(v, u);
        input[u_index] = C_SUB(v, u);
      }
    }
  }

  // Step 3: Find the major frequency.

  // At this moment of time we have almost done the FFT algorithm. The only thing remains is
  // division by (SCALE * sqrt(SIZE)).
  // However since we are interested in finding the major frequency, division by a constant changes nothing
  // and it could be omitted.
  // Amplitude of the value corresponds to module of the complex number. But we do not need to calculate
  // the module, it would be enough to compare square of module and avoid computation of suqare root.
  // So just find which index corresponds to the biggest value of (real*real + imag*imag).
  // Use long as short * short can easily exceed short limits.
  long maxValue = -1;
  long maxIndex = -1;
  // Range of frequencies that we want to consider.
  int beginIndex = long(minFrequency) * SIZE / (2 * MAX_FREQUENCY);
  int endIndex = long(maxFrequency) * SIZE / (2 * MAX_FREQUENCY);
  for (int i = beginIndex; i < endIndex; i++) {
    long val = long(input[i].real) * input[i].real + long(input[i].imag) * input[i].imag;
    if (val > maxValue) {
      maxValue = val;
      maxIndex = i;
    }
  }

  // Step 4: Convert index into frequency.

  // Fourier transformation of pure real sequence of values is symmetrical about the center.
  // So only the first half keeps frequency information - the other half is just a reflection.
  // The half of the output array evenly corresponds to values from 0 to MAX_FREQUENCY.
  // So use this formula to find out particular frequency.
  // Note that accuracy here is 2 * MAX_FREQUENCY / SIZE.
  return MAX_FREQUENCY * 2 * maxIndex / SIZE;
}

/**
 * Delay in milliseconds when the LED should remain alight to keep smoother blinking.
 */
#define LED_DELAY 100
/**
 * As we use pretty limited range of values for computation and low precision, we need to
 * amplify a bit the input signal from the sound sensor in order to not bring it to 0 during compurations.
 * Resolution of analogRead() is [0, 1023].
 */
#define INPUT_AMPLIFIER 16
/**
 * Minimal value of the sound amplitude to be considered as noise.
 * Values below are skipped.
 */
#define SILENCE_VALUE 300
/**
 * Frequency (Hz) that corresponds to the first LED.
 */
#define FREQUENCY_RANGE_MIN 150
/**
 * Frequency (Hz) that corresponds to the last LED.
 */
#define FREQUENCY_RANGE_MAX 1500
/**
 * Total amount of LEDs.
 */
#define NUM_LEDS 10

/**
 * Sound sensor analogue data pin.
 */
unsigned char soundSensorPin = A0;
/**
 * Pins that correspond to the LEDs.
 */
unsigned char ledPins[NUM_LEDS] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
/**
 * Keep timestamps of last activation of each LED.
 * This allows us to keep track on how long the LED is alight.
 */
long ledTimestamps[NUM_LEDS] = {0};
/**
 * Delay in microseconds between two measurements in order to keep
 * desired frequency of sampling.
 */
int samplingDelay;

/**
 * Initialize Arduino.
 */
void setup() {
  // According to Nyquist–Shannon-Kotelnikov Theorem, in order to represent frequencies up to MAX_FREQUENCY,
  // we have to take samples with frequency 2 * MAX_FREQUENCY.
  // Calculate delay in microseconds basing on this frequency and taking into account that we have READ_DELAY
  // caused by Arduino. We do not use more precise calculations involving micros() function just to have as less code
  // as possible in the sampling loop.
  samplingDelay = 1000000 / (MAX_FREQUENCY * 2) - READ_DELAY;
  // If the frequency is too high, end up with no delay.
  if (samplingDelay < 0) {
    samplingDelay = 0;
  }

  // Set up output pins.
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
}

/**
 * Loop function.
 */
void loop() {
  // Array of samples.
  complex samples[SIZE];
  // Maximal amplitude value.
  // Use it to start FFT only when we reach some threshold, so prevent transforming silence.
  short maxValue = 0;
  // Take samples. The loop should be as simple as possible to preserve the same delay between read operations.
  for (int i = 0; i < SIZE; i++) {
    // Read amplitude value from the sound sensor.
    short value = analogRead(soundSensorPin);
    // Save the value to the input vector (amplitudes are always real values, so imagine part is 0).
    samples[i] = {value * INPUT_AMPLIFIER, 0};
    // Keep track on maximal value of the noise.
    maxValue = max(value, maxValue);
    // Delay to preserve constant sampling frequency.
    delayMicroseconds(samplingDelay);
  }
  // Do not perform FFT for silence.
  if (maxValue >= SILENCE_VALUE) {
    // Perform FFT and extract a major frequency.
    short f = take_major_frequency(samples, FREQUENCY_RANGE_MIN, FREQUENCY_RANGE_MAX);
    // Calculate the LED position that should light up.
    // Taking into account that FREQUENCY_RANGE_MIN corresponds to the first LED and FREQUENCY_RANGE_MAX - to the last one.
    int ledPos = (f - FREQUENCY_RANGE_MIN) * NUM_LEDS / (FREQUENCY_RANGE_MAX - FREQUENCY_RANGE_MIN);
    // Mark the LED that corresponds to the major frequency.
    ledTimestamps[ledPos] = millis();
  }
  // Light up all LEDs which have been activated in the last LED_DELAY milliseconds.
  for (int i = 0; i < NUM_LEDS; i++) {
    // We use pins 2 - 12 for LEDs.
    digitalWrite(ledPins[i], millis() - ledTimestamps[i] < LED_DELAY ? HIGH : LOW);
  }
}
