/* ESP32 + LSM6DSOX + Magic Wand
 *  - Gyro-based orientation
 *  - Time-corrected integration
 *  - Simple 2D projection (orientation_x, orientation_y)
 *  - Debug for IsMoving() and stroke ranges
 */

#include <ArduinoBLE.h>
#include <Chirale_TensorFlowLite.h>

#include "tensorflow/lite/micro/compatibility.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"

#include "magic_wand_model_data.h"
#include "rasterize_stroke.h"

#include "I2C_Abstract.hpp"
#include "IMU.hpp"

#define BLE_SENSE_UUID(val) ("4798e0f2-" val "-4d68-af64-8a8f5258404e")

namespace {

const int VERSION = 0x00000000;

constexpr int stroke_transmit_stride     = 2;
constexpr int stroke_transmit_max_length = 160;
constexpr int stroke_max_length          = stroke_transmit_max_length * stroke_transmit_stride;
constexpr int stroke_points_byte_count   = 2 * sizeof(int8_t) * stroke_transmit_max_length;
constexpr int stroke_struct_byte_count   = (2 * sizeof(int32_t)) + stroke_points_byte_count;
constexpr int moving_sample_count        = 50;

constexpr int raster_width    = 32;
constexpr int raster_height   = 32;
constexpr int raster_channels = 3;
constexpr int raster_byte_count = raster_height * raster_width * raster_channels;
int8_t raster_buffer[raster_byte_count];
float  stroke_accel_norm[stroke_transmit_max_length] = {};

BLEService       service(BLE_SENSE_UUID("0000"));
BLECharacteristic strokeCharacteristic(BLE_SENSE_UUID("300a"),
                                       BLERead,
                                       stroke_struct_byte_count);

// BLE name
String name;

// Accel buffer (most recent 600 accel triples)
constexpr int acceleration_data_length = 600 * 3;
float acceleration_data[acceleration_data_length] = {};
int   acceleration_data_index = 0;

// Gyro + orientation buffer (most recent 600 gyro/orientation triples)
constexpr int gyroscope_data_length = 600 * 3;
float gyroscope_data[gyroscope_data_length]   = {};
float orientation_data[gyroscope_data_length] = {};
int   gyroscope_data_index = 0;

// Nominal sample rate + timing state
float    gyroscope_sample_rate = 104.0f; // fallback only
uint32_t last_gyro_time_ms     = 0;

float current_velocity[3]        = {0.0f, 0.0f, 0.0f};
float current_position[3]        = {0.0f, 0.0f, 0.0f};
float current_gravity[3]         = {0.0f, 0.0f, 0.0f};
float current_gyroscope_drift[3] = {0.0f, 0.0f, 0.0f};

int32_t stroke_length = 0;
uint8_t stroke_struct_buffer[stroke_struct_byte_count] = {};
int32_t* stroke_state           = reinterpret_cast<int32_t*>(stroke_struct_buffer);
int32_t* stroke_transmit_length = reinterpret_cast<int32_t*>(stroke_struct_buffer + sizeof(int32_t));
int8_t*  stroke_points          = reinterpret_cast<int8_t*>(stroke_struct_buffer + (sizeof(int32_t) * 2));

enum {
  eWaiting = 0,
  eDrawing = 1,
  eDone    = 2,
};

// TFLM
constexpr int kTensorArenaSize = 30 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

const tflite::Model*      model       = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;

constexpr int label_count = 10;
const char* labels[label_count] = { "0","1","2","3","4","5","6","7","8","9" };

// ---------------------------------------------------------------------------
// SENSOR READING
// ---------------------------------------------------------------------------

int ReadAccelerometerAndGyroscope(int maxSamples,
                                  int* new_accelerometer_samples,
                                  int* new_gyroscope_samples) {
  *new_accelerometer_samples = 0;
  *new_gyroscope_samples     = 0;

  uint8_t     numFifoSamples = 0;
  fifoSample_t rawFifoSample;
  fifoSample_t fifoSamples[3];

  for (int i = 0; i < maxSamples; i++) {
    uint16_t fifoSize = getFIFOSize();
    if (fifoSize == 0) break;

    // Read one raw 7-byte FIFO word (tag + payload)
    readFIFONoDecode(&rawFifoSample, 1);

    if (isGyroData(&rawFifoSample)) {
      numFifoSamples = decodeFifoWord(&rawFifoSample, fifoSamples, 3);
      for (int k = 0; k < numFifoSamples; k++) {
        int gyro_index = (gyroscope_data_index % gyroscope_data_length);
        gyroscope_data_index += 3;
        float* current_gyro = &gyroscope_data[gyro_index];
        processSample(fifoSamples[k], current_gyro);
        (*new_gyroscope_samples)++;
      }
    } else if (isAccelData(&rawFifoSample)) {
      numFifoSamples = decodeFifoWord(&rawFifoSample, fifoSamples, 3);
      for (int k = 0; k < numFifoSamples; k++) {
        int accel_index = (acceleration_data_index % acceleration_data_length);
        acceleration_data_index += 3;
        float* current_accel = &acceleration_data[accel_index];
        processSample(fifoSamples[k], current_accel);
        (*new_accelerometer_samples)++;
      }
    } else {
      // other tags ignored
    }
  }

  return 1;
}

// ---------------------------------------------------------------------------
// MATH HELPERS
// ---------------------------------------------------------------------------

float VectorMagnitude(const float* vec) {
  const float x = vec[0];
  const float y = vec[1];
  const float z = vec[2];
  return sqrtf((x * x) + (y * y) + (z * z));
}

void NormalizeVector(const float* in_vec, float* out_vec) {
  const float magnitude = VectorMagnitude(in_vec);
  const float x = in_vec[0];
  const float y = in_vec[1];
  const float z = in_vec[2];
  out_vec[0] = x / magnitude;
  out_vec[1] = y / magnitude;
  out_vec[2] = z / magnitude;
}

float DotProduct(const float* a, const float* b) {
  return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}

void EstimateGravityDirection(float* gravity) {
  int samples_to_average = 100;
  if (samples_to_average >= acceleration_data_index) {
    samples_to_average = acceleration_data_index;
  }

  const int start_index =
      ((acceleration_data_index +
        (acceleration_data_length - (3 * (samples_to_average + 1))))
       % acceleration_data_length);

  float x_total = 0.0f;
  float y_total = 0.0f;
  float z_total = 0.0f;
  for (int i = 0; i < samples_to_average; ++i) {
    const int index = ((start_index + (i * 3)) % acceleration_data_length);
    const float* entry = &acceleration_data[index];
    x_total += entry[0];
    y_total += entry[1];
    z_total += entry[2];
  }
  gravity[0] = x_total / samples_to_average;
  gravity[1] = y_total / samples_to_average;
  gravity[2] = z_total / samples_to_average;
}

void UpdateVelocity(int new_samples, float* gravity) {
  const float gx = gravity[0];
  const float gy = gravity[1];
  const float gz = gravity[2];

  const int start_index =
      ((acceleration_data_index +
        (acceleration_data_length - (3 * (new_samples + 1))))
       % acceleration_data_length);

  const float friction_fudge = 0.98f;

  for (int i = 0; i < new_samples; ++i) {
    const int index = ((start_index + (i * 3)) % acceleration_data_length);
    const float* entry = &acceleration_data[index];
    const float ax = entry[0];
    const float ay = entry[1];
    const float az = entry[2];

    const float ax_minus_g = ax - gx;
    const float ay_minus_g = ay - gy;
    const float az_minus_g = az - gz;

    current_velocity[0] += ax_minus_g;
    current_velocity[1] += ay_minus_g;
    current_velocity[2] += az_minus_g;

    current_velocity[0] *= friction_fudge;
    current_velocity[1] *= friction_fudge;
    current_velocity[2] *= friction_fudge;

    current_position[0] += current_velocity[0];
    current_position[1] += current_velocity[1];
    current_position[2] += current_velocity[2];
  }
}

void EstimateGyroscopeDrift(float* drift) {
  const bool isMoving = VectorMagnitude(current_velocity) > 0.1f;
  if (isMoving) {
    return;
  }

  int samples_to_average = 20;
  if (samples_to_average >= gyroscope_data_index) {
    samples_to_average = gyroscope_data_index;
  }

  const int start_index =
      ((gyroscope_data_index +
        (gyroscope_data_length - (3 * (samples_to_average + 1))))
       % gyroscope_data_length);

  float x_total = 0.0f;
  float y_total = 0.0f;
  float z_total = 0.0f;
  for (int i = 0; i < samples_to_average; ++i) {
    const int index = ((start_index + (i * 3)) % gyroscope_data_length);
    const float* entry = &gyroscope_data[index];
    x_total += entry[0];
    y_total += entry[1];
    z_total += entry[2];
  }
  drift[0] = x_total / samples_to_average;
  drift[1] = y_total / samples_to_average;
  drift[2] = z_total / samples_to_average;
}

// NEW: use dt_per_sample_sec instead of fixed sample rate
void UpdateOrientation(int new_samples,
                       float* /*gravity*/,
                       float* drift,
                       float dt_per_sample_sec) {
  const float drift_x = drift[0];
  const float drift_y = drift[1];
  const float drift_z = drift[2];

  const int start_index =
      ((gyroscope_data_index +
        (gyroscope_data_length - (3 * new_samples)))
       % gyroscope_data_length);

  const float dt = dt_per_sample_sec;

  for (int i = 0; i < new_samples; ++i) {
    const int index = ((start_index + (i * 3)) % gyroscope_data_length);
    const float* entry = &gyroscope_data[index];
    const float dx = entry[0];
    const float dy = entry[1];
    const float dz = entry[2];

    const float dx_minus_drift = dx - drift_x;
    const float dy_minus_drift = dy - drift_y;
    const float dz_minus_drift = dz - drift_z;

    const float dx_delta = dx_minus_drift * dt; // deg
    const float dy_delta = dy_minus_drift * dt;
    const float dz_delta = dz_minus_drift * dt;

    float* current_orientation = &orientation_data[index];
    const int previous_index =
        (index + (gyroscope_data_length - 3)) % gyroscope_data_length;
    const float* previous_orientation = &orientation_data[previous_index];

    current_orientation[0] = previous_orientation[0] + dx_delta;
    current_orientation[1] = previous_orientation[1] + dy_delta;
    current_orientation[2] = previous_orientation[2] + dz_delta;
  }
}

// ---------------------------------------------------------------------------
// IsMoving() with debug
// ---------------------------------------------------------------------------

bool IsMoving(int samples_before) {
  constexpr float moving_threshold = 5.0f;  // tuned from your logs

  if ((gyroscope_data_index - samples_before) < moving_sample_count) {
    return false;
  }

  const int start_index =
      ((gyroscope_data_index +
        (gyroscope_data_length - (3 * (moving_sample_count + samples_before))))
       % gyroscope_data_length);

  float total = 0.0f;
  for (int i = 0; i < moving_sample_count; ++i) {
    const int index = ((start_index + (i * 3)) % gyroscope_data_length);
    float* current_orientation = &orientation_data[index];
    const int previous_index =
        (index + (gyroscope_data_length - 3)) % gyroscope_data_length;
    const float* previous_orientation = &orientation_data[previous_index];
    const float dx = current_orientation[0] - previous_orientation[0];
    const float dy = current_orientation[1] - previous_orientation[1];
    const float dz = current_orientation[2] - previous_orientation[2];
    const float mag_squared = (dx * dx) + (dy * dy) + (dz * dz);
    total += mag_squared;
  }

  // DEBUG: print occasionally
  static uint32_t last_print = 0;
  uint32_t now = millis();
  if (now - last_print > 100) {
    Serial.printf("IsMoving total=%.3f (threshold=%.3f)\n",
                  total, moving_threshold);
    last_print = now;
  }

  const bool is_moving = (total > moving_threshold);
  return is_moving;
}

// ---------------------------------------------------------------------------
// Stroke update with SIMPLE 2D PROJECTION + debug
// ---------------------------------------------------------------------------

void UpdateStroke(int new_samples, bool* done_just_triggered) {
  constexpr int   minimum_stroke_length = 60;    // tuned with your len ~100–130
  constexpr float minimum_stroke_size   = 0.05f; // reject only tiny jitters

  *done_just_triggered = false;

  for (int i = 0; i < new_samples; ++i) {
    const int current_head = (new_samples - (i + 1));
    const bool is_moving   = IsMoving(current_head);
    const int32_t old_state = *stroke_state;

    if ((old_state == eWaiting) || (old_state == eDone)) {
      if (is_moving) {
        stroke_length = moving_sample_count;
        *stroke_state = eDrawing;
      }
    } else if (old_state == eDrawing) {
      if (!is_moving) {
        if (stroke_length > minimum_stroke_length) {
          *stroke_state = eDone;
        } else {
          stroke_length = 0;
          *stroke_state = eWaiting;
        }
      }
    }

    const bool is_waiting = (*stroke_state == eWaiting);
    if (is_waiting) {
      continue;
    }

    stroke_length += 1;
    if (stroke_length > stroke_max_length) {
      stroke_length = stroke_max_length;
    }

    const bool draw_last_point =
        ((i == (new_samples - 1)) && (*stroke_state == eDrawing));
    *done_just_triggered = ((old_state != eDone) && (*stroke_state == eDone));
    if (!(*done_just_triggered || draw_last_point)) {
      continue;
    }

    const int start_index =
        ((gyroscope_data_index +
          (gyroscope_data_length - (3 * (stroke_length + current_head))))
         % gyroscope_data_length);

    float x_total = 0.0f;
    float y_total = 0.0f;
    float z_total = 0.0f;
    for (int j = 0; j < stroke_length; ++j) {
      const int index = ((start_index + (j * 3)) % gyroscope_data_length);
      const float* entry = &orientation_data[index];
      x_total += entry[0];
      y_total += entry[1];
      z_total += entry[2];
    }

    const float x_mean = x_total / stroke_length;
    const float y_mean = y_total / stroke_length;
    const float z_mean = z_total / stroke_length;
    (void)z_mean;

    // zoom in based on your ranges
    constexpr float range = 30.0f; // degrees

    *stroke_transmit_length = stroke_length / stroke_transmit_stride;

    constexpr float accel_low  = 0.0f;
    constexpr float accel_high = 16000.0f;  // mg at 16g full-scale

    float x_min = 0.0f, y_min = 0.0f, x_max = 0.0f, y_max = 0.0f;

    for (int j = 0; j < *stroke_transmit_length; ++j) {
      const int orientation_index =
          ((start_index + ((j * stroke_transmit_stride) * 3))
           % gyroscope_data_length);
      const float* orientation_entry = &orientation_data[orientation_index];

      const float orientation_x = orientation_entry[0];
      const float orientation_y = orientation_entry[1];
      const float orientation_z = orientation_entry[2];
      (void)orientation_z;

      // SIMPLE 2D PROJECTION: directly from orientation_x / orientation_y
      const float x_axis = (orientation_x - x_mean) / range;
      const float y_axis = (orientation_y - y_mean) / range;

      const int stroke_index = j * 2;
      int8_t* stroke_entry   = &stroke_points[stroke_index];

      int32_t unchecked_x = static_cast<int32_t>(roundf(x_axis * 128.0f));
      int8_t stored_x;
      if (unchecked_x > 127) stored_x = 127;
      else if (unchecked_x < -128) stored_x = -128;
      else stored_x = (int8_t)unchecked_x;
      stroke_entry[0] = stored_x;

      int32_t unchecked_y = static_cast<int32_t>(roundf(y_axis * 128.0f));
      int8_t stored_y;
      if (unchecked_y > 127) stored_y = 127;
      else if (unchecked_y < -128) stored_y = -128;
      else stored_y = (int8_t)unchecked_y;
      stroke_entry[1] = stored_y;

      const bool is_first = (j == 0);
      if (is_first || (x_axis < x_min)) x_min = x_axis;
      if (is_first || (y_axis < y_min)) y_min = y_axis;
      if (is_first || (x_axis > x_max)) x_max = x_axis;
      if (is_first || (y_axis > y_max)) y_max = y_axis;

      const int accel_start_index =
          ((acceleration_data_index +
            (acceleration_data_length - (3 * (stroke_length + current_head))))
           % acceleration_data_length);
      const int accel_index =
          (accel_start_index + ((j * stroke_transmit_stride) * 3))
          % acceleration_data_length;

      const float* accel_entry = &acceleration_data[accel_index];
      const float ax = accel_entry[0];
      const float ay = accel_entry[1];
      const float az = accel_entry[2];

      float mag = sqrtf(ax * ax + ay * ay + az * az); // mg
      float norm = (mag - accel_low) / (accel_high - accel_low);
      if (norm < 0.0f) norm = 0.0f;
      if (norm > 1.0f) norm = 1.0f;

      stroke_accel_norm[j] = norm;
    }

    if (*done_just_triggered) {
      const float x_range = x_max - x_min;
      const float y_range = y_max - y_min;

      Serial.printf("Stroke DONE len=%ld, tx_len=%ld, x_range=%.3f, y_range=%.3f\n",
                    stroke_length, *stroke_transmit_length,
                    x_range, y_range);

      if ((x_range < minimum_stroke_size) && (y_range < minimum_stroke_size)) {
        Serial.println("Stroke rejected: too small.");
        *done_just_triggered     = false;
        *stroke_state            = eWaiting;
        *stroke_transmit_length  = 0;
        stroke_length            = 0;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// TFLITE SETUP
// ---------------------------------------------------------------------------

void setup_tflite() {
  model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model version mismatch");
    return;
  }

  static tflite::MicroMutableOpResolver<4> micro_op_resolver;
  micro_op_resolver.AddConv2D();
  micro_op_resolver.AddMean();
  micro_op_resolver.AddFullyConnected();
  micro_op_resolver.AddSoftmax();

  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors failed");
    return;
  }

  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size   != 4) ||
      (model_input->dims->data[0]!= 1) ||
      (model_input->dims->data[1]!= raster_height) ||
      (model_input->dims->data[2]!= raster_width) ||
      (model_input->dims->data[3]!= raster_channels) ||
      (model_input->type         != kTfLiteInt8)) {
    Serial.println("Bad input tensor params");
    return;
  }

  TfLiteTensor* model_output = interpreter->output(0);
  if ((model_output->dims->size   != 2) ||
      (model_output->dims->data[0]!= 1) ||
      (model_output->dims->data[1]!= label_count) ||
      (model_output->type         != kTfLiteInt8)) {
    Serial.println("Bad output tensor params");
    return;
  }
}

} // namespace

// ---------------------------------------------------------------------------
// ARDUINO SETUP / LOOP
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32 Magic Wand (gyro + debug) starting...");

  if (!startIMU(104u)) {
    Serial.println("startIMU FAILED");
    while (1) { delay(1000); }
  }

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1) { delay(1000); }
  }

  String address = BLE.address();
  Serial.print("address = ");
  Serial.println(address);
  address.toUpperCase();

  name  = "BLESense-";
  name += address[address.length() - 5];
  name += address[address.length() - 4];
  name += address[address.length() - 2];
  name += address[address.length() - 1];

  Serial.print("name = ");
  Serial.println(name);

  BLE.setLocalName(name.c_str());
  BLE.setDeviceName(name.c_str());
  BLE.setAdvertisedService(service);

  service.addCharacteristic(strokeCharacteristic);
  BLE.addService(service);
  BLE.advertise();

  *stroke_state           = eWaiting;
  *stroke_transmit_length = 0;
  stroke_length           = 0;

  setup_tflite();
}

void loop() {
  BLEDevice central = BLE.central();

  static bool was_connected_last = false;
  if (central && !was_connected_last) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
  }
  was_connected_last = central;

  uint16_t numSamplesInFifo = getFIFOSize();
  if (numSamplesInFifo == 0) {
    return;
  }

  int accelerometer_samples_read = 0;
  int gyroscope_samples_read     = 0;

  ReadAccelerometerAndGyroscope(100,
                                &accelerometer_samples_read,
                                &gyroscope_samples_read);

  bool done_just_triggered = false;

  if (gyroscope_samples_read > 0) {
    uint32_t now = millis();

    float dt_total_ms;
    if (last_gyro_time_ms == 0) {
      dt_total_ms = (1000.0f / gyroscope_sample_rate);
    } else {
      dt_total_ms = (float)(now - last_gyro_time_ms);
      if (dt_total_ms <= 0.0f) {
        dt_total_ms = (1000.0f / gyroscope_sample_rate);
      }
    }
    last_gyro_time_ms = now;

    float dt_per_sample = (dt_total_ms / 1000.0f) / gyroscope_samples_read;
    if (dt_per_sample <= 0.0f || dt_per_sample > 0.1f) {
      dt_per_sample = 1.0f / gyroscope_sample_rate;
    }

    EstimateGyroscopeDrift(current_gyroscope_drift);
    UpdateOrientation(gyroscope_samples_read,
                      current_gravity,
                      current_gyroscope_drift,
                      dt_per_sample);
    UpdateStroke(gyroscope_samples_read, &done_just_triggered);

    if (central && central.connected()) {
      strokeCharacteristic.writeValue(stroke_struct_buffer,
                                      stroke_struct_byte_count);
    }
  }

  if (accelerometer_samples_read > 0) {
    EstimateGravityDirection(current_gravity);
    UpdateVelocity(accelerometer_samples_read, current_gravity);
  }

  if (done_just_triggered) {
    RasterizeStroke(stroke_points,
                    *stroke_transmit_length,
                    0.6f, 0.6f,
                    raster_width,
                    raster_height,
                    raster_buffer);

    for (int y = 0; y < raster_height; ++y) {
      char line[raster_width + 1];
      for (int x = 0; x < raster_width; ++x) {
        const int8_t* pixel =
            &raster_buffer[(y * raster_width * raster_channels) +
                           (x * raster_channels)];
        const int8_t red   = pixel[0];
        const int8_t green = pixel[1];
        const int8_t blue  = pixel[2];
        char output = ((red > -128) || (green > -128) || (blue > -128))
                        ? '#'
                        : '.';
        line[x] = output;
      }
      line[raster_width] = 0;
      Serial.println(line);
    }

    TfLiteTensor* model_input = interpreter->input(0);
    for (int i = 0; i < raster_byte_count; ++i) {
      model_input->data.int8[i] = raster_buffer[i];
    }

    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      Serial.println("Invoke failed");
      return;
    }

    TfLiteTensor* output = interpreter->output(0);

    int8_t max_score = 0;
    int   max_index  = 0;
    for (int i = 0; i < label_count; ++i) {
      const int8_t score = output->data.int8[i];
      if ((i == 0) || (score > max_score)) {
        max_score = score;
        max_index = i;
      }
    }
    Serial.printf("Found %s (%d)\n", labels[max_index], max_score);

    *stroke_state           = eWaiting;
    *stroke_transmit_length = 0;
    stroke_length           = 0;
  }
}
