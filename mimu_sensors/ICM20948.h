#ifndef ICM20948_h_INCLUDED
#define ICM20948_h_INCLUDED

#include <string_view>
#include "MIMUSensor.h"

struct MIMUSensor
{
    static auto name() { return "ICM20948"; }
    static auto uuid() { return "1a5276ec-9fc1-43fb-beeb-76f819929422"; }

    struct serial_t {
        bool (*read_bytes)  (uint8_t reg,       uint8_t * buffer, uint8_t length, void * context);
        bool (*write_bytes) (uint8_t reg, const uint8_t * buffer, uint8_t length, void * context);
        void (*delay)(uint32_t milliseconds);
        void * context;
    };

    serial_t * serial;
    
    struct inputs_t : public MIMUSensor::inputs_t {
    } inputs;

    struct outputs_t : public MIMUSensor::outputs_t {
    } outputs;

    void setup();
    void loop();

    void _update_sleep();
    void _update_accelerometer_range();
    void _update_gyroscope_range();
    void _update_magnetometer_range();
    void _update_sampling_rate();
    void _set_error_condition() { outputs.error_condition.value = true; }
};

#endif // ICM20948_h_INCLUDED
