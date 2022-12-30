#ifndef ICM20948_h_INCLUDED
#define ICM20948_h_INCLUDED

#include <string_view>
#include "MIMUSensor.h"

struct ICM20948
{
    static auto name() { return "ICM20948"; }
    static auto uuid() { return "1a5276ec-9fc1-43fb-beeb-76f819929422"; }

    struct serial_t {
        bool (*read_bytes)  (uint8_t reg,       uint8_t * buffer, uint8_t length, void * context);
        bool (*write_bytes) (uint8_t reg, const uint8_t * buffer, uint8_t length, void * context);
        void (*delay)(uint32_t milliseconds);
        void * context;
    };

    enum byte_operation {READ, WRITE};
    static bool byte_op(ICM20948& mimu, byte_operation op, uint8_t reg, uint8_t& b)
    {
        if (!mimu.serial)
        {
            mimu.outputs.error_condition.value = true;
            mimu.outputs.error.value = mimu::outputs_t::SERIAL_ERROR;
            if (mimu.outputs.error.call.function) mimu.outputs.error.call.function(mimu.outputs.error.call.context, "ICM20948: Serial interface uninitialized\n");
            return false;
        }
        bool success;
        switch (op)
        {
            case WRITE:
                success = !mimu.serial->write_byte(reg, &b, 1, mimu.serial.context);
                break;
            case READ:
                success = !mimu.serial->read_byte(reg, &b, 1, mimu.serial.context);
                break;
        }
        if (!success)
        {
            mimu.outputs.error_condition.value = true;
            mimu.outputs.error.value = mimu::outputs_t::SERIAL_ERROR;
            if (mimu.outputs.error.call.function) mimu.outputs.error.call.function(mimu.outputs.error.call.context, "ICM20948: write_byte failed\n");
        }
        return success;
    }

    enum read_modify_write_operator {SET, CLEAR};
    static void read_modify_write(ICM20948& mimu, uint8_t reg, uint8_t b, read_modify_write_operator op)
    {
        uint8_t current;
        if (!mimu.byte_op(mimu, READ, reg, &current)) return;
    }

    static void write_byte(ICM20948& mimu, uint8_t reg, uint8_t b)
    {
        if (!mimu.serial)
        {
            mimu.outputs.error_condition.value = true;
            mimu.outputs.error.value = mimu::outputs_t::SERIAL_ERROR;
            if (mimu.outputs.error.call.function) mimu.outputs.error.call.function(mimu.outputs.error.call.context, "ICM20948: Serial interface uninitialized\n");
            return;
        }
        if (!mimu.serial->write_byte(reg, &b, mimu.serial.context)
        {
            mimu.outputs.error_condition.value = true;
            mimu.outputs.error.value = mimu::outputs_t::SERIAL_ERROR;
            if (mimu.outputs.error.call.function) mimu.outputs.error.call.function(mimu.outputs.error.call.context, "ICM20948: write_byte failed\n");
            return;
        }
    }

    serial_t * serial;

    struct enumerator {
        std::string_view name;
        uint8_t bit_value;
    };

    struct inputs_t : public MIMUSensor::inputs_t<ICM20948> {
        struct {
            static auto name() { return "register bank select"; };
            static auto c_name() { return "reg_bank_sel"; };
            static uint8_t address() { return 0x7F; };
            struct range {
                static enumerator values[5] = {
                    {"user bank 0", 0x00},
                    {"user bank 1", 0x10},
                    {"user bank 2", 0x20},
                    {"user bank 3", 0x30},
                    {"unknown following device reset", 0x40};
                };
                static uint8_t init = 4;
            };
            uint8_t value;
            void update(ICM20948& mimu) {serial.write_bytes(address(), &mimu.inputs.reg_bank_sel.value, 1, mimu.serial.context)};
        } reg_bank_sel;
        struct user_bank_0_t {
        } user_bank_0;
    } inputs;

    struct outputs_t : public MIMUSensor::outputs_t {
        enum error_type : int32_t { NO_ERROR = 0, SERIAL_ERROR};
        struct {
            static auto name() { return "errors"; }
            struct range {
                static std::string_view values[0] = {"NO_ERROR", "SERIAL_ERROR"};
                error_type init = NO_ERROR;
            };
            error_type value;
            struct {
                void (*function) (void * context, std::string_view error_message);
                void * context;
            } call;
        } errors;
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
