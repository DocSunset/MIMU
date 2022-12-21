#ifndef MIMUSensor_h_INCLUDED
#define MIMUSensor_h_INCLUDED

#include <string_view>

struct MIMUSensor
{
    static auto name() { return "MIMUSensor"; }
    static auto uuid() { return "8b2f5018-4760-495e-aa18-2cea7ade21a4"; }

    struct inputs_t {
        struct {
            static auto name() { return "sleep"; }
            void update(MIMUSensor& mimu) { mimu._update_sleep(); }
            bool value;
        } sleep;

        struct {
            static auto name() { return "accelerometer range"; }
            static auto c_name() { return "accl_range"; }
            static auto unit() { return "g"; }
            void update(MIMUSensor& mimu) { mimu._update_accelerometer_range(); }
            float value;
        } accl_range;

        struct {
            static auto name() { return "gyroscope range"; }
            static auto c_name() { return "gyro_range"; }
            static auto unit() { return "deg/s"; }
            void update(MIMUSensor& mimu) { mimu._update_gyroscope_range(); }
            float value;
        } gyro_range;

        struct {
            static auto name() { return "magnetometer range"; }
            static auto c_name() { return "magn_range"; }
            static auto unit() { return "uT"; }
            void update(MIMUSensor& mimu) { mimu._update_magnetometer_range(); }
            float value;
        } magn_range;

        struct {
            static auto name() { return "sampling rate"; }
            static auto c_name() { return "sampling_rate"; }
            void update(MIMUSensor& mimu) { mimu._update_sampling_rate(); }
            float value;
        } sampling_rate;
    } inputs;

    enum DeviceType {ICM20948}; // TODO: add support for LSM9DS1 and maybe others

    struct outputs_t {
        struct {
            static auto name() { return "error condition"; }
            static auto c_name() { return "error_condition"; }
            bool value;
        } error_condition;
        
        struct {
            static auto name() { return "device type"; }
            static auto c_name() { return "device_type"; }
            struct range {
                std::string_view values[1] = {"ICM20948"}; 
                DeviceType init{ICM20948};
            };
            DeviceType value;
        } device_type;

        struct {
            static auto name() { return "raw"; }
            struct {
                static auto name() { return "accelerometer"; }
                static auto c_name() { return "accl"; }
                struct {
                    static auto name() { return "sensitivity"; }
                    static auto unit() { return "g/LSB"; }
                    float value;
                } sensitivity;
                struct {
                    static auto name() { return "output data rate"; }
                    static auto c_name() { return "odr"; }
                    float value;
                } odr;
                int16_t x;
                int16_t y;
                int16_t z;
            } accl;
            struct {
                static auto name() { return "gyroscope"; }
                static auto c_name() { return "gyro"; }
                struct {
                    static auto name() { return "sensitivity"; }
                    static auto unit() { return "deg/s/LSB"; }
                    float value;
                } sensitivity;
                struct {
                    static auto name() { return "output data rate"; }
                    static auto c_name() { return "odr"; }
                    float value;
                } odr;
                int16_t x;
                int16_t y;
                int16_t z;
            } gyro;
            struct {
                static auto name() { return "magnetometer"; }
                static auto c_name() { return "magn"; }
                struct {
                    static auto name() { return "sensitivity"; }
                    static auto unit() { return "uT/LSB"; }
                    float value;
                } sensitivity;
                struct {
                    static auto name() { return "output data rate"; }
                    static auto c_name() { return "odr"; }
                    float value;
                } odr;
                int16_t x;
                int16_t y;
                int16_t z;
            } magn;
        } raw;

        struct {
            static auto name() { return "normalized"; }
            struct {
                static auto name() { return "accelerometer"; }
                static auto c_name() { return "accl"; }
                static auto unit() { return "g"; }
                struct {
                    static auto name() { return "range"; }
                    struct {
                        static auto name() { return "min"; }
                        float value;
                    } min;
                    struct {
                        static auto name() { return "max"; }
                        float value;
                    } max;
                } range;
                float x;
                float y;
                float z;
            } accl;
            struct {
                static auto name() { return "gyroscope"; }
                static auto c_name() { return "gyro"; }
                static auto unit() { return "deg/s"; }
                struct {
                    static auto name() { return "range"; }
                    struct {
                        static auto name() { return "min"; }
                        float value;
                    } min;
                    struct {
                        static auto name() { return "max"; }
                        float value;
                    } max;
                } range;
                float x;
                float y;
                float z;
            } gyro;
            struct {
                static auto name() { return "magnetometer"; }
                static auto c_name() { return "magn"; }
                static auto unit() { return "uT"; }
                struct {
                    static auto name() { return "range"; }
                    struct {
                        static auto name() { return "min"; }
                        float value;
                    } min;
                    struct {
                        static auto name() { return "max"; }
                        float value;
                    } max;
                } range;
                float x;
                float y;
                float z;
            } magn;
        } normalized;
    } outputs;

    struct spi_t {
        bool (*read_bytes)  (uint8_t reg,       uint8_t * buffer, uint8_t length, void * context);
        bool (*write_bytes) (uint8_t reg, const uint8_t * buffer, uint8_t length, void * context);
        void * context;
    };
    spi_t * spi;
    // TODO: add support for I2C
    
    void (*delay)(uint32_t milliseconds);

    void setup();

    void loop();

    void _update_sleep();
    void _update_accelerometer_range();
    void _update_gyroscope_range();
    void _update_magnetometer_range();
    void _update_sampling_rate();
    void _set_error_condition() { outputs.error_condition.value = true; }
};

#endif // MIMUSensor_h_INCLUDED
