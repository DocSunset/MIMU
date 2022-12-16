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
            void update(MIMUSensor& mimu) { mimu.update_sleep(); }
            bool value;
        } sleep;

        struct {
            static auto name() { return "accelerometer range"; }
            static auto c_name() { return "accelerometer_range"; }
            static auto unit() { return "g"; }
            void update(MIMUSensor& mimu) { mimu.update_accelerometer_range(); }
            float value;
        } accelerometer_range;

        struct {
            static auto name() { return "gyroscope range"; }
            static auto c_name() { return "gyroscope_range"; }
            static auto unit() { return "deg/s"; }
            void update(MIMUSensor& mimu) { mimu.update_gyroscope_range(); }
            float value;
        } gyroscope_range;

        struct {
            static auto name() { return "magnetometer range"; }
            static auto c_name() { return "magnetometer_range"; }
            static auto unit() { return "uT"; }
            void update(MIMUSensor& mimu) { mimu.update_magnetometer_range(); }
            float value;
        } magnetometer_range;

        struct {
            static auto name() { return "sampling rate"; }
            static auto c_name() { return "sampling_rate"; }
            void update(MIMUSensor& mimu) { mimu.update_sampling_rate(); }
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
                struct {
                    static auto name() { return "sensitivity"; }
                    static auto unit() { return "m/s/s/LSB"; }
                    float value;
                } sensitivity;
                struct {
                    static auto name() { return "output data rate"; }
                    static auto c_name() { return "odr"; }
                    float value;
                } odr;
                uint16_t x;
                uint16_t y;
                uint16_t z;
            } accelerometer;
            struct {
                static auto name() { return "gyroscope"; }
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
                uint16_t x;
                uint16_t y;
                uint16_t z;
            } gyroscope;
            struct {
                static auto name() { return "magnetometer"; }
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
                uint16_t x;
                uint16_t y;
                uint16_t z;
            } magnetometer;
        } raw;

        struct {
            static auto name() { return "normalized"; }
            struct {
                static auto name() { return "accelerometer"; }
                static auto unit() { return "m/s/s"; }
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
            } accelerometer;
            struct {
                static auto name() { return "gyroscope"; }
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
            } gyroscope;
            struct {
                static auto name() { return "magnetometer"; }
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
            } magnetometer;
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

    void update_sleep();
    void update_accelerometer_range();
    void update_gyroscope_range();
    void update_magnetometer_range();
    void update_sampling_rate();
    void set_error_condition() { outputs.error_condition.value = true; }
};

#endif // MIMUSensor_h_INCLUDED
