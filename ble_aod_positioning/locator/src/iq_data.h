#ifndef IQ_DATA_H
#define IQ_DATA_H

#include <stdbool.h> // For bool.
#include <stdint.h> // For uint8_t, int8_t, and int64_t.
#include <zephyr/bluetooth/bluetooth.h> // For BLE advertising info structure.
#include <zephyr/bluetooth/direction.h> // For BLE direction finding IQ samples report structure.
#include "bt_addr_utils.h" // For BT_ADDR_SIZE (6).

// TODO(wathne): Use sample16 instead of sample?
// TODO(wathne): Use dynamic IQ sampling settings. See report and info.
// TODO(wathne): Rename iq_raw_samples?

// Raw IQ samples are separated into reference samples and measurement samples.
// By default, each IQ samples report provides a total of 45 raw IQ samples. The
// first 8 IQ samples are reference samples and the remaining 37 IQ samples are
// measurement samples.

// "... the receiver shall take an IQ sample each microsecond during the
// reference period and an IQ sample each sample slot (thus there will be 8
// reference IQ samples, 1 to 37 IQ samples with 2 μs slots ..."
// - Bluetooth Core Specification 5.4

// Interval between samples in the reference period, in microseconds per
// reference sample.
// This constant must be set according to IQ sampling settings.
#define IQ_REFERENCE_SPACING 1

// Interval between samples in the measurement period, in microseconds per
// measurement sample.
// This constant must be set according to IQ sampling settings.
#define IQ_MEASUREMENT_SPACING 4

// Maximium IQ reference sample count.
// This constant must be set according to IQ sampling settings.
#define IQ_REFERENCE_MAX 8

// Maximium IQ measurement sample count.
// This constant must be set according to IQ sampling settings.
#define IQ_MEASUREMENT_MAX 37

// Data pipeline:
// IQ samples report -> raw IQ samples structure -> IQ data structure.

// Raw IQ samples structure.
// Intermediate data structure for raw IQ samples extracted from an IQ samples
// report. The purpose of an intermediate data structure is to have minimial
// data processing in the cte_recv_cb() callback function. Data processing
// should not occupy the main thread, and data should be forwarded to the work
// queue as quickly as possible. The work queue will then queue, process and
// evict data to remain within hardware processing capabilities.
// See the iq_raw_samples_init() function.
struct iq_raw_samples {
    // Timestamp of when the IQ samples report arrived in the cte_recv_cb()
    // callback function. Elapsed time since the system booted, in milliseconds.
    // See the k_uptime_get() function.
    // See the iq_raw_samples_init() function.
    int64_t report_timestamp;

    // Bluetooth LE channel index.
    // BLE channels 0-36 are secondary advertising channels.
    // BLE channels 37-39 are primary advertising channels.
    // See the iq_raw_samples_init() function.
    uint8_t channel_index;

    // Bluetooth LE device address (MAC address) of the beacon in little-endian
    // format (protocol/reversed octet order).
    // "Multi-octet fields ... shall be transmitted with the least significant
    // octet first."
    // - Bluetooth Core Specification 5.4, Vol 6, Part B, Sections 1.2 - 1.3.
    // The Nordic Semiconductor BLE implementation follows the same reversed
    // octet ordering when storing BLE device addresses.
    // This is the raw format received directly from the Bluetooth stack.
    // See the iq_raw_samples_init() function.
    uint8_t beacon_mac[BT_ADDR_SIZE];

    // Raw IQ sample count, constrained by maximum IQ sample count constants.
    // sample_count <= (IQ_REFERENCE_MAX + IQ_MEASUREMENT_MAX)
    // See the iq_raw_samples_init() function.
    uint8_t sample_count;

    // Raw I (In-phase) samples.
    // See the iq_raw_samples_init() function.
    int8_t i[IQ_REFERENCE_MAX + IQ_MEASUREMENT_MAX];

    // Raw Q (Quadrature) samples.
    // See the iq_raw_samples_init() function.
    int8_t q[IQ_REFERENCE_MAX + IQ_MEASUREMENT_MAX];
};

// IQ data structure.
// Raw IQ samples are separated into reference samples and measurement samples.
// See the iq_data_init() function.
struct iq_data {
    // See the iq_data_init() function.
    bool initialized;

    // Timestamp of when the IQ samples report arrived in the cte_recv_cb()
    // callback function. Elapsed time since the system booted, in milliseconds.
    // See the k_uptime_get() function.
    // See the iq_data_init() function.
    int64_t report_timestamp;

    // Bluetooth LE channel index.
    // BLE channels 0-36 are secondary advertising channels.
    // BLE channels 37-39 are primary advertising channels.
    // See the iq_data_init() function.
    uint8_t channel_index;

    // Bluetooth LE device address (MAC address) of the beacon in little-endian
    // format (protocol/reversed octet order).
    // "Multi-octet fields ... shall be transmitted with the least significant
    // octet first."
    // - Bluetooth Core Specification 5.4, Vol 6, Part B, Sections 1.2 - 1.3.
    // The Nordic Semiconductor BLE implementation follows the same reversed
    // octet ordering when storing BLE device addresses.
    // This is the raw format received directly from the Bluetooth stack.
    // See the iq_data_init() function.
    uint8_t beacon_mac[BT_ADDR_SIZE];

    // Reference sample count, constrained by IQ_REFERENCE_MAX.
    // See the iq_data_init() function.
    uint8_t reference_sample_count;

    // Measurement sample count, constrained by IQ_MEASUREMENT_MAX.
    // See the iq_data_init() function.
    uint8_t measurement_sample_count;

    // Raw IQ samples separated into reference samples and measurement samples.
    // See the iq_data_init() function.
    int8_t reference_i[IQ_REFERENCE_MAX];
    int8_t reference_q[IQ_REFERENCE_MAX];
    int8_t measurement_i[IQ_MEASUREMENT_MAX];
    int8_t measurement_q[IQ_MEASUREMENT_MAX];

    // Reference phase angles in radians.
    // See the calculate_reference_phases() function.
    float reference_phases[IQ_REFERENCE_MAX];

    // Measurement phase angles in radians.
    // See the calculate_measurement_phases() function.
    float measurement_phases[IQ_MEASUREMENT_MAX];

    // Unwrapped reference phase angles in radians.
    // See the unwrap_reference_phases() function.
    float reference_phases_unwrapped[IQ_REFERENCE_MAX];

    // Linear phase drift rate in radians per microsecond.
    // See the estimate_linear_phase_drift_rate() function.
    float linear_phase_drift_rate;

    // Measurement samples compensated at a linear phase drift rate.
    // See the compensate_measurement_samples() function.
    float measurement_i_compensated[IQ_MEASUREMENT_MAX];
    float measurement_q_compensated[IQ_MEASUREMENT_MAX];

    // Measurement phase angles compensated at a linear phase drift rate.
    // See the calculate_compensated_measurement_phases() function.
    float measurement_phases_compensated[IQ_MEASUREMENT_MAX];

    // TODO(wathne): Add documentation.
    float local_direction_cosine_x;
    float local_direction_cosine_y;
    float local_direction_cosine_z;
    float global_direction_cosine_x;
    float global_direction_cosine_y;
    float global_direction_cosine_z;

    // TODO(wathne): Add documentation.
    float aod_azimuth;
    float aod_elevation;
};

// Initialize a raw IQ samples structure from an IQ samples report.
// TODO(wathne): Add a comment about bt_le_per_adv_sync_info.
// The report_timestamp argument must be a timestamp of when the IQ samples
// report arrived in the cte_recv_cb() callback function. Elapsed time since the
// system booted, in milliseconds.
// See the k_uptime_get() function.
void iq_raw_samples_init(
        struct iq_raw_samples *iq_raw_samples,
        const struct bt_df_per_adv_sync_iq_samples_report *report,
        const struct bt_le_per_adv_sync_info *info,
        int64_t report_timestamp);

// Initialize an IQ data structure from a raw IQ samples structure.
// The iq_raw_samples argument must be a pointer to an initialized raw IQ
// samples structure.
// See the iq_raw_samples_init() function.
void iq_data_init(
        struct iq_data *iq_data,
        const struct iq_raw_samples *iq_raw_samples);

// Process IQ data.
// This function is compatible with the iq_raw_samples_processor_t function
// pointer type and can be set as the processor function in an IQ data work
// queue structure.
// The iq_raw_samples argument must be a pointer to an initialized raw IQ
// samples structure.
// See the iq_raw_samples_init() function.
void iq_data_process(const struct iq_raw_samples *iq_raw_samples);

#endif // IQ_DATA_H