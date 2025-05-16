#include "iq_data.h"
#include <math.h>
#include <zephyr/bluetooth/hci_types.h> // For bt_hci_le_iq_sample.
#include "ble_channel_constants.h" // For BLE channel lookup tables (LUTs).
#include "bt_addr_utils.h" // For BT_ADDR_SIZE (6) and bt_addr_mac_compare().
#include "chw1010_ant2_specs.h" // For antenna_spacing_orthogonal (37.5f).
#include "locator.h" // For locator structure and g_locator instance.
#include "directional_statistics.h" // For directional_statistics_circular_mean().

// TODO(wathne): Revise all #include directives, with comments.
// TODO(wathne): Use sample16 instead of sample?
// TODO(wathne): Use dynamic IQ sampling settings.
// TODO(wathne): Rename iq_raw_samples?

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Mathematical constant for 180/Pi as floating point type.
#define IQ_DATA_DEGREES_RADIANS_RATIO 57.295776f

// Raw IQ samples are separated into reference samples and measurement samples.
// By default, each IQ samples report provides a total of 45 raw IQ samples. The
// first 8 IQ samples are reference samples and the remaining 37 IQ samples are
// measurement samples.

// Data pipeline:
// IQ samples report -> raw IQ samples structure -> IQ data structure.

void iq_raw_samples_init(
        struct iq_raw_samples *iq_raw_samples,
        const struct bt_df_per_adv_sync_iq_samples_report *report,
        const struct bt_le_per_adv_sync_info *info,
        int64_t report_timestamp) {
    // Set timestamp of when the IQ samples report arrived in the cte_recv_cb()
    // callback function. Elapsed time since the system booted, in milliseconds.
    // See the k_uptime_get() function.
    iq_raw_samples->report_timestamp = report_timestamp;

    // Set Bluetooth LE channel index.
    iq_raw_samples->channel_index = report->chan_idx;

    // Set Bluetooth LE device address (MAC address) of the beacon in
    // little-endian format (protocol/reversed octet order).
    memcpy(iq_raw_samples->beacon_mac, info->addr.a.val, BT_ADDR_SIZE);

    static const int MAXIMUM_SAMPLES = IQ_REFERENCE_MAX + IQ_MEASUREMENT_MAX;
    // Set sample_count, constrained by maximum IQ sample count constants.
    // sample_count <= (IQ_REFERENCE_MAX + IQ_MEASUREMENT_MAX)
    iq_raw_samples->sample_count = report->sample_count;
    if (iq_raw_samples->sample_count > MAXIMUM_SAMPLES) {
        iq_raw_samples->sample_count = MAXIMUM_SAMPLES;
    }

    // Set raw IQ samples from IQ samples report.
    for (int i = 0; i < iq_raw_samples->sample_count; i++) {
        iq_raw_samples->i[i] = report->sample[i].i;
        iq_raw_samples->q[i] = report->sample[i].q;
    }
}

void iq_data_init(
        struct iq_data *iq_data,
        const struct iq_raw_samples *iq_raw_samples) {
    // Set timestamp of when the IQ samples report arrived in the cte_recv_cb()
    // callback function. Elapsed time since the system booted, in milliseconds.
    // See the k_uptime_get() function.
    iq_data->report_timestamp = iq_raw_samples->report_timestamp;

    // Set Bluetooth LE channel index.
    iq_data->channel_index = iq_raw_samples->channel_index;

    // Set Bluetooth LE device address (MAC address) of the beacon in
    // little-endian format (protocol/reversed octet order).
    memcpy(iq_data->beacon_mac, iq_raw_samples->beacon_mac, BT_ADDR_SIZE);

    static const int MAXIMUM_SAMPLES = IQ_REFERENCE_MAX + IQ_MEASUREMENT_MAX;
    uint8_t sample_count = iq_raw_samples->sample_count;
    if (sample_count > MAXIMUM_SAMPLES) {
        sample_count = MAXIMUM_SAMPLES;
    }

    // Set reference_sample_count, constrained by IQ_REFERENCE_MAX.
    if (sample_count >= IQ_REFERENCE_MAX) {
        iq_data->reference_sample_count = IQ_REFERENCE_MAX;
    } else {
        iq_data->reference_sample_count = sample_count;
    }

    // Set measurement_sample_count, constrained by IQ_MEASUREMENT_MAX.
    iq_data->measurement_sample_count =
            sample_count - iq_data->reference_sample_count;

    // Set reference samples from raw IQ samples.
    for (int i = 0; i < iq_data->reference_sample_count; i++) {
        iq_data->reference_i[i] = iq_raw_samples->i[i];
        iq_data->reference_q[i] = iq_raw_samples->q[i];
    }

    // Set measurement samples from raw IQ samples.
    for (int i = 0; i < iq_data->measurement_sample_count; i++) {
        iq_data->measurement_i[i] =
                iq_raw_samples->i[i + iq_data->reference_sample_count];
        iq_data->measurement_q[i] =
                iq_raw_samples->q[i + iq_data->reference_sample_count];
    }

    iq_data->initialized = true;
}

// TODO(wathne): Why is there a systematic intersample phase shift of 180
// degrees between samples in the reference period? There is conflicting
// information about the expected intersample phase shifts in the reference
// period. The intersample phase shifts should be about 90 degrees for 1 μs
// intervals at 250 kHz (CTE), but there are forum posts hinting at 360 (0)
// degrees for 1 μs intervals at 1000 kHz, which does not make sense. The CTE is
// supposed to always be 250 kHz. Despite this nonsense, the current assumption
// of 1000 kHz and 360 (0) degrees remains for the simple reason that this seems
// to net good estimates for the systematic linear phase drift if a temporary
// fix is applied to every other reference sample. This issue should be
// revisited, but the temporary fix works for now.
// IQ samples in the reference period have unexpected intersample phase shifts
// of 180 degrees. This temporary fix will undo the intersample phase shifts by
// rotating every other reference sample by 180 degrees.
// The iq_data argument must be a pointer to an initialized IQ data structure.
// See the iq_data_init() function.
static void iq_data_temp_fix_ref_samples(struct iq_data *iq_data) {
    if (!iq_data || !iq_data->initialized) {
        return;
    }

    for (int i = 1; i < iq_data->reference_sample_count; i = i + 2) {
        // int8_t range is -128 to 127. This is a special case for -128.
        if (iq_data->reference_i[i] == -128) {
            iq_data->reference_i[i] = 127;
        } else {
            iq_data->reference_i[i] = -iq_data->reference_i[i];
        }

        // int8_t range is -128 to 127. This is a special case for -128.
        if (iq_data->reference_q[i] == -128) {
            iq_data->reference_q[i] = 127;
        } else {
            iq_data->reference_q[i] = -iq_data->reference_q[i];
        }
    }
}

// Calculate reference phases for an IQ data structure.
// Populates reference_phases[] with phase angles in radians.
// The iq_data argument must be a pointer to an initialized IQ data structure.
// See the iq_data_init() function.
static void calculate_reference_phases(struct iq_data *iq_data) {
    if (!iq_data || !iq_data->initialized) {
        return;
    }

    // "Arg(x) is the principal value of the argument, or phase angle, of the
    // complex number x, in the range (–π, π ] ...
    // ... Given the IQ samples I(n) and Q(n) from sample slot n , the phase
    // φ(n) equals Arg(I(n) + iQ(n))" - Bluetooth Core Specification 5.4
    for (int i = 0; i < iq_data->reference_sample_count; i++) {
        iq_data->reference_phases[i] = atan2f(
                iq_data->reference_q[i],
                iq_data->reference_i[i]);
    }
}

// Calculate measurement phases for an IQ data structure.
// Populates measurement_phases[] with phase angles in radians.
// The iq_data argument must be a pointer to an initialized IQ data structure.
// See the iq_data_init() function.
static void calculate_measurement_phases(struct iq_data *iq_data) {
    if (!iq_data || !iq_data->initialized) {
        return;
    }

    // "Arg(x) is the principal value of the argument, or phase angle, of the
    // complex number x, in the range (–π, π ] ...
    // ... Given the IQ samples I(n) and Q(n) from sample slot n , the phase
    // φ(n) equals Arg(I(n) + iQ(n))" - Bluetooth Core Specification 5.4
    for (int i = 0; i < iq_data->measurement_sample_count; i++) {
        iq_data->measurement_phases[i] = atan2f(
                iq_data->measurement_q[i],
                iq_data->measurement_i[i]);
    }
}

// Unwrap reference phases for an IQ data structure.
// Populates reference_phases_unwrapped[] with unwrapped phase angles.
// The iq_data argument must be a pointer to an initialized IQ data structure.
// See the iq_data_init() function.
// iq_data->reference_phases[] must be populated.
// See the calculate_reference_phases() function.
static void unwrap_reference_phases(struct iq_data *iq_data) {
    if (!iq_data || !iq_data->initialized) {
        return;
    }

    if (iq_data->reference_sample_count == 0) {
        return;
    }

    // Initial phase.
    iq_data->reference_phases_unwrapped[0] = iq_data->reference_phases[0];

    // Subsequent phases.
    for (int i = 1; i < iq_data->reference_sample_count; i++) {
        float phase_difference =
                iq_data->reference_phases[i] -
                iq_data->reference_phases_unwrapped[i - 1];

        // Unwrap phase angle if absolute phase difference is greater than pi.
        if (phase_difference > M_PI) {
            iq_data->reference_phases_unwrapped[i] =
                    iq_data->reference_phases[i] - 2 * M_PI;
        } else if (phase_difference < -M_PI) {
            iq_data->reference_phases_unwrapped[i] =
                    iq_data->reference_phases[i] + 2 * M_PI;
        } else {
            iq_data->reference_phases_unwrapped[i] =
                    iq_data->reference_phases[i];
        }
    }
}

// Estimate linear phase drift rate for an IQ data structure.
// Sets linear_phase_drift_rate to the estimated rate of radians per
// microsecond.
// Calculates reference phases and populates reference_phases[] with phase
// angles in radians.
// Unwraps reference phases and populates reference_phases_unwrapped[] with
// unwrapped phase angles.
// The iq_data argument must be a pointer to an initialized IQ data structure.
// See the iq_data_init() function.
static void estimate_linear_phase_drift_rate(struct iq_data *iq_data) {
    if (!iq_data || !iq_data->initialized) {
        return;
    }

    if (iq_data->reference_sample_count == 0) {
        iq_data->linear_phase_drift_rate = 0.0f;
        return;
    }

    // Calculate reference phases and populate reference_phases[] with phase
    // angles in radians.
    calculate_reference_phases(iq_data);

    // Unwrap reference phases and populate reference_phases_unwrapped[] with
    // unwrapped phase angles.
    unwrap_reference_phases(iq_data);

    if (iq_data->reference_sample_count == 1) {
        iq_data->linear_phase_drift_rate = 0.0f;
        return;
    }

    // Linear regression using the least squares method:
    // y = mx + b
    // m = (n∑xy - ∑x∑y) / (n∑x^2 - (∑x)^2)
    // b = (∑y - m∑x) / n
    // Where m is the estimated linear phase drift rate in radians per reference
    // sample.
    const uint8_t n = iq_data->reference_sample_count;
    float y;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xy = 0.0f;
    float sum_xx = 0.0f;
    for (int x = 0; x < n; x++) {
        y = iq_data->reference_phases_unwrapped[x];
        sum_x = sum_x + x;
        sum_y = sum_y + y;
        sum_xy = sum_xy + x * y;
        sum_xx = sum_xx + x * x;
    }
    float m = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);

    // radians / microsecond
    // <=>
    // (radians / reference sample) * (reference samples / microsecond)
    // ~>
    // (m) * (1 / IQ_REFERENCE_SPACING)
    iq_data->linear_phase_drift_rate = (m / IQ_REFERENCE_SPACING);
}

// Compensate for linear phase drift in measurement samples for an IQ data
// structure.
// Populates measurement_i_compensated[] and measurement_q_compensated[] with
// measurement samples compensated at a linear phase drift rate.
// The iq_data argument must be a pointer to an initialized IQ data structure.
// See the iq_data_init() function.
// iq_data->linear_phase_drift_rate must be set.
// See the estimate_linear_phase_drift_rate() function.
static void compensate_measurement_samples(struct iq_data *iq_data) {
    if (!iq_data || !iq_data->initialized) {
        return;
    }

    // (-radians) / measurement sample
    // <=>
    // -(radians / microsecond ) * (microseconds / measurement sample)
    // ~>
    // -linear_phase_drift_rate * IQ_MEASUREMENT_SPACING
    float rate = -iq_data->linear_phase_drift_rate * IQ_MEASUREMENT_SPACING;
    for (int i = 0; i < iq_data->measurement_sample_count; i++) {
        float theta = rate * i;
        float cos_theta = cosf(theta);
        float sin_theta = sinf(theta);

        // i_c = i*cos(θ) - q*sin(θ)
        // q_c = i*sin(θ) + q*cos(θ)
        iq_data->measurement_i_compensated[i] =
                iq_data->measurement_i[i] * cos_theta -
                iq_data->measurement_q[i] * sin_theta;
        iq_data->measurement_q_compensated[i] =
                iq_data->measurement_i[i] * sin_theta +
                iq_data->measurement_q[i] * cos_theta;
    }
}

// Calculate compensated measurement phases for an IQ data structure.
// Populates measurement_phases_compensated[] with measurement phase angles
// compensated at a linear phase drift rate.
// The iq_data argument must be a pointer to an initialized IQ data structure.
// See the iq_data_init() function.
// measurement_i_compensated[] and measurement_q_compensated[] must be
// populated.
// See the compensate_measurement_samples() function.
static void calculate_compensated_measurement_phases(struct iq_data *iq_data) {
    if (!iq_data || !iq_data->initialized) {
        return;
    }

    // "Arg(x) is the principal value of the argument, or phase angle, of the
    // complex number x, in the range (–π, π ] ...
    // ... Given the IQ samples I(n) and Q(n) from sample slot n , the phase
    // φ(n) equals Arg(I(n) + iQ(n))" - Bluetooth Core Specification 5.4
    for (int i = 0; i < iq_data->measurement_sample_count; i++) {
        iq_data->measurement_phases_compensated[i] = atan2f(
                iq_data->measurement_q_compensated[i],
                iq_data->measurement_i_compensated[i]);
    }
}

// Estimate local direction cosines, azimuth, and elevation for an IQ data
// structure. Single row antenna pattern.
// Uses interferometry on compensated measurement samples.
// Sets local_direction_cosine_x, local_direction_cosine_y, and
// local_direction_cosine_z in the range [0, 1].
// Sets aod_azimuth and aod_elevation in radians.
// The iq_data argument must be a pointer to an initialized IQ data structure.
// See the iq_data_init() function.
// iq_data->measurement_i_compensated[] and iq_data->measurement_q_compensated[]
// must be populated with measurement samples compensated at a linear phase
// drift rate.
// See the compensate_measurement_samples() function.
static void iq_data_aod_row_interferometry(struct iq_data *iq_data) {
    if (!iq_data || !iq_data->initialized) {
        return;
    }

    // CoreHW CHW1010-ANT2-1.1 antenna grid for an antenna row:
    //  +----+----+----+----+
    //  |    |    |    |    |
    //  +----+----+----+----+
    //  |    |    |    |    |
    //  +----+----+----+----+
    //  |    |    |    |    |
    //  +----+----+----+----+
    //  |  2 |  3 |  4 |  6 |
    //  +----+----+----+----+

    // CoreHW CHW1010-ANT2-1.1 antenna coordinate system:
    //            Y
    //            |
    //    13   12 | 11    9
    //            |
    //    14   15 | 10    8
    //            +------------ X
    //     1    0    5    7
    //
    //     2    3    4    6
    //
    // Right-handed Cartesian (x, y, z) coordinate system.
    // X-axis: Points rightward when facing the array.
    // Y-axis: Points upward when facing the array.
    // Z-axis: Points outward from the array toward the locator.
    // Origin (0, 0, 0) is at the center of the array.
    // 
    // Azimuth is the angle in the XZ-plane with respect to the Z-axis.
    //   - Positive when the locator is to the right of the array (x > 0).
    //   - Negative when the locator is to the left of the array (x < 0).
    //   - Range is [-pi, pi].
    // Elevation is the angle from the XZ-plane toward the Y-axis.
    //   - Positive when the locator is above the XZ-plane (y > 0).
    //   - Negative when the locator is below the XZ-plane (y < 0).
    //   - Range is [-pi/2, pi/2].
    //
    // The above definition for Azimuth and Elevation is intuitive and
    // conventional, but note that many BLE direction finding references may use
    // an alternative convention:
    // (Alt.) Azimuth is the angle in the XY-plane with respect to the X-axis.
    // (Alt.) Elevation is the angle from the XY-plane toward the Z-axis.

    //float *phases = iq_data->measurement_phases_compensated;

    // Antenna switching sequence for 37 measurement samples.
    // This is for the default sample spacing of 4 microseconds where CTEType
    // field value is 2 for "AoD Constant Tone Extension with 2 μs slots".
    // antenna_switching_sequence[i] maps measurement index i to the antenna
    // number stored in antenna_switching_sequence[i].
    // For example, phases[4] was sampled from antenna 3, because
    // antenna_switching_sequence[4] = 3.
    static const uint8_t antenna_switching_sequence[37] = {
        3,  4,  6,  2,  3,  4,  6,  2,  3,  4,
        6,  2,  3,  4,  6,  2,  3,  4,  6,  2,
        3,  4,  6,  2,  3,  4,  6,  2,  3,  4,
        6,  2,  3,  4,  6,  2,  3
   };
    /* SWITCHPATTERN list, for reference, copied from beacon main.c:
     * 
     * SWITCHPATTERN[0]  = 0x0,  radio.dfe-pdu-antenna,  idle period (PDU Tx/Rx).
     * SWITCHPATTERN[1]  = 0x2,  ant_patterns[0],        guard and reference period.
     * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],         1st sample slot.
     * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],         2nd sample slot.
     * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],         3rd sample slot.
     * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],         4th sample slot.
     * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],         5th sample slot.
     * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],         6th sample slot.
     * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],         7th sample slot.
     * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],         8th sample slot.
     * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],         9th sample slot.
     * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        10th sample slot.
     * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        11th sample slot.
     * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        12th sample slot.
     * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        13th sample slot.
     * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        14th sample slot.
     * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        15th sample slot.
     * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        16th sample slot.
     * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        17th sample slot.
     * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        18th sample slot.
     * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        19th sample slot.
     * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        20th sample slot.
     * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        21st sample slot.
     * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        22nd sample slot.
     * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        23rd sample slot.
     * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        24th sample slot.
     * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        25th sample slot.
     * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        26th sample slot.
     * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        27th sample slot.
     * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        28th sample slot.
     * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        29th sample slot.
     * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        30th sample slot.
     * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        31st sample slot.
     * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        32nd sample slot.
     * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        33rd sample slot.
     * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        34th sample slot.
     * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        35th sample slot.
     * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        36th sample slot.
     * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        37th sample slot.
     */

    // Selected measurement index pairs for interferometry.
    // These numbers are measurement indices, not antenna numbers. This sequence
    // of index pairs is a repeating row pattern on the CoreHW CHW1010-ANT2-1.1
    // antenna grid. This row pattern ensures temporally adjacent measurements
    // of physically adjacent antennas. Measurement phases have been compensated
    // for an estimated linear phase drift, but some residual phase drift may
    // still remain in the compensated measurements. This row pattern aims to
    // minimize the effect of residual phase drift on the calculations by only
    // allowing temporally adjacent measurement pairs. Of the 27 selected pairs,
    // 0 pairs are vertically adjacent (bottom to top, top to bottom), and 27
    // pairs are horizontally adjacent (left to right, right to left).
    // For example, phases[19] and phases[20] make a valid pair, where
    // phases[19] (antenna 2) is to the left of phases[20] (antenna 3).
    // The pair encoding {19, 20, 0} is mathematically equivalent to {20, 19, 1}
    // when computing the phase delta. Both pair encodings represent the same
    // physical relationship.
    // The sign convention for phase delta is positive X and positive Y:
    // delta = phases[left antenna] - phases[right antenna], where a positive
    // phase delta means that the AoD locator is to the right of the origin in
    // the AoD beacon coordinate system.
    // delta = phases[bottom antenna] - phases[top antenna], where a positive
    // phase delta means the AoD locator is above the origin in the AoD beacon
    // coordinate system.
    // Pair direction encoding:
    // 0 = left to right
    // 1 = right to left
    // 2 = bottom to top
    // 3 = top to bottom
    static const struct {
        uint8_t index_1;
        uint8_t index_2;
        uint8_t direction;
    } measurement_pairs[27] = {
        { 0,  1, 0}, // left to right
        { 1,  2, 0}, // left to right

        { 3,  4, 0}, // left to right
        { 4,  5, 0}, // left to right
        { 5,  6, 0}, // left to right

        { 7,  8, 0}, // left to right
        { 8,  9, 0}, // left to right
        { 9, 10, 0}, // left to right

        {11, 12, 0}, // left to right
        {12, 13, 0}, // left to right
        {13, 14, 0}, // left to right

        {15, 16, 0}, // left to right
        {16, 17, 0}, // left to right
        {17, 18, 0}, // left to right

        {19, 20, 0}, // left to right
        {20, 21, 0}, // left to right
        {21, 22, 0}, // left to right

        {23, 24, 0}, // left to right
        {24, 25, 0}, // left to right
        {25, 26, 0}, // left to right

        {27, 28, 0}, // left to right
        {28, 29, 0}, // left to right
        {29, 30, 0}, // left to right

        {31, 32, 0}, // left to right
        {32, 33, 0}, // left to right
        {33, 34, 0}, // left to right

        {35, 36, 0}  // left to right
    };

    static const int measurement_pairs_length =
            sizeof(measurement_pairs) / sizeof(measurement_pairs[0]);

    uint8_t measurement_sample_count = iq_data->measurement_sample_count;
    if (measurement_sample_count < 3) {
        iq_data->local_direction_cosine_x = 0.0f;
        iq_data->local_direction_cosine_y = 0.0f;
        iq_data->local_direction_cosine_z = 1.0f;
        iq_data->aod_azimuth = 0.0f;
        iq_data->aod_elevation = 0.0f;
        return;
    }

    // BLE channels 0-36 are secondary advertising channels.
    // BLE channels 37-39 are primary advertising channels.
    uint8_t channel_index = iq_data->channel_index;

    // BLE channel frequency in MHz.
    uint16_t channel_frequency = ble_channel_get_frequency(channel_index);

    // BLE channel wavelength in millimeters.
    // lambda = c/f, where c = 299792458 meters/second, and f is BLE channel
    // frequency in MHz.
    float channel_wavelength = ble_channel_get_wavelength(channel_index);

    // BLE channel wavenumber in radians per millimeter.
    // k = 2*pi/lambda, where pi = 3.1415926535897932384626433832795, and lambda
    // is BLE channel wavelength in millimeters.
    float channel_wavenumber = ble_channel_get_wavenumber(channel_index);

    // CoreHW CHW1010-ANT2-1.1 antenna spacing for orthogonally adjacent
    // antennas, from antenna center to antenna center, in radians, at the BLE
    // channel frequency.
    // BLE channel wavenumber multiplied by orthogonal antenna spacing.
    // d_orth_rad = k * antenna_spacing_orthogonal, in radians.
    // For BLE channel index 18 (2442 Mhz):
    // d_orth_rad = 0.051181 rad/mm * 37.5mm = 1.91928750 rad.
    float d_orth_rad = channel_wavenumber * antenna_spacing_orthogonal;

    // By default, the interval between samples in the measurement period is 4
    // microseconds and the CTE frequency is 250 kilohertz. This is exactly 1
    // CTE cycle because (0.25 * 1000000) * (4 / 1000000) = 1. It is effectively
    // as if all 37 measurement samples are taken at the same time. If the
    // measurement samples are compensated for systematic linear phase drift,
    // then any remaining phase differences must be due to signal direction and
    // antenna positions. This enables conventional interferometry using first
    // differences, Delta(φ)[m] = φ[m] - φ[m-1], effectively emulating the
    // behavior of a conventional interferometer array.

    // TODO(wathne): Explain that AoD and AoA calculations are symmetric.
    // TODO(wathne): Explain how phase shifts at 250 kHz and 2.4 GHz are
    // related.

    // A note about the unit circle:
    // In mathematics, the unit circle range is (-pi, pi], where negative pi
    // is not included in the range. In the C math function atan2f(), the unit
    // circle range is [-pi, pi], where negative pi is included in the range.
    // Negative pi represents the case when the negative x-axis boundary
    // (+/- pi) is approached from below. This design choice is presumably
    // related to the limited precision of floating point numbers, and it
    // provides continuity when crossing the boundary.

    // The usual arithmetic mean is not appropriate for calculating the mean on
    // a unit circle with range (-pi, pi] or range [-pi, pi]. To see how the
    // arithmetic mean breaks down, consider the case where two angles, +0.9*pi
    // and -0.9*pi, are clustered around the negative x-axis. Their circular
    // mean is obviously +/- pi, which is exactly on the negative x-axis. Their
    // arithmetic mean is 0, which is exactly on the positive x-axis. The
    // arithmetic mean is 180 degrees off from the circular mean in this simple
    // example. Another approach is clearly necessary. There are two main
    // approaches for calculating a circular mean, either an extrinsic mean or
    // an intrinsic mean. The extrinsic mean is the simplest approach.

    // Extrinsic mean on a circle:
    // Put the circle in the two-dimensional Euclidean space (R^2), mapping
    // phase angles to (cos(φ), sin(φ)) coordinates. Calculate the average of
    // the (cos(φ), sin(φ)) coordinates. This average will be a point inside the
    // unit circle, but not on the unit circle itself. Finally, project the
    // point onto the unit circle using the atan2f() function. This extrinsic
    // mean is a good approximation of the true mean if the phase angles are
    // clustered.

    // Intrinsic mean on a circle:
    // TODO(wathne): Explain intrinsic circular mean. For now, see
    // "directional_statistics.h" and "directional_statistics.h".

    // TODO(wathne): Add more documentation.

    // First difference:
    // Delta(φ)[m] = φ[m] - φ[m-1]
    float delta;

    float horizontal_deltas[measurement_pairs_length];
    int horizontal_count = 0;
    float horizontal_mean = 0.0f;

    float vertical_deltas[measurement_pairs_length];
    int vertical_count = 0;
    float vertical_mean = 0.0f;

    for (int i = 0; i < measurement_pairs_length; i++) {
        uint8_t index_1 = measurement_pairs[i].index_1;
        uint8_t index_2 = measurement_pairs[i].index_2;
        uint8_t direction = measurement_pairs[i].direction;

        // Check if indices are out of bounds.
        if (index_1 >= measurement_sample_count ||
                index_2 >= measurement_sample_count) {
            continue;
        }

        float i1 = iq_data->measurement_i_compensated[index_1];
        float q1 = iq_data->measurement_q_compensated[index_1];
        float i2 = iq_data->measurement_i_compensated[index_2];
        float q2 = iq_data->measurement_q_compensated[index_2];

        float real_part = i1*i2 + q1*q2;
        float imag_part = q1*i2 - i1*q2;

        delta = atan2f(imag_part, real_part);

        // TODO(wathne): Remove this limit? Allow more than theoretical max?
        // Clamp delta if delta is greater than theoretical max, ~ 1.9 radians.
        // For BLE channel index 18 (2442 Mhz):
        // d_orth_rad = 0.051181 rad/mm * 37.5mm = 1.91928750 rad.
        if (delta > d_orth_rad) {
            // TODO(wathne): Remove this line.
            printk("Pair %d {%d, %d, %d}: CLAMPING %.6f to %.6f\n",
                    i, index_1, index_2, direction, delta, d_orth_rad);
            delta = d_orth_rad;
        } else if (delta < -d_orth_rad) {
            // TODO(wathne): Remove this line.
            printk("Pair %d {%d, %d, %d}: CLAMPING %.6f to %.6f\n",
                    i, index_1, index_2, direction, delta, -d_orth_rad);
            delta = -d_orth_rad;
        }

        // Pair direction decoding:
        // 0 = left to right
        // 1 = right to left
        // 2 = bottom to top
        // 3 = top to bottom
        switch (direction) {
            case 0:
                horizontal_deltas[horizontal_count] = delta;
                horizontal_count = horizontal_count + 1;
                break;
            case 1:
                horizontal_deltas[horizontal_count] = -delta;
                horizontal_count = horizontal_count + 1;
                break;
            case 2:
                vertical_deltas[vertical_count] = delta;
                vertical_count = vertical_count + 1;
                break;
            case 3:
                vertical_deltas[vertical_count] = -delta;
                vertical_count = vertical_count + 1;
                break;
        }
    }

    // Search for the intrinsic circular mean for horizontal deltas.
    if (horizontal_count > 0) {
        horizontal_mean = directional_statistics_circular_mean(
                horizontal_deltas,
                horizontal_count,
                5,
                0.01);
    }

    // Search for the intrinsic circular mean for vertical deltas.
    if (vertical_count > 0) {
        vertical_mean = directional_statistics_circular_mean(
                vertical_deltas,
                vertical_count,
                5,
                0.01);
    }

    float direction_cosine_x = 0.0f;
    if (horizontal_count > 0) {
        direction_cosine_x = -horizontal_mean / d_orth_rad;

        // Clamp to [-1, 1].
        if (direction_cosine_x > 1.0f) {
            direction_cosine_x = 1.0f;
        }
        if (direction_cosine_x < -1.0f) {
            direction_cosine_x = -1.0f;
        }
    }

    float direction_cosine_y = 0.0f;
    if (vertical_count > 0) {
        direction_cosine_y = -vertical_mean / d_orth_rad;

        // Clamp to [-1, 1].
        if (direction_cosine_y > 1.0f) {
            direction_cosine_y = 1.0f;
        }
        if (direction_cosine_y < -1.0f) {
            direction_cosine_y = -1.0f;
        }
    }

    // Calculate direction_cosine_z from the direction cosine relationship
    // cos^2(θx) + cos^2(θy) + cos^2(θz) = 1
    float direction_cosine_z_squared = 1.0f - (
            direction_cosine_x*direction_cosine_x +
            direction_cosine_y*direction_cosine_y);

    if (direction_cosine_z_squared < 0.0f) {
        direction_cosine_z_squared = 0.0f;
    }

    float direction_cosine_z = sqrtf(direction_cosine_z_squared);

    iq_data->local_direction_cosine_x = direction_cosine_x;
    iq_data->local_direction_cosine_y = direction_cosine_y;
    iq_data->local_direction_cosine_z = direction_cosine_z;

    iq_data->aod_azimuth = atan2f(direction_cosine_x, direction_cosine_z);
    iq_data->aod_elevation = asinf(direction_cosine_y);

    // TODO(wathne): Remove this line.
    printk("azimuth:   %.2f\n", iq_data->aod_azimuth);
    // TODO(wathne): Remove this line.
    printk("elevation: %.2f\n", iq_data->aod_elevation);
}

// Estimate local direction cosines, azimuth, and elevation for an IQ data
// structure. Full antenna pattern.
// Uses interferometry on compensated measurement samples.
// Sets local_direction_cosine_x, local_direction_cosine_y, and
// local_direction_cosine_z in the range [0, 1].
// Sets aod_azimuth and aod_elevation in radians.
// The iq_data argument must be a pointer to an initialized IQ data structure.
// See the iq_data_init() function.
// iq_data->measurement_i_compensated[] and iq_data->measurement_q_compensated[]
// must be populated with measurement samples compensated at a linear phase
// drift rate.
// See the compensate_measurement_samples() function.
static void iq_data_aod_interferometry(struct iq_data *iq_data) {
    if (!iq_data || !iq_data->initialized) {
        return;
    }

    // CoreHW CHW1010-ANT2-1.1 antenna grid:
    //  +----+----+----+----+
    //  | 13 | 12 | 11 |  9 |
    //  +----+----+----+----+
    //  | 14 | 15 | 10 |  8 |
    //  +----+----+----+----+
    //  |  1 |  0 |  5 |  7 |
    //  +----+----+----+----+
    //  |  2 |  3 |  4 |  6 |
    //  +----+----+----+----+

    // CoreHW CHW1010-ANT2-1.1 antenna coordinate system:
    //            Y
    //            |
    //    13   12 | 11    9
    //            |
    //    14   15 | 10    8
    //            +------------ X
    //     1    0    5    7
    //
    //     2    3    4    6
    //
    // Right-handed Cartesian (x, y, z) coordinate system.
    // X-axis: Points rightward when facing the array.
    // Y-axis: Points upward when facing the array.
    // Z-axis: Points outward from the array toward the locator.
    // Origin (0, 0, 0) is at the center of the array.
    // 
    // Azimuth is the angle in the XZ-plane with respect to the Z-axis.
    //   - Positive when the locator is to the right of the array (x > 0).
    //   - Negative when the locator is to the left of the array (x < 0).
    //   - Range is [-pi, pi].
    // Elevation is the angle from the XZ-plane toward the Y-axis.
    //   - Positive when the locator is above the XZ-plane (y > 0).
    //   - Negative when the locator is below the XZ-plane (y < 0).
    //   - Range is [-pi/2, pi/2].
    //
    // The above definition for Azimuth and Elevation is intuitive and
    // conventional, but note that many BLE direction finding references may use
    // an alternative convention:
    // (Alt.) Azimuth is the angle in the XY-plane with respect to the X-axis.
    // (Alt.) Elevation is the angle from the XY-plane toward the Z-axis.

    //float *phases = iq_data->measurement_phases_compensated;

    // Antenna switching sequence for 37 measurement samples.
    // This is for the default sample spacing of 4 microseconds where CTEType
    // field value is 2 for "AoD Constant Tone Extension with 2 μs slots".
    // antenna_switching_sequence[i] maps measurement index i to the antenna
    // number stored in antenna_switching_sequence[i].
    // For example, phases[4] was sampled from antenna 5, because
    // antenna_switching_sequence[4] = 5.
    static const uint8_t antenna_switching_sequence[37] = {
         1,  2,  3,  4,  5,  6,  7,  8,  9, 10,
        11, 12, 13, 14, 15,  0,  1,  2,  3,  4,
         5,  6,  7,  8,  9, 10, 11, 12, 13, 14,
        15,  0,  1,  2,  3,  4,  5
    };
    /* SWITCHPATTERN list, for reference, copied from beacon main.c:
     * 
     * SWITCHPATTERN[0]  = 0x0,  radio.dfe-pdu-antenna,  idle period (PDU Tx/Rx).
     * SWITCHPATTERN[1]  = 0x0,  ant_patterns[0],        guard and reference period.
     * SWITCHPATTERN[2]  = 0x1,  ant_patterns[1],         1st sample slot.
     * SWITCHPATTERN[3]  = 0x2,  ant_patterns[2],         2nd sample slot.
     * SWITCHPATTERN[4]  = 0x3,  ant_patterns[3],         3rd sample slot.
     * SWITCHPATTERN[5]  = 0x4,  ant_patterns[4],         4th sample slot.
     * SWITCHPATTERN[6]  = 0x5,  ant_patterns[5],         5th sample slot.
     * SWITCHPATTERN[7]  = 0x6,  ant_patterns[6],         6th sample slot.
     * SWITCHPATTERN[8]  = 0x7,  ant_patterns[7],         7th sample slot.
     * SWITCHPATTERN[9]  = 0x8,  ant_patterns[8],         8th sample slot.
     * SWITCHPATTERN[10] = 0x9,  ant_patterns[9],         9th sample slot.
     * SWITCHPATTERN[11] = 0xA,  ant_patterns[10],       10th sample slot.
     * SWITCHPATTERN[12] = 0xB,  ant_patterns[11],       11th sample slot.
     * SWITCHPATTERN[13] = 0xC,  ant_patterns[12],       12th sample slot.
     * SWITCHPATTERN[14] = 0xD,  ant_patterns[13],       13th sample slot.
     * SWITCHPATTERN[15] = 0xE,  ant_patterns[14],       14th sample slot.
     * SWITCHPATTERN[16] = 0xF,  ant_patterns[15],       15th sample slot.
     * SWITCHPATTERN[17] = 0x0,  ant_patterns[0],        16th sample slot.
     * SWITCHPATTERN[2]  = 0x1,  ant_patterns[1],        17th sample slot.
     * SWITCHPATTERN[3]  = 0x2,  ant_patterns[2],        18th sample slot.
     * SWITCHPATTERN[4]  = 0x3,  ant_patterns[3],        19th sample slot.
     * SWITCHPATTERN[5]  = 0x4,  ant_patterns[4],        20th sample slot.
     * SWITCHPATTERN[6]  = 0x5,  ant_patterns[5],        21st sample slot.
     * SWITCHPATTERN[7]  = 0x6,  ant_patterns[6],        22nd sample slot.
     * SWITCHPATTERN[8]  = 0x7,  ant_patterns[7],        23rd sample slot.
     * SWITCHPATTERN[9]  = 0x8,  ant_patterns[8],        24th sample slot.
     * SWITCHPATTERN[10] = 0x9,  ant_patterns[9],        25th sample slot.
     * SWITCHPATTERN[11] = 0xA,  ant_patterns[10],       26th sample slot.
     * SWITCHPATTERN[12] = 0xB,  ant_patterns[11],       27th sample slot.
     * SWITCHPATTERN[13] = 0xC,  ant_patterns[12],       28th sample slot.
     * SWITCHPATTERN[14] = 0xD,  ant_patterns[13],       29th sample slot.
     * SWITCHPATTERN[15] = 0xE,  ant_patterns[14],       30th sample slot.
     * SWITCHPATTERN[16] = 0xF,  ant_patterns[15],       31st sample slot.
     * SWITCHPATTERN[17] = 0x0,  ant_patterns[0],        32nd sample slot.
     * SWITCHPATTERN[2]  = 0x1,  ant_patterns[1],        33rd sample slot.
     * SWITCHPATTERN[3]  = 0x2,  ant_patterns[2],        34th sample slot.
     * SWITCHPATTERN[4]  = 0x3,  ant_patterns[3],        35th sample slot.
     * SWITCHPATTERN[5]  = 0x4,  ant_patterns[4],        36th sample slot.
     * SWITCHPATTERN[6]  = 0x5,  ant_patterns[5],        37th sample slot.
     */

    // Selected measurement index pairs for interferometry.
    // These numbers are measurement indices, not antenna numbers. This sequence
    // of index pairs resembles a snake pattern on the CoreHW CHW1010-ANT2-1.1
    // antenna grid. This snake pattern ensures temporally adjacent measurements
    // of physically adjacent antennas. Measurement phases have been compensated
    // for an estimated linear phase drift, but some residual phase drift may
    // still remain in the compensated measurements. This snake pattern aims to
    // minimize the effect of residual phase drift on the calculations by only
    // allowing temporally adjacent measurement pairs. Of the 32 selected pairs,
    // 18 pairs are vertically adjacent (bottom to top, top to bottom), and 14
    // pairs are horizontally adjacent (left to right, right to left).
    // For example, phases[26] and phases[27] make a valid pair, where
    // phases[26] (antenna 11) is to the right of phases[27] (antenna 12).
    // The pair encoding {26, 27, 1} is mathematically equivalent to {27, 26, 0}
    // when computing the phase delta. Both pair encodings represent the same
    // physical relationship.
    // The sign convention for phase delta is positive X and positive Y:
    // delta = phases[left antenna] - phases[right antenna], where a positive
    // phase delta means that the AoD locator is to the right of the origin in
    // the AoD beacon coordinate system.
    // delta = phases[bottom antenna] - phases[top antenna], where a positive
    // phase delta means the AoD locator is above the origin in the AoD beacon
    // coordinate system.
    // Pair direction encoding:
    // 0 = left to right
    // 1 = right to left
    // 2 = bottom to top
    // 3 = top to bottom
    static const struct {
        uint8_t index_1;
        uint8_t index_2;
        uint8_t direction;
    } measurement_pairs[32] = {
        { 0,  1, 3}, // top to bottom
        { 1,  2, 0}, // left to right
        { 2,  3, 0}, // left to right
        { 3,  4, 2}, // bottom to top
        { 5,  6, 2}, // bottom to top
        { 6,  7, 2}, // bottom to top
        { 7,  8, 2}, // bottom to top
        { 9, 10, 2}, // bottom to top
        {10, 11, 1}, // right to left
        {11, 12, 1}, // right to left
        {12, 13, 3}, // top to bottom
        {13, 14, 0}, // left to right
        {14, 15, 3}, // top to bottom
        {15, 16, 1}, // right to left
        {16, 17, 3}, // top to bottom
        {17, 18, 0}, // left to right
        {18, 19, 0}, // left to right
        {19, 20, 2}, // bottom to top
        {21, 22, 2}, // bottom to top
        {22, 23, 2}, // bottom to top
        {23, 24, 2}, // bottom to top
        {25, 26, 2}, // bottom to top
        {26, 27, 1}, // right to left
        {27, 28, 1}, // right to left
        {28, 29, 3}, // top to bottom
        {29, 30, 0}, // left to right
        {30, 31, 3}, // top to bottom
        {31, 32, 1}, // right to left
        {32, 33, 3}, // top to bottom
        {33, 34, 0}, // left to right
        {34, 35, 0}, // left to right
        {35, 36, 2}  // bottom to top
    };

    static const int measurement_pairs_length =
            sizeof(measurement_pairs) / sizeof(measurement_pairs[0]);

    uint8_t measurement_sample_count = iq_data->measurement_sample_count;
    if (measurement_sample_count < 3) {
        iq_data->local_direction_cosine_x = 0.0f;
        iq_data->local_direction_cosine_y = 0.0f;
        iq_data->local_direction_cosine_z = 1.0f;
        iq_data->aod_azimuth = 0.0f;
        iq_data->aod_elevation = 0.0f;
        return;
    }

    // BLE channels 0-36 are secondary advertising channels.
    // BLE channels 37-39 are primary advertising channels.
    uint8_t channel_index = iq_data->channel_index;

    // BLE channel frequency in MHz.
    uint16_t channel_frequency = ble_channel_get_frequency(channel_index);

    // BLE channel wavelength in millimeters.
    // lambda = c/f, where c = 299792458 meters/second, and f is BLE channel
    // frequency in MHz.
    float channel_wavelength = ble_channel_get_wavelength(channel_index);

    // BLE channel wavenumber in radians per millimeter.
    // k = 2*pi/lambda, where pi = 3.1415926535897932384626433832795, and lambda
    // is BLE channel wavelength in millimeters.
    float channel_wavenumber = ble_channel_get_wavenumber(channel_index);

    // CoreHW CHW1010-ANT2-1.1 antenna spacing for orthogonally adjacent
    // antennas, from antenna center to antenna center, in radians, at the BLE
    // channel frequency.
    // BLE channel wavenumber multiplied by orthogonal antenna spacing.
    // d_orth_rad = k * antenna_spacing_orthogonal, in radians.
    // For BLE channel index 18 (2442 Mhz):
    // d_orth_rad = 0.051181 rad/mm * 37.5mm = 1.91928750 rad.
    float d_orth_rad = channel_wavenumber * antenna_spacing_orthogonal;

    // By default, the interval between samples in the measurement period is 4
    // microseconds and the CTE frequency is 250 kilohertz. This is exactly 1
    // CTE cycle because (0.25 * 1000000) * (4 / 1000000) = 1. It is effectively
    // as if all 37 measurement samples are taken at the same time. If the
    // measurement samples are compensated for systematic linear phase drift,
    // then any remaining phase differences must be due to signal direction and
    // antenna positions. This enables conventional interferometry using first
    // differences, Delta(φ)[m] = φ[m] - φ[m-1], effectively emulating the
    // behavior of a conventional interferometer array.

    // TODO(wathne): Explain that AoD and AoA calculations are symmetric.
    // TODO(wathne): Explain how phase shifts at 250 kHz and 2.4 GHz are
    // related.

    // A note about the unit circle:
    // In mathematics, the unit circle range is (-pi, pi], where negative pi
    // is not included in the range. In the C math function atan2f(), the unit
    // circle range is [-pi, pi], where negative pi is included in the range.
    // Negative pi represents the case when the negative x-axis boundary
    // (+/- pi) is approached from below. This design choice is presumably
    // related to the limited precision of floating point numbers, and it
    // provides continuity when crossing the boundary.

    // The usual arithmetic mean is not appropriate for calculating the mean on
    // a unit circle with range (-pi, pi] or range [-pi, pi]. To see how the
    // arithmetic mean breaks down, consider the case where two angles, +0.9*pi
    // and -0.9*pi, are clustered around the negative x-axis. Their circular
    // mean is obviously +/- pi, which is exactly on the negative x-axis. Their
    // arithmetic mean is 0, which is exactly on the positive x-axis. The
    // arithmetic mean is 180 degrees off from the circular mean in this simple
    // example. Another approach is clearly necessary. There are two main
    // approaches for calculating a circular mean, either an extrinsic mean or
    // an intrinsic mean. The extrinsic mean is the simplest approach.

    // Extrinsic mean on a circle:
    // Put the circle in the two-dimensional Euclidean space (R^2), mapping
    // phase angles to (cos(φ), sin(φ)) coordinates. Calculate the average of
    // the (cos(φ), sin(φ)) coordinates. This average will be a point inside the
    // unit circle, but not on the unit circle itself. Finally, project the
    // point onto the unit circle using the atan2f() function. This extrinsic
    // mean is a good approximation of the true mean if the phase angles are
    // clustered.

    // Intrinsic mean on a circle:
    // TODO(wathne): Explain intrinsic circular mean. For now, see
    // "directional_statistics.h" and "directional_statistics.h".

    // TODO(wathne): Add more documentation.

    // First difference:
    // Delta(φ)[m] = φ[m] - φ[m-1]
    float delta;

    float horizontal_deltas[measurement_pairs_length];
    int horizontal_count = 0;
    float horizontal_mean = 0.0f;

    float vertical_deltas[measurement_pairs_length];
    int vertical_count = 0;
    float vertical_mean = 0.0f;

    for (int i = 0; i < measurement_pairs_length; i++) {
        uint8_t index_1 = measurement_pairs[i].index_1;
        uint8_t index_2 = measurement_pairs[i].index_2;
        uint8_t direction = measurement_pairs[i].direction;

        // Check if indices are out of bounds.
        if (index_1 >= measurement_sample_count ||
                index_2 >= measurement_sample_count) {
            continue;
        }

        float i1 = iq_data->measurement_i_compensated[index_1];
        float q1 = iq_data->measurement_q_compensated[index_1];
        float i2 = iq_data->measurement_i_compensated[index_2];
        float q2 = iq_data->measurement_q_compensated[index_2];

        float real_part = i1*i2 + q1*q2;
        float imag_part = q1*i2 - i1*q2;

        delta = atan2f(imag_part, real_part);

        // TODO(wathne): Remove this limit? Allow more than theoretical max?
        // Clamp delta if delta is greater than theoretical max, ~ 1.9 radians.
        // For BLE channel index 18 (2442 Mhz):
        // d_orth_rad = 0.051181 rad/mm * 37.5mm = 1.91928750 rad.
        if (delta > d_orth_rad) {
            // TODO(wathne): Remove this line.
            printk("Pair %d {%d, %d, %d}: CLAMPING %.6f to %.6f\n",
                    i, index_1, index_2, direction, delta, d_orth_rad);
            delta = d_orth_rad;
        } else if (delta < -d_orth_rad) {
            // TODO(wathne): Remove this line.
            printk("Pair %d {%d, %d, %d}: CLAMPING %.6f to %.6f\n",
                    i, index_1, index_2, direction, delta, -d_orth_rad);
            delta = -d_orth_rad;
        }

        // Pair direction decoding:
        // 0 = left to right
        // 1 = right to left
        // 2 = bottom to top
        // 3 = top to bottom
        switch (direction) {
            case 0:
                horizontal_deltas[horizontal_count] = delta;
                horizontal_count = horizontal_count + 1;
                break;
            case 1:
                horizontal_deltas[horizontal_count] = -delta;
                horizontal_count = horizontal_count + 1;
                break;
            case 2:
                vertical_deltas[vertical_count] = delta;
                vertical_count = vertical_count + 1;
                break;
            case 3:
                vertical_deltas[vertical_count] = -delta;
                vertical_count = vertical_count + 1;
                break;
        }
    }

    // Search for the intrinsic circular mean for horizontal deltas.
    if (horizontal_count > 0) {
        horizontal_mean = directional_statistics_circular_mean(
                horizontal_deltas,
                horizontal_count,
                5,
                0.01);
    }

    // Search for the intrinsic circular mean for vertical deltas.
    if (vertical_count > 0) {
        vertical_mean = directional_statistics_circular_mean(
                vertical_deltas,
                vertical_count,
                5,
                0.01);
    }

    float direction_cosine_x = 0.0f;
    if (horizontal_count > 0) {
        direction_cosine_x = -horizontal_mean / d_orth_rad;

        // Clamp to [-1, 1].
        if (direction_cosine_x > 1.0f) {
            direction_cosine_x = 1.0f;
        }
        if (direction_cosine_x < -1.0f) {
            direction_cosine_x = -1.0f;
        }
    }

    float direction_cosine_y = 0.0f;
    if (vertical_count > 0) {
        direction_cosine_y = -vertical_mean / d_orth_rad;

        // Clamp to [-1, 1].
        if (direction_cosine_y > 1.0f) {
            direction_cosine_y = 1.0f;
        }
        if (direction_cosine_y < -1.0f) {
            direction_cosine_y = -1.0f;
        }
    }

    // Calculate direction_cosine_z from the direction cosine relationship
    // cos^2(θx) + cos^2(θy) + cos^2(θz) = 1
    float direction_cosine_z_squared = 1.0f - (
            direction_cosine_x*direction_cosine_x +
            direction_cosine_y*direction_cosine_y);

    if (direction_cosine_z_squared < 0.0f) {
        direction_cosine_z_squared = 0.0f;
    }

    float direction_cosine_z = sqrtf(direction_cosine_z_squared);

    iq_data->local_direction_cosine_x = direction_cosine_x;
    iq_data->local_direction_cosine_y = direction_cosine_y;
    iq_data->local_direction_cosine_z = direction_cosine_z;

    iq_data->aod_azimuth = atan2f(direction_cosine_x, direction_cosine_z);
    iq_data->aod_elevation = asinf(direction_cosine_y);

    // TODO(wathne): Remove this line.
    printk("azimuth:   %.2f\n", iq_data->aod_azimuth);
    // TODO(wathne): Remove this line.
    printk("elevation: %.2f\n", iq_data->aod_elevation);
}

// Test an IQ data structure.
// The iq_data argument must be a pointer to an initialized IQ data structure.
// See the iq_data_init() function.
static void test_iq_data(struct iq_data *iq_data) {
    if (!iq_data || !iq_data->initialized) {
        return;
    }
    float in_phase;
    float quadrature;
    float phasor_amplitude;
    float phasor_radians;
    float phasor_degrees;

    for (int i = 0; i < iq_data->reference_sample_count; i++) {
        in_phase = iq_data->reference_i[i];
        quadrature = iq_data->reference_q[i];
        phasor_amplitude = sqrtf(in_phase*in_phase + quadrature*quadrature);
        phasor_radians = iq_data->reference_phases_unwrapped[i];
        phasor_degrees = phasor_radians * IQ_DATA_DEGREES_RADIANS_RATIO;
        printk("ref-unw[%d]: (%.2f, %.2f), amp %.2f, rad %.2f, deg %.2f,\n",
                i, in_phase, quadrature, phasor_amplitude, phasor_radians,
                phasor_degrees);
    }

    for (int i = 0; i < iq_data->measurement_sample_count; i++) {
        in_phase = iq_data->measurement_i_compensated[i];
        quadrature = iq_data->measurement_q_compensated[i];
        phasor_amplitude = sqrtf(in_phase*in_phase + quadrature*quadrature);
        phasor_radians = iq_data->measurement_phases_compensated[i];
        phasor_degrees = phasor_radians * IQ_DATA_DEGREES_RADIANS_RATIO;
        printk("msr-cmp[%d]: (%.2f, %.2f), amp %.2f, rad %.2f, deg %.2f,\n",
                i, in_phase, quadrature, phasor_amplitude, phasor_radians,
                phasor_degrees);
    }
}

// Test floating point support for an IQ data structure.
static bool test_float_support_first_callback_completed = false;

// Test floating point support for an IQ data structure.
// The iq_data argument must be a pointer to an initialized IQ data structure.
// See the iq_data_init() function.
static void test_float_support(struct iq_data *iq_data) {
    if (!iq_data || !iq_data->initialized) {
        return;
    }

    if (!test_float_support_first_callback_completed) {
        printk("TFS: Test Float Support, first callback start.\n");

        // int8_t array access.
        int8_t i_int8 = iq_data->measurement_i[0];
        int8_t q_int8 = iq_data->measurement_q[0];
        printk("TFS: int8_t array access.\n");
        printk("TFS: i_int8 = %d\n", i_int8);
        printk("TFS: q_int8 = %d\n", q_int8);

        // int8_t array access and int8_t to float conversion.
        float i_float = (float)iq_data->measurement_i[0];
        float q_float = (float)iq_data->measurement_q[0];
        printk("TFS: int8_t array access and int8_t to float conversion.\n");
        printk("TFS: i_float = %f\n", i_float);
        printk("TFS: q_float = %f\n", q_float);

        // atan2f() float calculation.
        float atan_float = atan2f(q_float, i_float);
        printk("TFS: atan2f() float calculation.\n");
        printk("TFS: atan2f(q_float, i_float) = %f\n", atan_float);

        // Store float in float array.
        iq_data->measurement_phases[0] = atan_float;
        printk("TFS: Store float in float array.\n");

        test_float_support_first_callback_completed = true;
        printk("TFS: Test Float Support, first callback completed.\n");
    } else {
        printk("TFS: Test Float Support, second callback start.\n");

        printk("TFS: First callback must have completed.");

        // float array access to stored float.
        float stored_float = iq_data->measurement_phases[0];
        printk("TFS: float array access to stored float.\n");
        printk("TFS: stored_float = %f\n", stored_float);

        test_float_support_first_callback_completed = false;
        printk("TFS: Test Float Support, second callback completed.\n");
    }
}

// TODO(wathne): Make a better system. This is temporary.
static struct iq_data previous_iq_data;
static bool first_iteration = true;

void iq_data_process(const struct iq_raw_samples *iq_raw_samples) {
    struct iq_data iq_data;

    // Initialize the IQ data structure from the raw IQ samples structure.
    iq_data_init(&iq_data, iq_raw_samples);

    // TODO(wathne): Why is there a systematic intersample phase shift of 180
    // degrees between samples in the reference period? There is conflicting
    // information about the expected intersample phase shifts in the reference
    // period. The intersample phase shifts should be about 90 degrees for 1 μs
    // intervals at 250 kHz (CTE), but there are forum posts hinting at 360 (0)
    // degrees for 1 μs intervals at 1000 kHz, which does not make sense. The
    // CTE is supposed to always be 250 kHz. Despite this nonsense, the current
    // assumption of 1000 kHz and 360 (0) degrees remains for the simple reason
    // that this seems to net good estimates for the systematic linear phase
    // drift if a temporary fix is applied to every other reference sample. This
    // issue should be revisited, but the temporary fix works for now.
    iq_data_temp_fix_ref_samples(&iq_data);

    // NOTE(wathne): Reference samples are not intended to be used directly in
    // Angle of Departure estimations. If we wanted to include the 8th (last)
    // reference sample in the Angle of Departure estimations, then we would
    // have to account for the special intersample delay between the last
    // reference sample and the first measurement sample.
    // See the Nordic Semiconductor whitepaper nwp_036.pdf, page 13.
    // Note that the iq_data_temp_fix_ref_samples() function applies a rotation
    // to reference samples of index 1, 3, 5 and 7. The rotation from this
    // temporary fix would also have to be accounted for because index 7 points
    // to the 8th (last) reference sample.

    // Estimate linear phase drift rate for the IQ data structure.
    // Set linear_phase_drift_rate to the estimated rate of radians per
    // microsecond.
    // reference_phases[] and reference_phases_unwrapped[] are also populated.
    estimate_linear_phase_drift_rate(&iq_data);

    // Compensate for linear phase drift in measurement samples.
    // Populate measurement_i_compensated[] and measurement_q_compensated[] with
    // measurement samples compensated at the estimated linear phase drift rate.
    compensate_measurement_samples(&iq_data);

    // Calculate compensated measurement phases.
    // Populate measurement_phases_compensated[] with measurement phase angles
    // compensated at the estimated linear phase drift rate.
    //calculate_compensated_measurement_phases(&iq_data);

    // Estimate local direction cosines, azimuth, and elevation.
    iq_data_aod_interferometry(&iq_data);
    //iq_data_aod_row_interferometry(&iq_data);

    // TODO(wathne): Make a better system. This is temporary.
    if (first_iteration) {
        previous_iq_data = iq_data;
        first_iteration = false;
        return;
    }

    // TODO(wathne): Make a better system. This is temporary.
    int ret = bt_addr_mac_compare(
            iq_data.beacon_mac,
            previous_iq_data.beacon_mac);
    if (ret == 1) {
        printk("DEBUG: same mac, no pair\n");
    } else {
        printk("DEBUG: new mac, have pair\n");
        ret = locator_estimate_position_from_skew_lines(
            &g_locator,
            previous_iq_data.beacon_mac,
            previous_iq_data.local_direction_cosine_x,
            previous_iq_data.local_direction_cosine_y,
            previous_iq_data.local_direction_cosine_z,
            iq_data.beacon_mac,
            iq_data.local_direction_cosine_x,
            iq_data.local_direction_cosine_y,
            iq_data.local_direction_cosine_z);
        if (ret == 0) {
            printk("DEBUG: position success\n");
        } else if (ret == -LOCATOR_ERROR_PARALLEL_LINES) {
            printk("DEBUG: position fail, parallel lines\n");
        } else {
            printk("DEBUG: position fail\n");
        }
        previous_iq_data = iq_data;
    }
}