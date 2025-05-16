#ifndef IQ_DATA_WORK_QUEUE_H
#define IQ_DATA_WORK_QUEUE_H

#include <zephyr/kernel.h> // For work structure and work queue structure.
#include <zephyr/spinlock.h> // For spinlock structure.
#include "iq_data.h" // For raw IQ samples structure.

// TODO(wathne): Make the IQ data work queue aware of beacon MAC addresses.
// TODO(wathne): Replace the strict LIFO processing with more intelligent
// processing. Try to alternate, serving raw IQ samples from different beacon
// MAC addresses while also prioritizing recency. FIFO eviction can still be
// viable.

// IQ data work queue capacity.
#define IQ_DATA_WORK_QUEUE_CAPACITY 8

// Function pointer type for processing a raw IQ samples structure.
typedef void (*iq_raw_samples_processor_t)(
        const struct iq_raw_samples *iq_raw_samples);

// IQ data work queue structure.
// LIFO processing and FIFO eviction.
// TODO(wathne): Add more description.
struct iq_data_work_queue {
    // Ring buffer for raw IQ samples structures, constrained by
    // IQ_DATA_WORK_QUEUE_CAPACITY.
    struct iq_raw_samples buffer[IQ_DATA_WORK_QUEUE_CAPACITY];

    // Function for processing a buffered raw IQ samples structure.
    iq_raw_samples_processor_t processor;

    // Queue state: head, index of the newest element in the buffer.
    int head;
    // Queue state: tail, index of the oldest element in the buffer.
    int tail;
    // Queue state: count, number of elements in the buffer.
    int count;
    // Spinlock to ensure atomic access to queue state variables.
    struct k_spinlock lock;

    // Work structure for submitting processor work to the target work queue.
    struct k_work processor_work;
    // Target work queue structure. For example, the system work queue
    // (&k_sys_work_q) is a reasonable default choice as the target work queue.
    struct k_work_q *target_work_queue;
};

// TODO(wathne): Add description.
void iq_data_work_queue_init(
        struct iq_data_work_queue *iq_data_work_queue,
        struct k_work_q *target_work_queue,
        iq_raw_samples_processor_t processor);

// TODO(wathne): Add description.
void iq_data_work_queue_submit(
        struct iq_data_work_queue *iq_data_work_queue,
        const struct iq_raw_samples *iq_raw_samples);

#endif // IQ_DATA_WORK_QUEUE_H