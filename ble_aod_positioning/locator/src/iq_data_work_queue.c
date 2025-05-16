#include "iq_data_work_queue.h" // For IQ data work queue structure, iq_raw_samples_processor_t, and IQ_DATA_WORK_QUEUE_CAPACITY.
#include <stdbool.h> // For bool.
#include <stddef.h> // For NULL ((void *)0).
#include <string.h> // For memcpy().
#include <zephyr/kernel.h> // For work structure, work queue structure, k_work_init(), and k_work_submit_to_queue().
#include <zephyr/spinlock.h> // For k_spinlock_key_t, k_spin_lock(), and k_spin_unlock().
#include <zephyr/sys/util.h> // For CONTAINER_OF() macro.
#include "iq_data.h" // For raw IQ samples structure.

// TODO(wathne): Make the IQ data work queue aware of beacon MAC addresses.
// TODO(wathne): Replace the strict LIFO processing with more intelligent
// processing. Try to alternate, serving raw IQ samples from different beacon
// MAC addresses while also prioritizing recency. FIFO eviction can still be
// viable.

// TODO(wathne): Add description.
static void iq_data_work_queue_handler(struct k_work *proc_work) {
    // Get the pointer to the IQ data work queue containing the processor work.
    struct iq_data_work_queue *queue = CONTAINER_OF(
            proc_work,
            struct iq_data_work_queue,
            processor_work);

    struct iq_raw_samples current_item;
    bool current_item_extracted;
    bool queue_exhausted;

    while (true) {
        current_item_extracted = false;

        // Ensure atomic access to queue state variables.
        k_spinlock_key_t key = k_spin_lock(&queue->lock);

        if (queue->count > 0) {
            memcpy(
                    &current_item,
                    &queue->buffer[queue->head],
                    sizeof(struct iq_raw_samples));

            // Decrement head by 1 or wrap around.
            if (queue->head == 0) {
                // Wrap around.
                queue->head = IQ_DATA_WORK_QUEUE_CAPACITY - 1;
            } else {
                // Decrement head by 1.
                queue->head--;
            }
            // Decrement count by 1.
            queue->count--;

            current_item_extracted = true;
        }

        if (queue->count == 0) {
            queue_exhausted = true;
        } else {
            queue_exhausted = false;
        }

        k_spin_unlock(&queue->lock, key);

        // Process the extracted item (raw IQ samples).
        if (current_item_extracted && queue->processor != NULL) {
            queue->processor(&current_item);
        }

        if (queue_exhausted) {
            break;
        }
    }

    // Ensure atomic access to queue state variables.
    k_spinlock_key_t key = k_spin_lock(&queue->lock);

    // Check if new items arrived during processing.
    bool start;
    if (queue->count > 0) {
        start = true;
    } else {
        start = false;
    }

    k_spin_unlock(&queue->lock, key);

    if (start && queue->target_work_queue != NULL) {
        k_work_submit_to_queue(
                queue->target_work_queue,
                &queue->processor_work);
    }
}

void iq_data_work_queue_init(
        struct iq_data_work_queue *iq_data_work_queue,
        struct k_work_q *target_work_queue,
        iq_raw_samples_processor_t processor) {
    if (iq_data_work_queue == NULL || target_work_queue == NULL ||
            processor == NULL) {
        return;
    }

    // Ensure atomic access to queue state variables.
    k_spinlock_key_t key = k_spin_lock(&iq_data_work_queue->lock);

    iq_data_work_queue->head = 0;
    iq_data_work_queue->tail = 0;
    iq_data_work_queue->count = 0;

    k_spin_unlock(&iq_data_work_queue->lock, key);

    iq_data_work_queue->target_work_queue = target_work_queue;
    iq_data_work_queue->processor = processor;

    k_work_init(
            &iq_data_work_queue->processor_work,
            iq_data_work_queue_handler);
}

void iq_data_work_queue_submit(
        struct iq_data_work_queue *iq_data_work_queue,
        const struct iq_raw_samples *iq_raw_samples) {
    if (iq_data_work_queue == NULL || iq_raw_samples == NULL) {
        return;
    }

    // Ensure atomic access to queue state variables.
    k_spinlock_key_t key = k_spin_lock(&iq_data_work_queue->lock);

    if (iq_data_work_queue->count == IQ_DATA_WORK_QUEUE_CAPACITY) {
        // Increment tail by 1 or wrap around.
        iq_data_work_queue->tail =
                (iq_data_work_queue->tail + 1) % IQ_DATA_WORK_QUEUE_CAPACITY;
        // Increment head by 1 or wrap around.
        iq_data_work_queue->head =
                (iq_data_work_queue->head + 1) % IQ_DATA_WORK_QUEUE_CAPACITY;
    } else {
        // Increment head by 1 or wrap around.
        iq_data_work_queue->head =
                (iq_data_work_queue->head + 1) % IQ_DATA_WORK_QUEUE_CAPACITY;
        // Increment count by 1.
        iq_data_work_queue->count++;
    }

    memcpy(
            &iq_data_work_queue->buffer[iq_data_work_queue->head],
            iq_raw_samples,
            sizeof(struct iq_raw_samples));

    bool start;
    if (iq_data_work_queue->count == 1) {
        start = true;
    } else {
        start = false;
    }

    k_spin_unlock(&iq_data_work_queue->lock, key);

    if (start && iq_data_work_queue->target_work_queue != NULL) {
        k_work_submit_to_queue(
                iq_data_work_queue->target_work_queue,
                &iq_data_work_queue->processor_work);
    }
}