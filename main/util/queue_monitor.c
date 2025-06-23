#include "util/queue_monitor.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "QUEUE_MON";

#define MAX_MONITORED_QUEUES 10

static queue_stats_t monitored_queues[MAX_MONITORED_QUEUES];
static int queue_count = 0;

esp_err_t queue_monitor_init(void) {
    memset(monitored_queues, 0, sizeof(monitored_queues));
    queue_count = 0;
    ESP_LOGI(TAG, "Queue monitor initialized");
    return ESP_OK;
}

esp_err_t queue_monitor_register(const char* name, QueueHandle_t queue, UBaseType_t max_items) {
    if (queue_count >= MAX_MONITORED_QUEUES) {
        ESP_LOGE(TAG, "Too many queues registered");
        return ESP_ERR_NO_MEM;
    }
    
    queue_stats_t* stats = &monitored_queues[queue_count++];
    stats->name = name;
    stats->handle = queue;
    stats->max_items = max_items;
    stats->current_items = 0;
    stats->peak_items = 0;
    stats->overflow_count = 0;
    stats->total_sends = 0;
    stats->failed_sends = 0;
    stats->health = QUEUE_HEALTH_GOOD;
    
    ESP_LOGI(TAG, "Registered queue: %s (max: %u items)", name, (unsigned int)max_items);
    return ESP_OK;
}

void queue_monitor_update(void) {
    for (int i = 0; i < queue_count; i++) {
        queue_stats_t* stats = &monitored_queues[i];
        
        stats->current_items = uxQueueMessagesWaiting(stats->handle);
        
        if (stats->current_items > stats->peak_items) {
            stats->peak_items = stats->current_items;
        }
        
        // Update health status
        float usage = (float)stats->current_items / stats->max_items;
        if (usage >= 1.0f) {
            stats->health = QUEUE_HEALTH_FULL;
        } else if (usage >= 0.95f) {
            stats->health = QUEUE_HEALTH_CRITICAL;
        } else if (usage >= 0.8f) {
            stats->health = QUEUE_HEALTH_WARNING;
        } else {
            stats->health = QUEUE_HEALTH_GOOD;
        }
    }
}

queue_stats_t* queue_monitor_get_stats(const char* name) {
    for (int i = 0; i < queue_count; i++) {
        if (strcmp(monitored_queues[i].name, name) == 0) {
            return &monitored_queues[i];
        }
    }
    return NULL;
}

void queue_monitor_print_all(void) {
    ESP_LOGI(TAG, "=== QUEUE STATISTICS ===");
    for (int i = 0; i < queue_count; i++) {
        queue_stats_t* stats = &monitored_queues[i];
        const char* health_str[] = {"GOOD", "WARNING", "CRITICAL", "FULL"};
        
        ESP_LOGI(TAG, "%s: %u/%u items (%.1f%%) [%s]", 
                 stats->name,
                 (unsigned int)stats->current_items,
                 (unsigned int)stats->max_items,
                 (float)stats->current_items / stats->max_items * 100.0f,
                 health_str[stats->health]);
    }
    ESP_LOGI(TAG, "========================");
}