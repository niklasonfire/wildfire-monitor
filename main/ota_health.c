#include "ota_health.h"

#include <inttypes.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ota";

#define WATCH_MS    1000
#define WATCH_STACK 3072
#define WATCH_PRIO  2               /* below the UI and the capture writer */

/* The gates are set by three different tasks - main, the capture store's
 * caller and the redraw task - so the read-modify-write behind them is the one
 * thing in here that needs guarding. A critical section rather than a mutex:
 * it is two instructions, and ota_health_pass() must stay callable from
 * anywhere that has just finished doing something useful. */
static portMUX_TYPE      s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_gates;

static bool          s_probation;
static volatile bool s_confirmed;
static bool          s_rolled_back;
static const char   *s_running  = "unknown";
static const char   *s_boot     = "unknown";
static const char   *s_rollback = "none";

static const char *label_of(const esp_partition_t *p)
{
    return (p != NULL) ? p->label : "unknown";
}

/* The gates still outstanding, as a printable list. Only ever used for the one
 * complaint this module makes, so it names subsystems the way the rest of the
 * log does rather than printing a bitmask nobody can read on a serial line. */
static void missing_gates(char *out, size_t cap)
{
    uint32_t missing = OTA_GATES_ALL & ~ota_health_gates();

    snprintf(out, cap, "%s%s%s",
             (missing & OTA_GATE_NVS)     ? " nvs" : "",
             (missing & OTA_GATE_STORE)   ? " store" : "",
             (missing & OTA_GATE_DISPLAY) ? " display" : "");
}

/* Runs until the image has either earned its place or provably cannot, then
 * exits: there is nothing to watch afterwards, and a task that has finished
 * its one job should not hold a stack for the rest of the ride. */
static void watch_task(void *arg)
{
    bool warned = false;

    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(WATCH_MS));

        int64_t up_s = esp_timer_get_time() / 1000000;

        if ((ota_health_gates() & OTA_GATES_ALL) != OTA_GATES_ALL) {
            /* Said once, and only after the deadline has already gone by, so
             * that the serial log names the subsystem that is about to cost a
             * rollback instead of leaving the next person to guess at it. */
            if (!warned && up_s >= OTA_HEALTH_UPTIME_S) {
                char miss[32];
                missing_gates(miss, sizeof(miss));
                ESP_LOGE(TAG, "health check stalled after %" PRId64 " s, missing:%s%s",
                         up_s, miss,
                         s_probation ? " - the next reset will roll back"
                                     : " (not on probation, so nothing rolls back)");
                warned = true;
            }
            continue;
        }
        if (up_s < OTA_HEALTH_UPTIME_S) {
            continue;
        }

        if (!s_probation) {
            /* Nothing in otadata to change. Logged all the same: this is the
             * line that shows the check ran on a USB-flashed board. */
            ESP_LOGI(TAG, "%s healthy after %" PRId64 " s, nothing to confirm",
                     s_running, up_s);
            s_confirmed = true;
            break;
        }

        esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        if (err != ESP_OK) {
            /* otadata would not take the write, and retrying a flash that just
             * refused one is not a plan. Leaving it is the safe end of the
             * failure: the image stays pending and the bootloader undoes it. */
            ESP_LOGE(TAG, "could not confirm %s: %s - the next reset will roll back",
                     s_running, esp_err_to_name(err));
            break;
        }
        ESP_LOGI(TAG, "%s confirmed after %" PRId64 " s, rollback cancelled",
                 s_running, up_s);
        s_confirmed = true;
        break;
    }
    vTaskDelete(NULL);
}

void ota_health_start(void)
{
    const esp_partition_t *run  = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();

    s_running = label_of(run);
    s_boot    = label_of(boot);

    /* PENDING_VERIFY is the only state that means probation. A USB flash
     * leaves otadata erased, which reads back as UNDEFINED and boots the first
     * app slot - so a board on the bench is never on probation, and must not
     * be treated as though a rollback were waiting for it. */
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    if (run != NULL && esp_ota_get_state_partition(run, &state) == ESP_OK) {
        s_probation = (state == ESP_OTA_IMG_PENDING_VERIFY);
    }

    ESP_LOGI(TAG, "running from %s at 0x%06" PRIx32 ", %s",
             s_running, (run != NULL) ? run->address : 0u,
             s_probation ? "on probation" : "confirmed");

    /* The other slot, which is where a rollback leaves its evidence: the
     * bootloader marks an image it gave up on INVALID or ABORTED and comes
     * back here. A board that has only ever been flashed over USB has no
     * otadata entry for that slot at all, so the read fails and this stays
     * false - which is what keeps a bench board from claiming a rollback that
     * never happened. */
    const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);
    esp_ota_img_states_t   ostate = ESP_OTA_IMG_UNDEFINED;
    if (other != NULL && esp_ota_get_state_partition(other, &ostate) == ESP_OK &&
        (ostate == ESP_OTA_IMG_INVALID || ostate == ESP_OTA_IMG_ABORTED)) {
        s_rolled_back = true;
        s_rollback    = label_of(other);
        ESP_LOGW(TAG, "%s holds an image the bootloader gave up on - "
                      "the last update was rolled back and %s is what came up",
                 s_rollback, s_running);
    }

    xTaskCreate(watch_task, "otahealth", WATCH_STACK, NULL, WATCH_PRIO, NULL);
}

void ota_health_pass(ota_gate_t gate)
{
    portENTER_CRITICAL_SAFE(&s_lock);
    s_gates |= (uint32_t)gate;
    portEXIT_CRITICAL_SAFE(&s_lock);
}

const char *ota_running_label(void) { return s_running; }
const char *ota_boot_label(void)    { return s_boot; }
bool     ota_health_on_probation(void) { return s_probation; }
uint32_t ota_health_gates(void)        { return s_gates; }
bool     ota_health_confirmed(void)    { return s_confirmed; }
bool        ota_health_rolled_back(void) { return s_rolled_back; }
const char *ota_rollback_label(void)     { return s_rollback; }
