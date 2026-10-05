/*
 * Phase 3 — test de capture I2S independant du BLE.
 *
 * Objectif (voir docs/plan_implementation.md Phase 3) : verifier que le
 * cablage ADC PCM1808 <-> ESP32-S3 Supermini (MCLK=GPIO4, BCLK=GPIO5,
 * LRCK=GPIO6, DATA=GPIO7 - voir plan_implementation.md Phase 3 pour le
 * detail et la justification du choix de ces GPIO) transporte un signal
 * audio propre, SANS toucher au BLE/ASHA (code separe de firmware/, qui
 * reste la version validee Phase 2).
 *
 * Horloge : le breakout PCM1808 a son propre quartz 24,576 MHz et ses
 * options en mode MAITRE : c'est lui qui genere BCK et LRCK. Reglage d'usine
 * = maitre 96 kHz (BCK 6,144 MHz) ; OP2 court-circuite = maitre 48 kHz
 * (BCK 3,072 MHz), reglage utilise ici. L'ESP32 est donc ESCLAVE I2S : BCK et
 * LRCK en entrees, MCLK inutilise (le piloter ou piloter BCK/LRCK creait un
 * conflit de sorties sur les memes fils). Indices au scope : DATA changeait
 * sur une grille de 163 ns = 1/6,144 MHz quel que soit le MCLK de l'ESP32.
 * La decimation 48 -> 16 kHz se fera en logiciel.
 *
 * Format : Philips I2S standard, stereo, 32 bits par slot. Le PCM1808 sort
 * un echantillon de 24 bits cale a gauche (MSB first) dans chaque slot de
 * 32 BCLK - on recupere donc la valeur signee en decalant le mot brut de 8
 * bits vers la droite (>> 8, decalage arithmetique, conserve le signe).
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

static const char *TAG = "i2s_test";

#define I2S_SAMPLE_RATE_HZ   48000
#define I2S_BCLK_GPIO        GPIO_NUM_5
#define I2S_WS_GPIO          GPIO_NUM_6
#define I2S_DIN_GPIO         GPIO_NUM_7

/* 256 echantillons par lecture (~5,3 ms a 48 kHz), niveau affiche toutes
 * les 94 lectures (~0,5 s). */
#define FRAMES_PER_READ      256
#define PRINT_PERIOD_BLOCKS  94

/* Pleine echelle d'un echantillon 24 bits signe (apres le >>8). */
#define FULL_SCALE_24BIT     8388607.0

static void
i2s_capture_task(void *arg)
{
    i2s_chan_handle_t rx_chan = NULL;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_SLAVE);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = I2S_SAMPLE_RATE_HZ,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            .bclk_div = 8,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                         I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_GPIO,
            .ws = I2S_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_DIN_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    ESP_LOGI(TAG, "Capture I2S esclave demarree (Fs attendue=%d Hz, GPIO bclk=%d ws=%d din=%d)",
             I2S_SAMPLE_RATE_HZ, I2S_BCLK_GPIO, I2S_WS_GPIO, I2S_DIN_GPIO);

    /* static plutot que sur la pile : evite de cumuler ce buffer (2 Ko) avec
     * la profondeur d'appel de l'init I2S + printf - un premier essai avec
     * une pile de tache de 4096 octets et ce buffer en automatique a
     * provoque un stack overflow qui a corrompu le tas (crash retarde et
     * sans rapport apparent, dans le idle task watchdog). */
    static int32_t raw[FRAMES_PER_READ * 2]; /* stereo : L,R,L,R,... */

    int64_t sum_sq = 0;
    int32_t peak = 0;
    int32_t min_val = INT32_MAX;
    int32_t max_val = INT32_MIN;
    uint32_t clip_count = 0; /* echantillons a moins de 1% de la pleine echelle */
    uint32_t total_frames = 0;
    int block_count = 0;

    while (1) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_channel_read(rx_chan, raw, sizeof(raw), &bytes_read, 1000);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2s_channel_read: erreur %d (timeout ou surcharge)", err);
            continue;
        }

        size_t frames = bytes_read / (2 * sizeof(int32_t));

        /* Dump brut des 8 premiers echantillons de CE bloc (L,R,L,R...), a
         * chaque periode d'affichage (~0.5s) et pas seulement au boot - pour
         * voir si la forme d'onde evolue de facon credible (variation lente,
         * passages par zero) plutot que du bruit numerique incoherent. */
        if (block_count == 0) {
            ESP_LOGI(TAG, "mots bruts de ce bloc (L,R,L,R...) :");
            for (size_t i = 0; i < 8 && i < frames * 2; i++) {
                printf("  raw[%u] = 0x%08" PRIx32 "\n", (unsigned)i, (uint32_t)raw[i]);
            }
        }

        for (size_t i = 0; i < frames; i++) {
            int32_t left = raw[2 * i] >> 8;
            int32_t right = raw[2 * i + 1] >> 8;
            int32_t mono = (left + right) / 2;

            int32_t abs_mono = mono < 0 ? -mono : mono;
            if (abs_mono > peak) {
                peak = abs_mono;
            }
            if (mono < min_val) {
                min_val = mono;
            }
            if (mono > max_val) {
                max_val = mono;
            }
            if (abs_mono > (int32_t)(FULL_SCALE_24BIT * 0.99)) {
                clip_count++;
            }
            sum_sq += (int64_t)mono * (int64_t)mono;
        }
        total_frames += frames;
        block_count++;

        if (block_count >= PRINT_PERIOD_BLOCKS) {
            double rms = (total_frames > 0) ? sqrt((double)sum_sq / (double)total_frames) : 0.0;
            double peak_dbfs = (peak > 0) ? 20.0 * log10(peak / FULL_SCALE_24BIT) : -INFINITY;
            double rms_dbfs = (rms > 0.0) ? 20.0 * log10(rms / FULL_SCALE_24BIT) : -INFINITY;

            ESP_LOGI(TAG, "niveau mono : peak=%" PRId32 " (%.1f dBFS)  rms=%.0f (%.1f dBFS)  min=%" PRId32
                      "  max=%" PRId32 "  clip=%" PRIu32 "/%u",
                      peak, peak_dbfs, rms, rms_dbfs, min_val, max_val, clip_count, (unsigned)total_frames);

            sum_sq = 0;
            peak = 0;
            min_val = INT32_MAX;
            max_val = INT32_MIN;
            clip_count = 0;
            total_frames = 0;
            block_count = 0;
        }
    }
}

void
app_main(void)
{
    xTaskCreate(i2s_capture_task, "i2s_capture", 8192, NULL, 10, NULL);
}
