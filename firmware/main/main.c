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
 * Decimation 48 -> 16 kHz : FIR 80 coefficients (fir_48k_16k.h) via le
 * decimateur optimise d'ESP-DSP (dsps_fird_f32).
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
#include "esp_cpu.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "dsps_fir.h"
#include "fir_48k_16k.h"

static const char *TAG = "i2s_test";

#define I2S_SAMPLE_RATE_HZ   48000
#define I2S_BCLK_GPIO        GPIO_NUM_5
#define I2S_WS_GPIO          GPIO_NUM_6
#define I2S_DIN_GPIO         GPIO_NUM_7

#define DECIM                3
/* 240 echantillons par lecture (5 ms a 48 kHz, multiple de DECIM), niveau
 * affiche toutes les 100 lectures (0,5 s). */
#define FRAMES_PER_READ      240
#define OUT_PER_READ         (FRAMES_PER_READ / DECIM)
#define PRINT_PERIOD_BLOCKS  100

/* Pleine echelle d'un echantillon 24 bits signe (apres le >>8). */
#define FULL_SCALE_24BIT     8388608.0f
#define CPU_HZ               (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000.0)

/* Flux audio vers le PC (USB-Serial/JTAG) : demarre a la reception de 'S'.
 * Trame : magic (4 octets) + sequence (uint16 LE) + nombre d'echantillons
 * (uint16 LE) + echantillons int16 LE mono 16 kHz. Les logs sont coupes
 * pendant le flux pour ne pas melanger texte et binaire. */
static const uint8_t STREAM_MAGIC[4] = {0xA5, 0x5A, 0xC3, 0x3C};
#define STREAM_HEADER_LEN    8

/* Ligne a retard du FIR : N + 4 et alignee sur 16 octets (exigence de la
 * version ESP32-S3 optimisee de dsps_fird_f32). */
static float s_fir_delay[FIR_48K_16K_TAPS + 4] __attribute__((aligned(16)));

static double
to_dbfs(double v)
{
    return v > 0.0 ? 20.0 * log10(v) : -INFINITY;
}

/* Fait passer des sinusoides synthetiques d'amplitude connue dans une instance
 * separee du filtre et affiche le gain mesure : verifie la reponse du filtre
 * sur la cible, independamment du signal audio reel. */
static void
fir_self_test(void)
{
    static float delay[FIR_48K_16K_TAPS + 4] __attribute__((aligned(16)));
    static float in[FRAMES_PER_READ] __attribute__((aligned(16)));
    static float out[OUT_PER_READ] __attribute__((aligned(16)));
    const float freqs[] = {1000, 5000, 7000, 7500, 9000, 10000, 15000, 20000};
    const float amplitude = 0.5f;

    for (size_t k = 0; k < sizeof(freqs) / sizeof(freqs[0]); k++) {
        fir_f32_t fir;
        ESP_ERROR_CHECK(dsps_fird_init_f32(&fir, fir_48k_16k_coeffs, delay, FIR_48K_16K_TAPS, DECIM));
        double phase = 0.0;
        double step = 2.0 * M_PI * freqs[k] / I2S_SAMPLE_RATE_HZ;
        double sum_sq = 0.0;
        int n = 0;
        for (int b = 0; b < 40; b++) {
            for (int i = 0; i < FRAMES_PER_READ; i++) {
                in[i] = amplitude * (float)sin(phase);
                phase += step;
                if (phase > 2.0 * M_PI) {
                    phase -= 2.0 * M_PI;
                }
            }
            int nout = dsps_fird_f32(&fir, in, out, OUT_PER_READ);
            if (b >= 4) { /* ignore le regime transitoire (80 coefficients < 4 blocs) */
                for (int j = 0; j < nout; j++) {
                    sum_sq += (double)out[j] * out[j];
                    n++;
                }
            }
        }
        double gain_db = to_dbfs(sqrt(sum_sq / n) / (amplitude / M_SQRT2));
        ESP_LOGI(TAG, "auto-test FIR : %5.0f Hz -> %6.1f dB", freqs[k], gain_db);
    }
}

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

    fir_f32_t fir;
    ESP_ERROR_CHECK(dsps_fird_init_f32(&fir, fir_48k_16k_coeffs, s_fir_delay, FIR_48K_16K_TAPS, DECIM));

    usb_serial_jtag_driver_config_t usb_cfg = {.tx_buffer_size = 4096, .rx_buffer_size = 256};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
    bool streaming = false;
    uint16_t stream_seq = 0;
    static uint8_t frame[STREAM_HEADER_LEN + OUT_PER_READ * 2];
    ESP_LOGI(TAG, "Envoyer 'S' sur le port USB pour demarrer le flux audio 16 kHz (binaire)");

    /* static plutot que sur la pile : un premier essai avec une pile de tache
     * de 4096 octets et le buffer brut en automatique a provoque un stack
     * overflow qui a corrompu le tas (crash retarde, dans le idle task
     * watchdog). */
    static int32_t raw[FRAMES_PER_READ * 2]; /* stereo : L,R,L,R,... */
    static float mono48[FRAMES_PER_READ] __attribute__((aligned(16)));
    static float out16[OUT_PER_READ] __attribute__((aligned(16)));

    double sum_sq48 = 0.0, sum_sq16 = 0.0;
    float peak48 = 0.0f, peak16 = 0.0f;
    uint32_t clip_count = 0; /* echantillons a moins de 1% de la pleine echelle */
    uint32_t frames48 = 0, frames16 = 0;
    uint64_t fir_cycles = 0;
    int block_count = 0;

    while (1) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_channel_read(rx_chan, raw, sizeof(raw), &bytes_read, 1000);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2s_channel_read: erreur %d (timeout ou surcharge)", err);
            continue;
        }
        size_t frames = bytes_read / (2 * sizeof(int32_t));

        for (size_t i = 0; i < frames; i++) {
            int32_t left = raw[2 * i] >> 8;
            int32_t right = raw[2 * i + 1] >> 8;
            float mono = (float)((left + right) / 2) / FULL_SCALE_24BIT;
            float a = fabsf(mono);
            if (a > peak48) {
                peak48 = a;
            }
            if (a > 0.99f) {
                clip_count++;
            }
            sum_sq48 += (double)mono * mono;
            mono48[i] = mono;
        }
        frames48 += frames;

        uint32_t t0 = esp_cpu_get_cycle_count();
        int nout = dsps_fird_f32(&fir, mono48, out16, frames / DECIM);
        fir_cycles += esp_cpu_get_cycle_count() - t0;

        for (int j = 0; j < nout; j++) {
            float a = fabsf(out16[j]);
            if (a > peak16) {
                peak16 = a;
            }
            sum_sq16 += (double)out16[j] * out16[j];
        }
        frames16 += nout;

        if (!streaming) {
            uint8_t c;
            if (usb_serial_jtag_read_bytes(&c, 1, 0) == 1 && c == 'S') {
                ESP_LOGI(TAG, "Flux audio demarre, logs coupes");
                vTaskDelay(pdMS_TO_TICKS(20));
                esp_log_level_set("*", ESP_LOG_NONE);
                streaming = true;
            }
        } else {
            memcpy(frame, STREAM_MAGIC, 4);
            frame[4] = stream_seq & 0xff;
            frame[5] = stream_seq >> 8;
            frame[6] = nout & 0xff;
            frame[7] = nout >> 8;
            for (int j = 0; j < nout; j++) {
                float v = out16[j] * 32767.0f;
                int32_t q = (int32_t)lrintf(v > 32767.0f ? 32767.0f : (v < -32768.0f ? -32768.0f : v));
                frame[STREAM_HEADER_LEN + 2 * j] = q & 0xff;
                frame[STREAM_HEADER_LEN + 2 * j + 1] = (q >> 8) & 0xff;
            }
            usb_serial_jtag_write_bytes(frame, STREAM_HEADER_LEN + 2 * nout, pdMS_TO_TICKS(20));
            stream_seq++;
        }

        if (++block_count >= PRINT_PERIOD_BLOCKS && !streaming) {
            double cpu_pct = 100.0 * fir_cycles / (frames48 / (double)I2S_SAMPLE_RATE_HZ * CPU_HZ);
            ESP_LOGI(TAG, "48 kHz: pic %.1f rms %.1f dBFS clip %" PRIu32 " | 16 kHz: pic %.1f rms %.1f dBFS"
                     " (%" PRIu32 " ech.) | FIR %.2f %% CPU",
                     to_dbfs(peak48), to_dbfs(sqrt(sum_sq48 / frames48)), clip_count,
                     to_dbfs(peak16), to_dbfs(sqrt(sum_sq16 / frames16)), frames16, cpu_pct);
            sum_sq48 = sum_sq16 = 0.0;
            peak48 = peak16 = 0.0f;
            clip_count = 0;
            frames48 = frames16 = 0;
            fir_cycles = 0;
            block_count = 0;
        }
    }
}

void
app_main(void)
{
    fir_self_test();
    xTaskCreate(i2s_capture_task, "i2s_capture", 8192, NULL, 10, NULL);
}
