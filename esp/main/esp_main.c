/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_psram.h"
#include "esp_partition.h"
#include "driver/uart.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/ppa.h"
#include "esp_cache.h"

#include "../../ini.h"
#include "../../pc.h"
#include "common.h"

//
#include "esp_private/system_internal.h"
uint32_t get_uticks()
{
	return esp_system_get_time();
}

void *psmalloc(long size);
void *fbmalloc(long size);
void *bigmalloc(size_t size)
{
	return psmalloc(size);
}

static char *pcram;
static long pcram_off;
static long pcram_len;
void *pcmalloc(long size)
{
	void *ret = pcram + pcram_off;

	size = (size + 31) / 32 * 32;
	if (pcram_off + size > pcram_len) {
		fprintf(stderr, "pcram error %ld %ld %ld\n", size, pcram_off, pcram_len);
		abort();
	}
	pcram_off += size;
	return ret;
}

void pcmalloc_init(void *ptr, long len)
{
	pcram = ptr;
	pcram_len = len;
}

int load_rom(void *phys_mem, const char *file, uword addr, int backward)
{
	if (file && file[0] == '/') {
		FILE *fp = fopen(file, "rb");
		assert(fp);
		fseek(fp, 0, SEEK_END);
		int len = ftell(fp);
		fprintf(stderr, "%s len %d\n", file, len);
		rewind(fp);
		if (backward)
			fread(phys_mem + addr - len, 1, len, fp);
		else
			fread(phys_mem + addr, 1, len, fp);
		fclose(fp);
		return len;
	}
	const esp_partition_t *part =
		esp_partition_find_first(ESP_PARTITION_TYPE_ANY,
					 ESP_PARTITION_SUBTYPE_ANY,
					 file);
	assert(part);
	int len = part->size;
	fprintf(stderr, "%s len %d\n", file, len);
	if (backward)
		esp_partition_read(part, 0, phys_mem + addr - len, len);
	else
		esp_partition_read(part, 0, phys_mem + addr, len);
	return len;
}

//
EventGroupHandle_t global_event_group;
struct Globals globals;

typedef struct {
	PC *pc;
	u8 *fb1;
	u8 *fb;
} Console;

#define NN 24

static uint16_t *volatile disp_src = NULL;
static TaskHandle_t disp_task_handle = NULL;
static uint16_t *rot_buf = NULL;
static uint16_t *snap_buf = NULL;
static ppa_client_handle_t ppa_srm_handle = NULL;
static volatile bool g_no_rotate = false;
static SemaphoreHandle_t ppa_done_sem = NULL;
static bool ppa_trans_done_cb(ppa_client_handle_t h, ppa_event_data_t *e, void *u)
{
    BaseType_t w = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)u, &w);
    return w == pdTRUE;
}
static void ppa_benchmark(void)
{
    if (!ppa_srm_handle) return;
    ppa_srm_oper_config_t o;
    int64_t a, b;
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) rot_buf[i] = (uint16_t)((i * 2654435761u) >> 16);
    esp_cache_msync(rot_buf, LCD_WIDTH * LCD_HEIGHT * 2, ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
    a = esp_timer_get_time();
    memcpy(snap_buf, rot_buf, LCD_WIDTH * LCD_HEIGHT * 2);
    esp_cache_msync(snap_buf, LCD_WIDTH * LCD_HEIGHT * 2, ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
    b = esp_timer_get_time();
    ESP_LOGW("PPABENCH", "BUS cpu-memcpy 1.8MB: %lld us (~%d MB/s)", (long long)(b - a), (int)((LCD_WIDTH * LCD_HEIGHT * 2) / (b - a)));
    memset(&o, 0, sizeof(o));
    o.in.buffer = rot_buf; o.in.pic_w = 720; o.in.pic_h = 480;
    o.in.block_w = 720; o.in.block_h = 480; o.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    o.out.buffer = snap_buf; o.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    o.mode = PPA_TRANS_MODE_BLOCKING; o.user_data = ppa_done_sem;
    o.scale_x = 1.0f; o.scale_y = 1.0f; o.rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
    o.out.buffer_size = (720 * 480 * 2 + 127) & ~127; o.out.pic_w = 720; o.out.pic_h = 480;
    ppa_do_scale_rotate_mirror(ppa_srm_handle, &o);
    a = esp_timer_get_time();
    for (int i = 0; i < 5; i++) ppa_do_scale_rotate_mirror(ppa_srm_handle, &o);
    b = esp_timer_get_time();
    ESP_LOGW("PPABENCH", "BUS ppa-copy 720x480: %lld us", (long long)((b - a) / 5));
    o.out.buffer_size = (1280 * 720 * 2 + 127) & ~127; o.out.pic_w = 1280; o.out.pic_h = 720;
    o.scale_x = 1280.0f / 720.0f; o.scale_y = 720.0f / 480.0f;
    ppa_do_scale_rotate_mirror(ppa_srm_handle, &o);
    a = esp_timer_get_time();
    for (int i = 0; i < 5; i++) ppa_do_scale_rotate_mirror(ppa_srm_handle, &o);
    b = esp_timer_get_time();
    ESP_LOGW("PPABENCH", "SCALE 720x480->1280x720: %lld us", (long long)((b - a) / 5));
    o.out.buffer_size = (480 * 720 * 2 + 127) & ~127; o.out.pic_w = 480; o.out.pic_h = 720;
    o.scale_x = 1.0f; o.scale_y = 1.0f; o.rotation_angle = PPA_SRM_ROTATION_ANGLE_270;
    ppa_do_scale_rotate_mirror(ppa_srm_handle, &o);
    a = esp_timer_get_time();
    for (int i = 0; i < 5; i++) ppa_do_scale_rotate_mirror(ppa_srm_handle, &o);
    b = esp_timer_get_time();
    ESP_LOGW("PPABENCH", "ROTATE 720x480->480x720: %lld us", (long long)((b - a) / 5));
    o.out.buffer_size = (720 * 1280 * 2 + 127) & ~127; o.out.pic_w = 720; o.out.pic_h = 1280;
    o.scale_x = 1280.0f / 720.0f; o.scale_y = 720.0f / 480.0f;
    ppa_do_scale_rotate_mirror(ppa_srm_handle, &o);
    a = esp_timer_get_time();
    for (int i = 0; i < 5; i++) ppa_do_scale_rotate_mirror(ppa_srm_handle, &o);
    b = esp_timer_get_time();
    ESP_LOGW("PPABENCH", "FULL scale+rotate ->720x1280: %lld us", (long long)((b - a) / 5));
}
   /* Benchmark: Rotation aus */
static void display_task(void *arg);

Console *console_init(int width, int height)
{
	Console *c = malloc(sizeof(Console));
	c->fb1 = fbmalloc(LCD_WIDTH * LCD_HEIGHT / NN * 2);
	if (rot_buf == NULL) {
		/* PSRAM-Heap ignoriert grosse Alignments -> manuell auf 64 aufrunden */
		size_t sz = LCD_WIDTH * LCD_HEIGHT * 2;
		uint8_t *raw = heap_caps_malloc(sz + 128, MALLOC_CAP_SPIRAM);
		rot_buf = (uint16_t *)(((uintptr_t)raw + 127) & ~(uintptr_t)127);
	}
	if (snap_buf == NULL) {
		size_t sz = LCD_WIDTH * LCD_HEIGHT * 2;
		uint8_t *raw = heap_caps_malloc(sz + 128, MALLOC_CAP_SPIRAM);
		snap_buf = (uint16_t *)(((uintptr_t)raw + 127) & ~(uintptr_t)127);
	}
	if (ppa_srm_handle == NULL) {
		ppa_client_config_t ppa_cfg = {
			.oper_type = PPA_OPERATION_SRM,
			.max_pending_trans_num = 1,
		};
		esp_err_t err = ppa_register_client(&ppa_cfg, &ppa_srm_handle);
		if (err != ESP_OK) {
			ESP_LOGE("PPA", "register failed: %s", esp_err_to_name(err));
			ppa_srm_handle = NULL;
		} else {
			ESP_LOGI("PPA", "SRM client registered");
		ppa_done_sem = xSemaphoreCreateBinary();
		xSemaphoreGive(ppa_done_sem);
		ppa_event_callbacks_t cbs = { .on_trans_done = ppa_trans_done_cb };
		ppa_client_register_event_callbacks(ppa_srm_handle, &cbs);
		ppa_benchmark();
		}
	}
	if (disp_task_handle == NULL)
		xTaskCreatePinnedToCore(display_task, "display", 4096, NULL, 0,
					&disp_task_handle, 0);

	/* VGA rendert in eigenen Buffer (PPA-Input); die PPA schreibt
	 * zero-copy in den DPI-Framebuffer (Output). Kein In-Place! */
	c->fb = bigmalloc(LCD_WIDTH * LCD_HEIGHT * 2);
	return c;
}

void lcd_draw(int x_start, int y_start, int x_end, int y_end, void *src);
#define STRETCH 1   /* Vollbild-Streckung */
/* Eigener Display-Task (niedrige Prio, Core 0): macht die teure
 * Stretch+Transpose-Arbeit, OHNE die vga_step/Retrace-Schleife zu blockieren. */
/* Umschaltbar via Strg+M auf der USB-Tastatur (Hook in usb_input.c). */
void display_toggle_rotate(void)
{
	g_no_rotate = !g_no_rotate;
	ESP_LOGW("PERF", ">>> no_rotate=%d (Strg+M)", (int) g_no_rotate);
}

static void display_task(void *arg)
{
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		uint16_t *src = disp_src;
		uint16_t *out_buf = globals.panel_fb ? (uint16_t *)globals.panel_fb : rot_buf;
		if (!src || !rot_buf || !snap_buf)
			continue;

		/* Kein Snapshot noetig: vga_task (schreibt fb) und display_task
		 * (PPA) laufen beide auf Core 0 -> fb ist stabil waehrend PPA liest. */
		int64_t t0 = esp_timer_get_time();
		int64_t t1 = t0;   /* memcpy entfaellt */

		int64_t t_cfg = 0, t_ppa = 0, t_edge = 0, t_sync = 0, t_wait = 0;

		if (ppa_srm_handle) {
			/* PPA-Hardware: Rotation 270 + Skalierung in einem Durchlauf */
			int64_t tw0 = esp_timer_get_time();
			xSemaphoreTake(ppa_done_sem, portMAX_DELAY);
			t_wait = esp_timer_get_time() - tw0;
			int64_t tc0 = esp_timer_get_time();
			ppa_srm_oper_config_t oper;
			memset(&oper, 0, sizeof(oper));
			oper.in.buffer  = src;
			oper.in.pic_w   = LCD_WIDTH;    /* 1280 */
			oper.in.pic_h   = LCD_HEIGHT;   /* 720 */

			/* Wie alter STRETCH-Pfad:
			 * Nimm den zentralen 720x480-Ausschnitt aus dem 1280x720 VGA-Framebuffer
			 * und strecke ihn auf Vollbild. */
			oper.in.block_offset_x = 280;
			oper.in.block_offset_y = 120;
			oper.in.block_w = 720;
			oper.in.block_h = 480;
			oper.in.srm_cm  = PPA_SRM_COLOR_MODE_RGB565;

			oper.out.buffer      = out_buf;
			/* PPA erfordert: buffer_size muss aligned sein */
			size_t raw_sz = LCD_WIDTH * LCD_HEIGHT * 2;
			oper.out.buffer_size = (raw_sz + 127) & ~127;  /* auf 128 runden */
			oper.out.pic_w       = LCD_HEIGHT;  /* 720 */
			oper.out.pic_h       = LCD_WIDTH;   /* 1280 */
			oper.out.srm_cm      = PPA_SRM_COLOR_MODE_RGB565;

			/* Orientierung war mit 270 korrekt.
			 * PPA skaliert vor der Rotation:
			 * 720x480 -> 1280x720, danach Rotation -> 720x1280. */
			if (g_no_rotate) {
				/* BENCHMARK: keine Rotation, nur Skalierung */
				oper.rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
				oper.scale_x = 1280.0f / 720.0f;
				oper.scale_y = 1280.0f / 480.0f;
			} else {
				oper.rotation_angle = PPA_SRM_ROTATION_ANGLE_270;
				oper.scale_x = 1280.0f / 720.0f;
				oper.scale_y = 720.0f / 480.0f;
			}
			oper.mirror_x = false;
			oper.mirror_y = false;
			oper.mode = PPA_TRANS_MODE_NON_BLOCKING;
			oper.user_data = ppa_done_sem;
			int64_t tc1 = esp_timer_get_time();
			t_cfg = tc1 - tc0;

			esp_err_t perr = ppa_do_scale_rotate_mirror(ppa_srm_handle, &oper);
			int64_t tc2 = esp_timer_get_time();
			t_ppa = tc2 - tc1;
			if (perr != ESP_OK) {
				xSemaphoreGive(ppa_done_sem);
				static int ec = 0;
				if (++ec <= 3)
					ESP_LOGE("PPA", "srm failed: %s", esp_err_to_name(perr));
				/* Fallback: Software */
				for (int y = 0; y < LCD_HEIGHT; y++) {
					const uint16_t *row = src + (size_t) y * LCD_WIDTH;
					uint16_t *dst = rot_buf + y;
					for (int x = 0; x < LCD_WIDTH; x++)
						dst[(size_t) x * LCD_HEIGHT] = row[LCD_WIDTH - 1 - x];
				}
			}
		} else {
			int64_t tc1 = esp_timer_get_time();
			/* Fallback: Software */
			for (int y = 0; y < LCD_HEIGHT; y++) {
				const uint16_t *row = src + (size_t) y * LCD_WIDTH;
				uint16_t *dst = rot_buf + y;
				for (int x = 0; x < LCD_WIDTH; x++)
					dst[(size_t) x * LCD_HEIGHT] = row[LCD_WIDTH - 1 - x];
			}
			t_ppa = esp_timer_get_time() - tc1;
		}

		/* Kanten-Artefakt der PPA-Rotation: aeusserste Zeilen schwarz */
		int64_t te0 = esp_timer_get_time();
		memset(out_buf, 0, 2 * 720 * 2);                      /* row 0..1  -> rechte Kante */
		memset(out_buf + (1280 - 2) * 720, 0, 2 * 720 * 2);
		memset(rot_buf, 0, 2 * 720 * 2);                      /* row 0..1  -> rechte Kante */
		memset(rot_buf + (1280 - 2) * 720, 0, 2 * 720 * 2);   /* row last  -> linke Kante  */
		int64_t te1 = esp_timer_get_time();
		t_edge = te1 - te0;

		esp_cache_msync(out_buf, 4 * 720 * 2, ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
		esp_cache_msync(out_buf + (1280 - 4) * 720, 4 * 720 * 2, ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
		int64_t t2 = esp_timer_get_time();
		t_sync = t2 - te1;

		if (!globals.panel_fb)
			lcd_draw(0, 0, 720, 1280, rot_buf);
		int64_t t3 = esp_timer_get_time();

		static int fc = 0;
		static int64_t acc_mem = 0, acc_dr = 0;
		static int64_t acc_cfg = 0, acc_ppa = 0, acc_edge = 0, acc_sync = 0, acc_wait = 0;
		static int64_t last_log = 0;
		acc_mem  += t1 - t0;
		acc_cfg  += t_cfg;
		acc_ppa  += t_ppa;
		acc_edge += t_edge;
		acc_sync += t_sync;
		acc_wait += t_wait;
		acc_dr   += t3 - t2;
		fc++;
		if (t3 - last_log > 1000000) {   /* 1x pro Sekunde */
			long tr = (long)((acc_cfg + acc_ppa + acc_edge + acc_sync) / fc);
			ESP_LOGW("PERF", "disp fps=%d avg us: memcpy=%ld | tr=%ld [cfg=%ld ppa=%ld wait=%ld edge=%ld sync=%ld] | draw=%ld",
				 fc, (long)(acc_mem / fc), tr,
				 (long)(acc_cfg / fc), (long)(acc_ppa / fc), (long)(acc_wait / fc),
				 (long)(acc_edge / fc), (long)(acc_sync / fc),
				 (long)(acc_dr / fc));
			fc = 0;
			acc_mem = acc_dr = 0;
			acc_cfg = acc_ppa = acc_edge = acc_sync = acc_wait = 0;
			last_log = t3;
		}
	}
}

static int redraw_count = 0;
static int redraw_last_log = 0;
static void redraw(void *opaque,
		   int x, int y, int w, int h)
{
	Console *s = opaque;
	/* Nur signalisieren - die schwere Arbeit macht display_task,
	 * damit vga_step/Retrace nicht blockiert wird. */
	disp_src = (uint16_t *) s->fb;
	redraw_count++;
	if (disp_task_handle)
		xTaskNotifyGive(disp_task_handle);
}


static int pc_main(const char *file)
{
	PCConfig conf;
	memset(&conf, 0, sizeof(conf));
	conf.mem_size = 8 * 1024 * 1024;
	conf.vga_mem_size = 256 * 1024;
	conf.cpu_gen = 4;
	conf.fpu = 0;

	int err = ini_parse(file, parse_conf_ini, &conf);
	if (err) {
		fprintf(stderr, "error %d\n", err);
		return err;
	}

	if (conf.width != LCD_WIDTH || conf.height != LCD_HEIGHT) {
		fprintf(stderr, "fixing width/height mismatch %dx%d => %dx%d\n",
			conf.width, conf.height, LCD_WIDTH, LCD_HEIGHT);
		conf.width = LCD_WIDTH;
		conf.height = LCD_HEIGHT;
	}

	Console *console = console_init(conf.width, conf.height);
	PC *pc = pc_new(redraw, console, console->fb, &conf);
	console->pc = pc;
	globals.pc = pc;
	globals.kbd = pc->kbd;
	globals.mouse = pc->mouse;
	xEventGroupSetBits(global_event_group, BIT0);

	load_bios_and_reset(pc);

	pc->boot_start_time = get_uticks();
	for (; pc->shutdown_state != 8;) {
		pc_step(pc);
	}
	return 0;
}

//

void *esp_psram_get(size_t *size);
void vga_task(void *arg);
void i2s_main();
void wifi_main(const char *, const char *);
void storage_init(void);
void usb_setup(void);

struct esp_ini_config {
	const char *filename;
	char ssid[16];
	char pass[32];
	int enable_usb;
};

static void i386_task(void *arg)
{
	struct esp_ini_config *config = arg;
	int core_id = esp_cpu_get_core_id();
	fprintf(stderr, "main runs on core %d\n", core_id);
	/* Wait for LCD panel (and panel_fb) to be ready before starting the
	 * PC emulator.  console_init() uses globals.panel_fb if set. */
	xEventGroupWaitBits(global_event_group,
	                    BIT1,
	                    pdFALSE,
	                    pdFALSE,
	                    portMAX_DELAY);
	pc_main(config->filename);
	vTaskDelete(NULL);
}

static char *psram;
static long psram_off;
static long psram_len;
void *psmalloc(long size)
{
	void *ret = psram + psram_off;

	size = (size + 4095) / 4096 * 4096;
	if (psram_off + size > psram_len) {
		fprintf(stderr, "psram error %ld %ld %ld\n", size, psram_off, psram_len);
		abort();
	}
	psram_off += size;
	return ret;
}

void *fbmalloc(long size)
{
	void *fb = (uint8_t *) heap_caps_calloc(1, size, MALLOC_CAP_DMA);
	if (!fb) {
		fprintf(stderr, "fbmalloc error %ld\n", size);
		abort();
	}
	return fb;
}

static int parse_ini(void* user, const char* section,
		     const char* name, const char* value)
{
	struct esp_ini_config *conf = user;
#define SEC(a) (strcmp(section, a) == 0)
#define NAME(a) (strcmp(name, a) == 0)
	if (SEC("esp")) {
		if (NAME("ssid")) {
			if (strlen(value) < 32)
				strcpy(conf->ssid, value);
		} else if (NAME("pass")) {
			if (strlen(value) < 64)
				strcpy(conf->pass, value);
		} else if (NAME("enable_usb")) {
			conf->enable_usb = atoi(value);
		}
	}
#undef SEC
#undef NAME
	return 1;
}

void app_main(void)
{
	global_event_group = xEventGroupCreate();

#ifdef ESPDEBUG
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity	= UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	uart_param_config(UART_NUM_0, &uart_config);
	if (uart_driver_install(UART_NUM_0, 2 * 1024, 0, 0, NULL, 0) != ESP_OK) {
		assert(false);
	}
#endif

	i2s_main();
	storage_init();

	esp_psram_init();
#ifndef PSRAM_ALLOC_LEN
	// use the whole psram
	size_t len;
	psram_len = 20 * 1024 * 1024;
	psram = heap_caps_calloc(1, psram_len, MALLOC_CAP_SPIRAM);
	if (!psram) {  /* Fallback: kleiner */
		psram_len = 12 * 1024 * 1024;
		psram = heap_caps_calloc(1, psram_len, MALLOC_CAP_SPIRAM);
	}
	ESP_LOGW("MEM", "emulator PSRAM pool = %ld MB", psram_len / (1024*1024));
#else
	psram_len = 20 * 1024 * 1024;
	psram = heap_caps_calloc(1, psram_len, MALLOC_CAP_SPIRAM);
	if (!psram) {  /* Fallback: kleiner */
		psram_len = 12 * 1024 * 1024;
		psram = heap_caps_calloc(1, psram_len, MALLOC_CAP_SPIRAM);
	}
	ESP_LOGW("MEM", "emulator PSRAM pool = %ld MB", psram_len / (1024*1024));
#endif

	const static char *files[] = {
		"/sdcard/tiny386.ini",
		"/spiflash/tiny386.ini",
		NULL,
	};
	static struct esp_ini_config config;
	for (int i = 0; files[i]; i++) {
		if (ini_parse(files[i], parse_ini, &config) == 0) {
			config.filename = files[i];
			break;
		}
	}

	if (config.enable_usb) {
		vTaskDelay(2000 / portTICK_PERIOD_MS);
		usb_setup();
	}

	if (config.ssid[0]) {
		wifi_main(config.ssid, config.pass);
	}

	if (psram) {
		xTaskCreatePinnedToCore(i386_task, "i386_main", 4096, &config, 3, NULL, 1);
		xTaskCreatePinnedToCore(vga_task, "vga_task", 4096, NULL, 0, NULL, 0);
	}
}
