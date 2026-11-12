/**
 * @file bridge.c
 * @brief I2C/UART/SPI <-> 1-Wire bridge core - see bridge.h
 *
 * One command at a time owns the 1-Wire bus (the library rule). A submitted
 * command either starts immediately or is queued; the host polls the response
 * until it stops being BR_BUSY. Each command is a start/poll pair over the
 * library's non-blocking API.
 */

#include "bridge.h"
#include "cache.h"
#include "stats.h"
#include "config.h"
#include "board.h"

#include "onewire.h"
#include "ds18b20.h"
#include "ds2431.h"
#include "ds2413.h"

#include "fast_copy.h"

/* --- Bus state ------------------------------------------------------------ */

typedef enum {
    BR_BUS_IDLE = 0,
    BR_BUS_BUSY,
    BR_BUS_DONE
} br_bus_state_t;

typedef struct {
    void (*start)(const br_command_t *cmd);
    uint8_t (*poll)(br_response_t *resp);
    uint8_t instant;   /**< poll() completes on the first call (RAM-only) */
} br_cmd_desc_t;

typedef struct {
    br_bus_state_t state;
    uint8_t account_pending;
    br_command_t cmd;
    br_response_t resp;
    const br_cmd_desc_t *desc;
    uint8_t resp_valid;
} br_bus_t;

static br_bus_t bus;

/* Set by OW_SET_PULLUP so the bridge knows the host wants the strong pull-up
   to stay engaged across commands (e.g. a host-driven parasite conversion). */
static uint8_t pullup_hold;

/* --- Last response (decoupled from the bus state machine) ----------------- */

/* The frame starts one byte into the storage so that last_resp + 3 (the data
   area) is word-aligned: fast_copy() can then use 32-bit moves even though the
   header itself is byte-sized. */
static union {
    uint32_t align;
    uint8_t bytes[4u + 3u + BR_MAX_RESP];
} last_resp_store;
#define last_resp (last_resp_store.bytes + 1u)

static uint8_t last_resp_len;
static uint8_t last_resp_seq;

/* Shared scratch buffers: only one bus operation is active at a time, so the
   raw-command pulse buffers share one allocation. This is what keeps the
   bridge inside the 4 KB RAM of the F030F4. */
static ow_pulse_t shared_pulse[BR_MAX_DATA * 8u + 1u];
static uint8_t shared_read[BR_MAX_DATA * 8u] __attribute__((aligned(4)));
static uint8_t shared_payload[BR_MAX_DATA] __attribute__((aligned(4)));

static inline void resp_commit(void)
{
    /* The command poll writes directly into last_resp + 3 (bus.resp.data), so
       committing the response only updates the three header bytes. */
    last_resp[0] = bus.resp.status;
    last_resp[1] = bus.resp.cmd;
    last_resp[2] = bus.resp.len;
    last_resp_len = (uint8_t)(3u + bus.resp.len);
    last_resp_seq++;
}

static inline void resp_busy(uint8_t cmd)
{
    bus.resp.status = BR_BUSY;
    bus.resp.cmd = cmd;
    bus.resp.len = 0u;
    resp_commit();
}

/* Mirror the last completed command on the error LED (active-high API). */
static void led_result(uint8_t status)
{
    if (config_get()->led_enable) {
        board_led_error((status == BR_OK) ? 0u : 1u);
    }
}

/* Latch the response and account for a finished command. Used both by the
   deferred main-loop poll and by the immediate poll in bridge_submit(), so an
   instant command never has to pass through a visible BUSY state. */
static inline void bus_finish_command(void)
{
    /* Latch the response first. Statistics and the LED are not part of the
       response, so they are accounted from the main loop; this keeps the
       I2C STOP handler short enough for a fast STOP->START turnaround. */
    resp_commit();
    bus.state = BR_BUS_DONE;
    bus.account_pending = 1u;
}

/* Arm a command. The start is always executed here: it is cheap (it only
   programs the 1-Wire engine) and starting it at STOP means BUSY polling by
   the host cannot starve the operation. Instant commands additionally run
   their poll so the response is ready before the next I2C START; for long
   commands the poll runs in the main loop, keeping the STOP handler short. */
static inline void bus_begin_command(const br_cmd_desc_t *desc, uint8_t cmd)
{
    /* The DS18B20 driver parks in parasite mode with the strong pull-up
       engaged. On this port that switches PA10 to push-pull, so the bus
       cannot be driven by a slave until an explicit release. Restore the
       open-drain line before every operation unless the host asked to hold
       the pull-up through OW_SET_PULLUP. */
    if (!pullup_hold) {
        onewire_strong_pullup(0);
        if (config_get()->strong_pullup) {
            board_gate_pullup(0);
        }
    }
    bus.desc = desc;
    bus.state = BR_BUS_BUSY;
    desc->start(&bus.cmd);
    if (desc->instant && desc->poll(&bus.resp)) {
        bus_finish_command();
    } else {
        resp_busy(cmd);
    }
}

/* --- Command queue -------------------------------------------------------- */

static br_command_t queue[BR_QUEUE_SIZE];
static uint8_t queue_head;
static uint8_t queue_tail;
static uint8_t queue_count;

/* --- DS18B20 completion capture ------------------------------------------- */

static volatile uint8_t ds18b20_done;
static int16_t ds18b20_temp;

/* Weak callback overrides -------------------------------------------------- */

void ds18b20_complete(int16_t temp)
{
    ds18b20_temp = temp;
    ds18b20_done = 1u;
}

void ds18b20_busy(unsigned action)
{
    if (config_get()->led_enable) {
        board_led_status(action ? 1u : 0u);
    }
}

/* --- Search sink ---------------------------------------------------------- */

static uint8_t search_sink(const uint8_t *rom)
{
    cache_add(rom);
    return 0u; /* continue the search */
}

/* --- Helpers -------------------------------------------------------------- */

static void encode_bytes(ow_pulse_t *out, const uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0u; i < len; i++) {
        onewire_encode_byte(&out[i * 8u], data[i]);
    }
    out[len * 8u] = ONEWIRE_RELEASE_PULSE;
}

static int16_t decode_temp(const uint8_t *scratch)
{
    int16_t raw = (int16_t)((scratch[1] << 8) | scratch[0]);
    return (int16_t)(((int32_t)raw * 10 + ((raw < 0) ? -8 : 8)) / 16);
}

static uint8_t scratch_resolution(const uint8_t *scratch)
{
    return (uint8_t)(DS18B20_RES_MIN + ((scratch[4] >> 5) & 0x3u));
}

static void resp_simple(br_response_t *resp, uint8_t cmd, uint8_t status)
{
    resp->status = status;
    resp->cmd = cmd;
    resp->len = 0u;
}

/* ========================================================================= */
/* Raw 1-Wire commands                                                      */
/* ========================================================================= */

/* --- OW_RESET ------------------------------------------------------------- */

static uint16_t reset_pulses[2];

static void start_ow_reset(const br_command_t *cmd)
{
    (void)cmd;
    onewire_reset(reset_pulses);
}

static uint8_t poll_ow_reset(br_response_t *resp)
{
    if (!onewire_bus_done()) {
        return 0u;
    }
    uint8_t captured = onewire_reset_captured();
    uint8_t present = onewire_present(reset_pulses, captured);
    stats_count(present ? BR_STAT_RESETS : BR_STAT_PRESENCE_FAIL);
    resp->status = present ? BR_OK : BR_NO_DEVICE;
    resp->cmd = BR_CMD_OW_RESET;
    resp->len = 2u;
    resp->data[0] = present;
    resp->data[1] = 0u; /* short-detect not wired */
    return 1u;
}

/* --- OW_WRITE_BYTE -------------------------------------------------------- */

static ow_pulse_t write_byte_pulse[9];

static void start_ow_write_byte(const br_command_t *cmd)
{
    onewire_encode_byte(write_byte_pulse, cmd->data[0]);
    write_byte_pulse[8] = ONEWIRE_RELEASE_PULSE;
    onewire_write_slots(write_byte_pulse, 8u);
}

static uint8_t poll_ow_write_byte(br_response_t *resp)
{
    if (!onewire_bus_done()) {
        return 0u;
    }
    resp_simple(resp, BR_CMD_OW_WRITE_BYTE, BR_OK);
    return 1u;
}

/* --- OW_READ_BYTE --------------------------------------------------------- */

static uint8_t read_byte_pulse[8];
static uint8_t read_byte_value;

static void start_ow_read_byte(const br_command_t *cmd)
{
    (void)cmd;
    onewire_read_data(read_byte_pulse, 1u);
}

static uint8_t poll_ow_read_byte(br_response_t *resp)
{
    if (!onewire_bus_done()) {
        return 0u;
    }
    onewire_decode_pulses(&read_byte_value, read_byte_pulse, 1u);
    resp->status = BR_OK;
    resp->cmd = BR_CMD_OW_READ_BYTE;
    resp->len = 1u;
    resp->data[0] = read_byte_value;
    return 1u;
}

/* --- OW_WRITE_BYTES ------------------------------------------------------- */

static void start_ow_write_bytes(const br_command_t *cmd)
{
    encode_bytes(shared_pulse, cmd->data, cmd->len);
    onewire_write_slots(shared_pulse, (uint16_t)cmd->len * 8u);
}

static uint8_t poll_ow_write_bytes(br_response_t *resp)
{
    if (!onewire_bus_done()) {
        return 0u;
    }
    resp_simple(resp, BR_CMD_OW_WRITE_BYTES, BR_OK);
    return 1u;
}

/* --- OW_READ_BYTES -------------------------------------------------------- */

static uint8_t read_bytes_len;

static void start_ow_read_bytes(const br_command_t *cmd)
{
    uint8_t n = (cmd->len > 0u) ? cmd->data[0] : 1u;
    if (n > ONEWIRE_MAX_READ_BYTES) {
        n = ONEWIRE_MAX_READ_BYTES;
    }
    read_bytes_len = n;
    onewire_read_data(shared_read, n);
}

static uint8_t poll_ow_read_bytes(br_response_t *resp)
{
    if (!onewire_bus_done()) {
        return 0u;
    }
    onewire_decode_pulses(resp->data, shared_read, read_bytes_len);
    resp->status = BR_OK;
    resp->cmd = BR_CMD_OW_READ_BYTES;
    resp->len = read_bytes_len;
    return 1u;
}

/* --- OW_MATCH_ROM / OW_SKIP_ROM ------------------------------------------ */

static uint8_t match_step;
static uint8_t match_payload_len;
static uint16_t match_reset_pulses[2];

static void start_ow_match(const br_command_t *cmd)
{
    match_payload_len = (uint8_t)(cmd->len - 8u);
    if (match_payload_len > BR_MAX_DATA) {
        match_payload_len = 0u;
    }
    fast_copy(shared_payload, cmd->data + 8u, match_payload_len);
    match_step = 0u;
    /* reset */
    onewire_reset(match_reset_pulses);
}

static uint8_t poll_ow_match(br_response_t *resp)
{
    switch (match_step) {
    case 0u:
        if (!onewire_bus_done()) {
            return 0u;
        }
        {
            uint8_t buf[9];
            buf[0] = ONEWIRE_MATCH_ROM;
            fast_copy(buf + 1u, bus.cmd.data, 8u);
            encode_bytes(shared_pulse, buf, 9u);
            onewire_write_slots(shared_pulse, 72u);
        }
        match_step = 1u;
        return 0u;
    case 1u:
        if (!onewire_bus_done()) {
            return 0u;
        }
        if (match_payload_len > 0u) {
            encode_bytes(shared_pulse, shared_payload, match_payload_len);
            onewire_write_slots(shared_pulse, (uint16_t)match_payload_len * 8u);
            match_step = 2u;
            return 0u;
        }
        match_step = 3u;
        return 0u;
    case 2u:
        if (!onewire_bus_done()) {
            return 0u;
        }
        match_step = 3u;
        return 0u;
    default:
        resp_simple(resp, BR_CMD_OW_MATCH_ROM, BR_OK);
        return 1u;
    }
}

static uint8_t skip_step;
static uint16_t skip_reset_pulses[2];

static void start_ow_skip(const br_command_t *cmd)
{
    fast_copy(shared_payload, cmd->data, cmd->len);
    skip_step = 0u;
    onewire_reset(skip_reset_pulses);
}

static uint8_t poll_ow_skip(br_response_t *resp)
{
    switch (skip_step) {
    case 0u:
        if (!onewire_bus_done()) {
            return 0u;
        }
        encode_bytes(shared_pulse, shared_payload, bus.cmd.len);
        onewire_write_slots(shared_pulse, (uint16_t)bus.cmd.len * 8u);
        skip_step = 1u;
        return 0u;
    case 1u:
        if (!onewire_bus_done()) {
            return 0u;
        }
        skip_step = 2u;
        return 0u;
    default:
        resp_simple(resp, BR_CMD_OW_SKIP_ROM, BR_OK);
        return 1u;
    }
}

/* --- OW_READ_ROM ---------------------------------------------------------- */

static uint8_t read_rom_step;
static uint16_t read_rom_reset_pulses[2];

static void start_ow_read_rom(const br_command_t *cmd)
{
    (void)cmd;
    read_rom_step = 0u;
    onewire_reset(read_rom_reset_pulses);
}

static uint8_t poll_ow_read_rom(br_response_t *resp)
{
    switch (read_rom_step) {
    case 0u:
        if (!onewire_bus_done()) {
            return 0u;
        }
        {
            ow_pulse_t pulse[9];
            onewire_encode_byte(pulse, 0x33u);
            pulse[8] = ONEWIRE_RELEASE_PULSE;
            onewire_write_slots(pulse, 8u);
        }
        read_rom_step = 1u;
        return 0u;
    case 1u:
        if (!onewire_bus_done()) {
            return 0u;
        }
        onewire_read_data(shared_read, 8u);
        read_rom_step = 2u;
        return 0u;
    case 2u:
        if (!onewire_bus_done()) {
            return 0u;
        }
        onewire_decode_pulses(resp->data, shared_read, 8u);
        resp->status = BR_OK;
        resp->cmd = BR_CMD_OW_READ_ROM;
        resp->len = 8u;
        return 1u;
    default:
        resp_simple(resp, BR_CMD_OW_READ_ROM, BR_BUS_ERROR);
        return 1u;
    }
}

/* --- OW_SEARCH / OW_SEARCH_ALM ------------------------------------------- */

static uint8_t search_cmd;
static uint8_t search_family;
static uint8_t search_max;

static void start_ow_search(const br_command_t *cmd)
{
    search_cmd = (cmd->len > 0u) ? cmd->data[0] : ONEWIRE_SEARCH_ROM;
    search_family = (cmd->len > 1u) ? cmd->data[1] : 0u;
    search_max = (cmd->len > 2u) ? cmd->data[2] : (uint8_t)BR_CACHE_MAX;
    cache_clear();
    onewire_search_start(search_sink, search_max, search_cmd, search_family);
}

static uint8_t poll_ow_search(br_response_t *resp)
{
    if (!onewire_search_poll()) {
        return 0u;
    }
    resp->status = BR_OK;
    resp->cmd = BR_CMD_OW_SEARCH;
    resp->len = 1u;
    resp->data[0] = cache_count();
    return 1u;
}

static void start_ow_search_alm(const br_command_t *cmd)
{
    search_cmd = ONEWIRE_ALARM_SEARCH;
    search_family = 0u;
    search_max = (cmd->len > 0u) ? cmd->data[0] : (uint8_t)BR_CACHE_MAX;
    cache_clear();
    onewire_search_start(search_sink, search_max, search_cmd, search_family);
}

static uint8_t poll_ow_search_alm(br_response_t *resp)
{
    if (!onewire_search_poll()) {
        return 0u;
    }
    resp->status = BR_OK;
    resp->cmd = BR_CMD_OW_SEARCH_ALM;
    resp->len = 1u;
    resp->data[0] = cache_count();
    return 1u;
}

/* --- OW_SET_PULLUP / OW_SET_POWER ---------------------------------------- */

static void start_ow_set_pullup(const br_command_t *cmd)
{
    uint8_t on = (cmd->len > 0u) ? cmd->data[0] : 0u;
    pullup_hold = on;
    onewire_strong_pullup(on);
    if (config_get()->strong_pullup) {
        board_gate_pullup(on);
    }
}

static uint8_t poll_ow_set_pullup(br_response_t *resp)
{
    resp_simple(resp, BR_CMD_OW_SET_PULLUP, BR_OK);
    return 1u;
}

static void start_ow_set_power(const br_command_t *cmd)
{
    uint8_t parasite = (cmd->len > 0u) ? cmd->data[0] : 0u;
    ds18b20_set_parasite(parasite);
    ds2431_set_parasite(parasite);
}

static uint8_t poll_ow_set_power(br_response_t *resp)
{
    resp_simple(resp, BR_CMD_OW_SET_POWER, BR_OK);
    return 1u;
}

/* --- OW_GET_STATUS -------------------------------------------------------- */

static uint8_t poll_ow_get_status(br_response_t *resp)
{
    uint8_t flags = 0u;
    if (bus.state == BR_BUS_BUSY) {
        flags |= 0x01u;
    }
    if (ds18b20_parasite_mode()) {
        flags |= 0x08u;
    }
    resp->status = BR_OK;
    resp->cmd = BR_CMD_OW_GET_STATUS;
    resp->len = 2u;
    resp->data[0] = flags;
    resp->data[1] = cache_count();
    return 1u;
}

static void start_ow_get_status(const br_command_t *cmd)
{
    (void)cmd;
}

/* --- OW_GET_DEVICES ------------------------------------------------------- */

static void start_ow_get_devices(const br_command_t *cmd)
{
    (void)cmd;
}

static uint8_t poll_ow_get_devices(br_response_t *resp)
{
    uint8_t total = cache_count();
    uint8_t offset = (bus.cmd.len > 0u) ? bus.cmd.data[0] : 0u;
    uint8_t count = (bus.cmd.len > 1u) ? bus.cmd.data[1] : total;
    if (offset > total) {
        offset = total;
    }
    uint8_t n = (uint8_t)(total - offset);
    if (n > count) {
        n = count;
    }
    if (n != 0u) {
        fast_copy(resp->data, cache_roms() + (uint16_t)offset * 8u,
                  (uint16_t)n * 8u);
    }
    resp->status = BR_OK;
    resp->cmd = BR_CMD_OW_GET_DEVICES;
    resp->len = (uint8_t)(n * 8u);
    return 1u;
}

/* --- OW_GET_COUNT --------------------------------------------------------- */

static void start_ow_get_count(const br_command_t *cmd)
{
    (void)cmd;
}

static uint8_t poll_ow_get_count(br_response_t *resp)
{
    resp->status = BR_OK;
    resp->cmd = BR_CMD_OW_GET_COUNT;
    resp->len = 1u;
    resp->data[0] = cache_count();
    return 1u;
}

/* --- OW_CLEAR_CACHE ------------------------------------------------------- */

static void start_ow_clear_cache(const br_command_t *cmd)
{
    (void)cmd;
    cache_clear();
}

static uint8_t poll_ow_clear_cache(br_response_t *resp)
{
    resp_simple(resp, BR_CMD_OW_CLEAR_CACHE, BR_OK);
    return 1u;
}

/* --- OW_ABORT ------------------------------------------------------------- */

static void start_ow_abort(const br_command_t *cmd)
{
    (void)cmd;
}

static uint8_t poll_ow_abort(br_response_t *resp)
{
    /* The library has no abort for the in-flight hardware operation. This
       command runs after that operation has finished; bridge_submit() has
       already discarded anything that was queued behind it. */
    resp_simple(resp, BR_CMD_OW_ABORT, BR_ABORTED);
    return 1u;
}

/* ========================================================================= */
/* DS18B20 temperature commands                                             */
/* ========================================================================= */

/* --- TEMP_CONVERT --------------------------------------------------------- */

static void start_temp_convert(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds18b20_select(cmd->data);
    } else {
        ds18b20_select(0);
    }
    ds18b20_done = 0u;
    ds18b20_start_measure();
}

static uint8_t poll_temp_convert(br_response_t *resp)
{
    ds18b20_poll();
    if (!ds18b20_done) {
        return 0u;
    }
    if (ds18b20_temp == DS18B20_TEMP_ERROR_NO_SENSOR) {
        resp_simple(resp, BR_CMD_TEMP_CONVERT, BR_NO_DEVICE);
        return 1u;
    }
    if (ds18b20_temp == DS18B20_TEMP_ERROR_CRC_FAIL) {
        /* A broadcast conversion (Skip ROM) collides on the scratchpad read;
           the conversion itself completed, so report OK. A single-device
           conversion that fails CRC is a real error. */
        if (bus.cmd.len < 8u || bus.cmd.data[0] == 0u) {
            resp_simple(resp, BR_CMD_TEMP_CONVERT, BR_OK);
            return 1u;
        }
        stats_count(BR_STAT_CRC_ERROR);
        resp_simple(resp, BR_CMD_TEMP_CONVERT, BR_CRC_ERROR);
        return 1u;
    }
    if (ds18b20_temp == DS18B20_TEMP_ERROR_GENERIC) {
        resp_simple(resp, BR_CMD_TEMP_CONVERT, BR_BUS_ERROR);
        return 1u;
    }
    resp->status = BR_OK;
    resp->cmd = BR_CMD_TEMP_CONVERT;
    resp->len = 2u;
    resp->data[0] = (uint8_t)(ds18b20_temp >> 8);
    resp->data[1] = (uint8_t)(ds18b20_temp & 0xFFu);
    return 1u;
}

/* --- TEMP_READ ------------------------------------------------------------ */

static uint8_t temp_scratch[9];

static void start_temp_read(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds18b20_select(cmd->data);
    } else {
        ds18b20_select(0);
    }
    ds18b20_read_scratchpad(temp_scratch);
}

static uint8_t poll_temp_read(br_response_t *resp)
{
    if (!ds18b20_read_scratchpad_poll()) {
        return 0u;
    }
    if (!ds18b20_last_command_ok()) {
        stats_count(BR_STAT_PRESENCE_FAIL);
        resp_simple(resp, BR_CMD_TEMP_READ, BR_NO_DEVICE);
        return 1u;
    }
    int16_t t = decode_temp(temp_scratch);
    resp->status = BR_OK;
    resp->cmd = BR_CMD_TEMP_READ;
    resp->len = 2u;
    resp->data[0] = (uint8_t)(t >> 8);
    resp->data[1] = (uint8_t)(t & 0xFFu);
    return 1u;
}

/* --- TEMP_READ_ALL -------------------------------------------------------- */

static uint8_t read_all_index;
static uint8_t read_all_step;

static void start_temp_read_all(const br_command_t *cmd)
{
    (void)cmd;
    read_all_index = 0u;
    read_all_step = 0u;
    bus.resp.len = 0u;
}

static uint8_t poll_temp_read_all(br_response_t *resp)
{
    if (read_all_step == 0u) {
        while (read_all_index < cache_count()) {
            const uint8_t *rom = cache_get(read_all_index);
            read_all_index++;
            if (rom != 0 && rom[0] == DS18B20_FAMILY_CODE) {
                ds18b20_select(rom);
                ds18b20_read_scratchpad(temp_scratch);
                read_all_step = 1u;
                return 0u;
            }
        }
        resp->status = BR_OK;
        resp->cmd = BR_CMD_TEMP_READ_ALL;
        return 1u;
    }
    if (!ds18b20_read_scratchpad_poll()) {
        return 0u;
    }
    if (ds18b20_last_command_ok()) {
        int16_t t = decode_temp(temp_scratch);
        if ((uint16_t)resp->len + 2u <= BR_MAX_RESP) {
            resp->data[resp->len++] = (uint8_t)(t >> 8);
            resp->data[resp->len++] = (uint8_t)(t & 0xFFu);
        }
    }
    read_all_step = 0u;
    return 0u;
}

/* --- TEMP_SET_RES / TEMP_GET_RES ----------------------------------------- */

static void start_temp_set_res(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds18b20_select(cmd->data);
    } else {
        ds18b20_select(0);
    }
    uint8_t bits = (cmd->len > 8u) ? cmd->data[8] : DS18B20_RES_DEFAULT;
    ds18b20_set_resolution(bits);
}

static uint8_t poll_temp_set_res(br_response_t *resp)
{
    if (!ds18b20_set_resolution_poll()) {
        return 0u;
    }
    resp_simple(resp, BR_CMD_TEMP_SET_RES, BR_OK);
    return 1u;
}

static void start_temp_get_res(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds18b20_select(cmd->data);
    } else {
        ds18b20_select(0);
    }
    ds18b20_read_scratchpad(temp_scratch);
}

static uint8_t poll_temp_get_res(br_response_t *resp)
{
    if (!ds18b20_read_scratchpad_poll()) {
        return 0u;
    }
    resp->status = BR_OK;
    resp->cmd = BR_CMD_TEMP_GET_RES;
    resp->len = 1u;
    resp->data[0] = ds18b20_last_command_ok() ? scratch_resolution(temp_scratch) : 0u;
    return 1u;
}

/* --- TEMP_SET_ALARM / TEMP_GET_ALARM ------------------------------------- */

static void start_temp_set_alarm(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds18b20_select(cmd->data);
    } else {
        ds18b20_select(0);
    }
    uint8_t th = (cmd->len > 8u) ? cmd->data[8] : 0u;
    uint8_t tl = (cmd->len > 9u) ? cmd->data[9] : 0u;
    ds18b20_set_alarm_thresholds(th, tl);
}

static uint8_t poll_temp_set_alarm(br_response_t *resp)
{
    if (!ds18b20_set_alarm_thresholds_poll()) {
        return 0u;
    }
    resp_simple(resp, BR_CMD_TEMP_SET_ALARM, BR_OK);
    return 1u;
}

static void start_temp_get_alarm(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds18b20_select(cmd->data);
    } else {
        ds18b20_select(0);
    }
    ds18b20_read_scratchpad(temp_scratch);
}

static uint8_t poll_temp_get_alarm(br_response_t *resp)
{
    if (!ds18b20_read_scratchpad_poll()) {
        return 0u;
    }
    resp->status = BR_OK;
    resp->cmd = BR_CMD_TEMP_GET_ALARM;
    resp->len = 2u;
    resp->data[0] = ds18b20_last_command_ok() ? temp_scratch[2] : 0u;
    resp->data[1] = ds18b20_last_command_ok() ? temp_scratch[3] : 0u;
    return 1u;
}

/* --- TEMP_COPY / TEMP_RECALL / TEMP_DETECT_P ------------------------------ */

static void start_temp_copy(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds18b20_select(cmd->data);
    } else {
        ds18b20_select(0);
    }
    ds18b20_copy_scratchpad();
}

static uint8_t poll_temp_copy(br_response_t *resp)
{
    if (!ds18b20_copy_scratchpad_poll()) {
        return 0u;
    }
    resp_simple(resp, BR_CMD_TEMP_COPY, ds18b20_last_command_ok() ? BR_OK : BR_BUS_ERROR);
    return 1u;
}

static void start_temp_recall(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds18b20_select(cmd->data);
    } else {
        ds18b20_select(0);
    }
    ds18b20_recall_eeprom();
}

static uint8_t poll_temp_recall(br_response_t *resp)
{
    if (!ds18b20_recall_eeprom_poll()) {
        return 0u;
    }
    resp_simple(resp, BR_CMD_TEMP_RECALL, ds18b20_last_command_ok() ? BR_OK : BR_BUS_ERROR);
    return 1u;
}

static void start_temp_detect_p(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds18b20_select(cmd->data);
    } else {
        ds18b20_select(0);
    }
    ds18b20_detect_parasite();
}

static uint8_t poll_temp_detect_p(br_response_t *resp)
{
    if (!ds18b20_detect_parasite_poll()) {
        return 0u;
    }
    resp->status = BR_OK;
    resp->cmd = BR_CMD_TEMP_DETECT_P;
    resp->len = 1u;
    resp->data[0] = ds18b20_last_command_ok() ? ds18b20_parasite_mode() : 0u;
    return 1u;
}

/* ========================================================================= */
/* DS2431 EEPROM commands                                                   */
/* ========================================================================= */

/* --- EEPROM_READ ---------------------------------------------------------- */

static uint8_t eeprom_dst[DS2431_READ_PASS];

static void start_eeprom_read(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds2431_select(cmd->data);
    } else {
        ds2431_select(0);
    }
    uint16_t addr = (cmd->len > 8u) ? cmd->data[8] : 0u;
    uint8_t len = (cmd->len > 9u) ? cmd->data[9] : 1u;
    if (len > DS2431_READ_PASS) {
        len = DS2431_READ_PASS;
    }
    ds2431_read_memory(eeprom_dst, addr, len);
}

static uint8_t poll_eeprom_read(br_response_t *resp)
{
    if (!ds2431_read_memory_poll()) {
        return 0u;
    }
    uint8_t len = (bus.cmd.len > 9u) ? bus.cmd.data[9] : 1u;
    if (len > DS2431_READ_PASS) {
        len = DS2431_READ_PASS;
    }
    resp->status = ds2431_last_command_ok() ? BR_OK : BR_NO_DEVICE;
    resp->cmd = BR_CMD_EEPROM_READ;
    resp->len = len;
    fast_copy(resp->data, eeprom_dst, len);
    return 1u;
}

/* --- EEPROM_WRITE --------------------------------------------------------- */

static uint8_t eeprom_step;
static uint8_t eeprom_error;
static uint8_t eeprom_scratch[DS2431_SCRATCH_FRAME];

static void start_eeprom_write(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds2431_select(cmd->data);
    } else {
        ds2431_select(0);
    }
    uint16_t addr = (cmd->len > 8u) ? cmd->data[8] : 0u;
    uint8_t len = (cmd->len > 9u) ? (uint8_t)(cmd->len - 9u) : 0u;
    eeprom_error = 0u;
    eeprom_step = 0u;
    if (len == 0u || len > DS2431_SCRATCH_SIZE) {
        eeprom_error = 1u;
        return;
    }
    ds2431_write_scratchpad(addr, cmd->data + 9u, len);
}

static uint8_t poll_eeprom_write(br_response_t *resp)
{
    if (eeprom_error) {
        resp_simple(resp, BR_CMD_EEPROM_WRITE, BR_PARAM_ERROR);
        return 1u;
    }
    switch (eeprom_step) {
    case 0u:
        if (!ds2431_write_scratchpad_poll()) {
            return 0u;
        }
        ds2431_read_scratchpad(eeprom_scratch);
        eeprom_step = 1u;
        return 0u;
    case 1u:
        if (!ds2431_read_scratchpad_poll()) {
            return 0u;
        }
        ds2431_copy_scratchpad();
        eeprom_step = 2u;
        return 0u;
    case 2u:
        if (!ds2431_copy_scratchpad_poll()) {
            return 0u;
        }
        resp_simple(resp, BR_CMD_EEPROM_WRITE,
                    ds2431_last_command_ok() ? BR_OK : BR_BUS_ERROR);
        return 1u;
    default:
        resp_simple(resp, BR_CMD_EEPROM_WRITE, BR_BUS_ERROR);
        return 1u;
    }
}

/* --- EEPROM_READ_S -------------------------------------------------------- */

static void start_eeprom_read_s(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds2431_select(cmd->data);
    } else {
        ds2431_select(0);
    }
    ds2431_read_scratchpad(eeprom_scratch);
}

static uint8_t poll_eeprom_read_s(br_response_t *resp)
{
    if (!ds2431_read_scratchpad_poll()) {
        return 0u;
    }
    resp->status = ds2431_last_command_ok() ? BR_OK : BR_NO_DEVICE;
    resp->cmd = BR_CMD_EEPROM_READ_S;
    resp->len = DS2431_SCRATCH_FRAME;
    fast_copy(resp->data, eeprom_scratch, DS2431_SCRATCH_FRAME);
    return 1u;
}

/* ========================================================================= */
/* DS2413 PIO commands                                                      */
/* ========================================================================= */

/* --- IO_READ -------------------------------------------------------------- */

static uint8_t io_state;

static void start_io_read(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds2413_select(cmd->data);
    } else {
        ds2413_select(0);
    }
    ds2413_read_pio(&io_state);
}

static uint8_t poll_io_read(br_response_t *resp)
{
    if (!ds2413_read_pio_poll()) {
        return 0u;
    }
    resp->status = ds2413_last_command_ok() ? BR_OK : BR_NO_DEVICE;
    resp->cmd = BR_CMD_IO_READ;
    resp->len = 2u;
    resp->data[0] = ds2413_last_pio();
    resp->data[1] = ds2413_last_latches();
    return 1u;
}

/* --- IO_WRITE ------------------------------------------------------------- */

static void start_io_write(const br_command_t *cmd)
{
    if (cmd->len >= 8u && cmd->data[0] != 0u) {
        ds2413_select(cmd->data);
    } else {
        ds2413_select(0);
    }
    uint8_t states = (cmd->len > 8u) ? cmd->data[8] : 0u;
    ds2413_write_pio(states);
}

static uint8_t poll_io_write(br_response_t *resp)
{
    if (!ds2413_write_pio_poll()) {
        return 0u;
    }
    resp_simple(resp, BR_CMD_IO_WRITE, ds2413_last_command_ok() ? BR_OK : BR_BUS_ERROR);
    return 1u;
}

/* ========================================================================= */
/* Service commands                                                         */
/* ========================================================================= */

/* --- CONFIG_GET / CONFIG_SET --------------------------------------------- */

static void start_config_get(const br_command_t *cmd)
{
    (void)cmd;
}

static uint8_t poll_config_get(br_response_t *resp)
{
    const struct br_config *cfg = config_get();
    resp->status = BR_OK;
    resp->cmd = BR_CMD_CONFIG_GET;
    resp->len = BR_CONFIG_SIZE;
    fast_copy(resp->data, cfg, BR_CONFIG_SIZE);
    return 1u;
}

static void start_config_set(const br_command_t *cmd)
{
    (void)cmd;
}

static uint8_t poll_config_set(br_response_t *resp)
{
    if (bus.cmd.len != BR_CONFIG_SIZE) {
        resp_simple(resp, BR_CMD_CONFIG_SET, BR_PARAM_ERROR);
        return 1u;
    }
    struct br_config cfg;
    fast_copy(&cfg, bus.cmd.data, BR_CONFIG_SIZE);
    uint8_t ok = config_set(&cfg);
    resp_simple(resp, BR_CMD_CONFIG_SET, ok ? BR_OK : BR_PARAM_ERROR);
    return 1u;
}

/* --- FIRMWARE_VER --------------------------------------------------------- */

static void start_firmware_ver(const br_command_t *cmd)
{
    (void)cmd;
}

static uint8_t poll_firmware_ver(br_response_t *resp)
{
    static const uint8_t ver[] __attribute__((aligned(4))) =
        "owbridge 1.0.0 (lib 3.1.2)";
    resp->status = BR_OK;
    resp->cmd = BR_CMD_FIRMWARE_VER;
    resp->len = (uint8_t)sizeof(ver) - 1u;
    fast_copy(resp->data, ver, resp->len);
    return 1u;
}

/* --- RESET ---------------------------------------------------------------- */

static void start_reset(const br_command_t *cmd)
{
    (void)cmd;
}

static uint8_t poll_reset(br_response_t *resp)
{
    resp_simple(resp, BR_CMD_RESET, BR_OK);
    board_reset(); /* does not return; the response may not reach the host */
    return 1u;
}

/* --- STATS_GET / STATS_RESET --------------------------------------------- */

static void start_stats_get(const br_command_t *cmd)
{
    (void)cmd;
}

static uint8_t poll_stats_get(br_response_t *resp)
{
    uint8_t n = 0u;
    for (uint8_t i = 0u; i < BR_STAT_COUNT; i++) {
        uint32_t v = stats_get(i);
        resp->data[n++] = (uint8_t)(v >> 24);
        resp->data[n++] = (uint8_t)(v >> 16);
        resp->data[n++] = (uint8_t)(v >> 8);
        resp->data[n++] = (uint8_t)(v);
    }
    uint32_t up = stats_uptime();
    resp->data[n++] = (uint8_t)(up >> 24);
    resp->data[n++] = (uint8_t)(up >> 16);
    resp->data[n++] = (uint8_t)(up >> 8);
    resp->data[n++] = (uint8_t)(up);
    resp->status = BR_OK;
    resp->cmd = BR_CMD_STATS_GET;
    resp->len = n;
    return 1u;
}

static void start_stats_reset(const br_command_t *cmd)
{
    (void)cmd;
    stats_reset();
}

static uint8_t poll_stats_reset(br_response_t *resp)
{
    resp_simple(resp, BR_CMD_STATS_RESET, BR_OK);
    return 1u;
}

/* ========================================================================= */
/* Command table                                                            */
/* ========================================================================= */

/* Indexed by the command byte itself. Sixteen free kilobytes are worth far
   more than a 2 KB table: the dispatcher becomes one shift and one load
   instead of a linear scan over 38 entries. Unused slots stay zero, checked
   through the non-null poll pointer. */
static const br_cmd_desc_t cmd_map[256] = {
    [BR_CMD_OW_RESET]       = { start_ow_reset,       poll_ow_reset },
    [BR_CMD_OW_WRITE_BYTE]  = { start_ow_write_byte,  poll_ow_write_byte },
    [BR_CMD_OW_READ_BYTE]   = { start_ow_read_byte,   poll_ow_read_byte },
    [BR_CMD_OW_WRITE_BYTES] = { start_ow_write_bytes, poll_ow_write_bytes },
    [BR_CMD_OW_READ_BYTES]  = { start_ow_read_bytes,  poll_ow_read_bytes },
    [BR_CMD_OW_MATCH_ROM]   = { start_ow_match,       poll_ow_match },
    [BR_CMD_OW_SKIP_ROM]    = { start_ow_skip,        poll_ow_skip },
    [BR_CMD_OW_READ_ROM]    = { start_ow_read_rom,    poll_ow_read_rom },
    [BR_CMD_OW_SEARCH]      = { start_ow_search,      poll_ow_search },
    [BR_CMD_OW_SEARCH_ALM]  = { start_ow_search_alm,  poll_ow_search_alm },
    [BR_CMD_OW_SET_PULLUP]  = { start_ow_set_pullup,  poll_ow_set_pullup,  1u },
    [BR_CMD_OW_SET_POWER]   = { start_ow_set_power,   poll_ow_set_power,   1u },
    [BR_CMD_OW_GET_STATUS]  = { start_ow_get_status,  poll_ow_get_status,  1u },
    [BR_CMD_OW_GET_DEVICES] = { start_ow_get_devices, poll_ow_get_devices, 1u },
    [BR_CMD_OW_GET_COUNT]   = { start_ow_get_count,   poll_ow_get_count,   1u },
    [BR_CMD_OW_CLEAR_CACHE] = { start_ow_clear_cache, poll_ow_clear_cache, 1u },
    [BR_CMD_OW_ABORT]       = { start_ow_abort,       poll_ow_abort,       1u },

    [BR_CMD_TEMP_CONVERT]   = { start_temp_convert,   poll_temp_convert },
    [BR_CMD_TEMP_READ]      = { start_temp_read,      poll_temp_read },
    [BR_CMD_TEMP_READ_ALL]  = { start_temp_read_all,  poll_temp_read_all },
    [BR_CMD_TEMP_SET_RES]   = { start_temp_set_res,   poll_temp_set_res },
    [BR_CMD_TEMP_GET_RES]   = { start_temp_get_res,   poll_temp_get_res },
    [BR_CMD_TEMP_SET_ALARM] = { start_temp_set_alarm, poll_temp_set_alarm },
    [BR_CMD_TEMP_GET_ALARM] = { start_temp_get_alarm, poll_temp_get_alarm },
    [BR_CMD_TEMP_COPY]      = { start_temp_copy,      poll_temp_copy },
    [BR_CMD_TEMP_RECALL]    = { start_temp_recall,    poll_temp_recall },
    [BR_CMD_TEMP_DETECT_P]  = { start_temp_detect_p,  poll_temp_detect_p },

    [BR_CMD_EEPROM_READ]    = { start_eeprom_read,    poll_eeprom_read },
    [BR_CMD_EEPROM_WRITE]   = { start_eeprom_write,   poll_eeprom_write },
    [BR_CMD_EEPROM_READ_S]  = { start_eeprom_read_s,  poll_eeprom_read_s },

    [BR_CMD_IO_READ]        = { start_io_read,        poll_io_read },
    [BR_CMD_IO_WRITE]       = { start_io_write,       poll_io_write },

    [BR_CMD_CONFIG_GET]     = { start_config_get,     poll_config_get,     1u },
    [BR_CMD_CONFIG_SET]     = { start_config_set,     poll_config_set },
    [BR_CMD_FIRMWARE_VER]   = { start_firmware_ver,   poll_firmware_ver,   1u },
    [BR_CMD_RESET]          = { start_reset,          poll_reset },
    [BR_CMD_STATS_GET]      = { start_stats_get,      poll_stats_get,      1u },
    [BR_CMD_STATS_RESET]    = { start_stats_reset,    poll_stats_reset,    1u },
};

static inline const br_cmd_desc_t *find_desc(uint8_t cmd)
{
    const br_cmd_desc_t *d = &cmd_map[cmd];
    return (d->poll != 0) ? d : 0;
}

/* ========================================================================= */
/* Dispatcher                                                               */
/* ========================================================================= */

static void bus_start_next(void)
{
    if (queue_count == 0u) {
        bus.state = BR_BUS_IDLE;
        bus.desc = 0;
        return;
    }
    br_command_t *src = &queue[queue_head];
    uint8_t cmd = src->cmd;
    const br_cmd_desc_t *desc = find_desc(cmd);
    if (desc == 0) {
        bus.resp.status = BR_INVALID_CMD;
        bus.resp.cmd = cmd;
        bus.resp.len = 0u;
        queue_head = (uint8_t)((queue_head + 1u) % BR_QUEUE_SIZE);
        queue_count--;
        resp_commit();
        led_result(BR_INVALID_CMD);
        bus.state = BR_BUS_DONE;
        return;
    }

    bus.cmd.cmd = cmd;
    bus.cmd.len = src->len;
    if (src->len != 0u) {
        fast_copy(bus.cmd.data, src->data, src->len);
    }
    queue_head = (uint8_t)((queue_head + 1u) % BR_QUEUE_SIZE);
    queue_count--;

    bus_begin_command(desc, cmd);
}

uint8_t bridge_submit(const uint8_t *frame, uint8_t len)
{
    if (frame == 0 || len < 2u) {
        return 0u;
    }
    uint8_t cmd = frame[0];
    uint8_t dlen = frame[1];
    if (dlen > BR_MAX_DATA || len < (uint8_t)(2u + dlen)) {
        return 0u;
    }
    if (cmd == BR_CMD_OW_ABORT) {
        /* The library cannot cancel the in-flight hardware operation, but we
           can drop everything that was queued behind it. */
        queue_count = 0u;
        queue_head = 0u;
        queue_tail = 0u;
    }
    if (queue_count >= BR_QUEUE_SIZE) {
        bus.resp.status = BR_QUEUE_FULL;
        bus.resp.cmd = cmd;
        bus.resp.len = 0u;
        resp_commit();
        led_result(BR_QUEUE_FULL);
        return 0u;
    }
    if (bus.state == BR_BUS_IDLE && queue_count == 0u) {
        const br_cmd_desc_t *desc = find_desc(cmd);
        if (desc == 0) {
            bus.resp.status = BR_INVALID_CMD;
            bus.resp.cmd = cmd;
            bus.resp.len = 0u;
            resp_commit();
            led_result(BR_INVALID_CMD);
            bus.state = BR_BUS_DONE;
            return 1u;
        }
        bus.cmd.cmd = cmd;
        bus.cmd.len = dlen;
        if (dlen != 0u) {
            fast_copy(bus.cmd.data, frame + 2u, dlen);
        }
        bus_begin_command(desc, cmd);
        return 1u;
    }

    queue[queue_tail].cmd = cmd;
    queue[queue_tail].len = dlen;
    if (dlen != 0u) {
        fast_copy(queue[queue_tail].data, frame + 2u, dlen);
    }
    queue_tail = (uint8_t)((queue_tail + 1u) % BR_QUEUE_SIZE);
    queue_count++;
    resp_busy(cmd);
    return 1u;
}

void bridge_poll(void)
{
    if (bus.state == BR_BUS_BUSY) {
        if (bus.desc != 0 && bus.desc->poll(&bus.resp)) {
            bus_finish_command();
        }
    } else if (bus.state == BR_BUS_DONE) {
        if (bus.account_pending) {
            bus.account_pending = 0u;
            stats_count(BR_STAT_COMMANDS);
            led_result(bus.resp.status);
        }
        /* The response is already latched in last_resp; start the next
           queued command. */
        bus.state = BR_BUS_IDLE;
        bus_start_next();
    }
}

uint8_t bridge_get_response(uint8_t *out)
{
    if (out == 0) {
        return 0u;
    }
    if (last_resp_len > 0u) {
        fast_copy(out, last_resp, last_resp_len);
    }
    return last_resp_len;
}

uint8_t bridge_response_seq(void)
{
    return last_resp_seq;
}

const uint8_t *bridge_response_ptr(void)
{
    return last_resp;
}

uint8_t bridge_response_len(void)
{
    return last_resp_len;
}

void bridge_abort(void)
{
    if (bus.state == BR_BUS_BUSY) {
        stats_count(BR_STAT_ABORTS);
    }
    bus.state = BR_BUS_IDLE;
    bus.account_pending = 0u;
    bus.desc = 0;
    queue_count = 0u;
    queue_head = 0u;
    queue_tail = 0u;
    bus.resp.status = BR_ABORTED;
    bus.resp.cmd = 0u;
    bus.resp.len = 0u;
    resp_commit();
}

uint8_t bridge_busy(void)
{
    return (bus.state == BR_BUS_BUSY) ? 1u : 0u;
}

/* ========================================================================= */
/* Init                                                                    */
/* ========================================================================= */

void bridge_init(void)
{
    cache_clear();
    stats_init();
    config_init();

    bus.state = BR_BUS_IDLE;
    bus.account_pending = 0u;
    bus.desc = 0;
    bus.resp.data = last_resp + 3u;
    pullup_hold = 0u;
    queue_head = queue_tail = queue_count = 0u;

    last_resp[0] = BR_OK;
    last_resp[1] = 0u;
    last_resp[2] = 0u;
    last_resp_len = 3u;
    last_resp_seq = 0u;

    ds18b20_init();
    ds2431_init();
    ds2413_init();

    const struct br_config *cfg = config_get();
    ds18b20_set_parasite(cfg->parasite);
    ds2431_set_parasite(cfg->parasite);
}
