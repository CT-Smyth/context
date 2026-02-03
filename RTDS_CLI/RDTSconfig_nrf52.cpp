#include "RDTSconfig.h"
#include <string.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "RDTSserver.h"
#include "RDTSpacket.h"

using namespace Adafruit_LittleFS_Namespace;

static InternalFileSystem fs;

bool rdtscfg_load(rdts_config_t* out)
{
    if (!fs.begin()) return false;

    File f = fs.open("rdts.cfg", FILE_O_READ);
    if (!f) return false;

    bool ok = true;

    if (f.read(out, sizeof(*out)) != sizeof(*out)) ok = false;
    else if (out->magic != RDTS_CONFIG_MAGIC) ok = false;
    else if (out->version != RDTS_CONFIG_VERSION) ok = false;
    else if (out->size != sizeof(*out)) ok = false;

    f.close();
    return ok;
}

void rdtscfg_save(const rdts_config_t* cfg)
{
    if (!fs.begin()) return;

    fs.remove("rdts.cfg");  // required: no FILE_O_TRUNC support

    File f = fs.open("rdts.cfg", FILE_O_WRITE);
    if (!f) return;

    f.write((const uint8_t*)cfg, sizeof(*cfg));
    f.close();
}

void rdtscfg_set_defaults(rdts_config_t* cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->magic = RDTS_CONFIG_MAGIC;
    cfg->version = RDTS_CONFIG_VERSION;
    cfg->size = sizeof(*cfg);

    cfg->beacon_period_ms = 3000;
    cfg->beacon_burst_len = 100;
    cfg->beacon_burst_span_ms = 1000;
    cfg->tx_power_dbm = 8;
    cfg->auth_mode = RDTSM_AUTH_DEV;
    cfg->default_mode = RDTS_MODE_IDLE;
}