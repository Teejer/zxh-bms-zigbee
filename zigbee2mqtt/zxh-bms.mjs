// zigbee2mqtt external converter for the ZXH-BMS multi-pack gateway.
//
// Install: save as `external_converters/zxh-bms.mjs` next to your
// configuration.yaml, set MAX_PACKS to match the firmware's ZXH_PACK_COUNT
// in zxh_config.h, restart z2m, then (device page -> Dev console or General
// tab) hit "Reconfigure" (or re-interview) once so bindings land.
//
// z2m >= 2.11: external converters must be enabled (enable_external_js).

import {Zcl} from 'zigbee-herdsman';
import {deviceAddCustomCluster} from 'zigbee-herdsman-converters/lib/modernExtend';
import {access as ea, presets as e} from 'zigbee-herdsman-converters/lib/exposes';
import * as reporting from 'zigbee-herdsman-converters/lib/reporting';

export const MAX_PACKS = 5; // must match the firmware build

const ZXH_CLUSTER = 0xff10;

// attr id -> [name, zcl type] (must match firmware's zxh_zigbee.c)
const ATTR = {
    0x0001: ['voltage', Zcl.DataType.UINT16], // centivolts
    0x0002: ['current', Zcl.DataType.INT16], // centiamperes
    0x0003: ['power', Zcl.DataType.INT16], // watts
    0x0004: ['soc', Zcl.DataType.UINT8],
    0x0005: ['cellMin', Zcl.DataType.UINT16],
    0x0006: ['cellMax', Zcl.DataType.UINT16],
    0x0007: ['cellDelta', Zcl.DataType.UINT16],
    0x0008: ['mosTemp', Zcl.DataType.INT16],
    0x0009: ['temp1', Zcl.DataType.INT16],
    0x000a: ['temp2', Zcl.DataType.INT16],
    0x000b: ['cycles', Zcl.DataType.UINT16],
    0x000c: ['health', Zcl.DataType.UINT8],
    0x000d: ['capacity', Zcl.DataType.UINT16], // 0.1 Ah remaining
    0x000e: ['fullCap', Zcl.DataType.UINT16],
    0x000f: ['cellCount', Zcl.DataType.UINT8],
    0x0010: ['protection', Zcl.DataType.BITMAP16],
    0x0011: ['equilibrium', Zcl.DataType.UINT32],
    0x0012: ['cellMv', Zcl.DataType.OCTET_STR],
    0x0013: ['online', Zcl.DataType.UINT8],
};

const clusterDefinition = {
    ID: ZXH_CLUSTER,
    name: 'ZXHBMS',
    manufacturerCode: null,
    attributes: Object.fromEntries(
        Object.entries(ATTR).map(([id, [name, type]]) => [name, {ID: Number(id), type}]),
    ),
    commands: {},
    commandsResponse: {},
};

// Protection bitmask (bit index -> label), matching the bms-cli project.
const PROTECTION = {
    0: 'cell voltage difference large', 1: 'voltage detect line open', 2: 'MOS high temp',
    3: 'board locked', 4: 'chip failure', 5: 'short circuit', 6: 'discharge overcurrent',
    7: 'charge overcurrent', 8: 'discharge low temp', 9: 'discharge high temp',
    10: 'charge low temp', 11: 'charge high temp', 14: 'cell undervoltage',
    15: 'cell overvoltage',
};

function protectionNames(bitmap) {
    const names = [];
    for (const [bit, label] of Object.entries(PROTECTION)) {
        if (bitmap & (1 << Number(bit))) names.push(label);
    }
    return names.length ? names.join(', ') : 'none';
}

function decodeCells(buf) {
    // octet string: [len][mv_hi, mv_lo] * n
    if (!buf || !buf.length) return '';
    const n = Math.min(Math.floor((buf[0] || buf.length - 1) / 2), 32);
    const cells = [];
    for (let i = 0; i < n; i++) cells.push(buf[1 + 2 * i] * 256 + buf[2 + 2 * i]);
    return cells.join(',');
}

const fz = {
    zxbms: {
        cluster: 'ZXHBMS',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const d = msg.data;
            const r = {};
            const has = (k) => d[k] !== undefined;
            if (has('voltage')) r.voltage = d.voltage / 100;
            if (has('current')) r.current = d.current / 100;
            if (has('power')) r.power = d.power;
            if (has('soc')) r.soc = d.soc;
            if (has('cellMin')) r.cell_min = d.cellMin;
            if (has('cellMax')) r.cell_max = d.cellMax;
            if (has('cellDelta')) r.cell_delta = d.cellDelta;
            if (has('mosTemp')) r.mos_temperature = d.mosTemp / 10;
            if (has('temp1')) r.temperature_1 = d.temp1 / 10;
            if (has('temp2')) r.temperature_2 = d.temp2 / 10;
            if (has('cycles')) r.cycles = d.cycles;
            if (has('health')) r.health = d.health;
            if (has('capacity')) r.capacity = d.capacity / 10;
            if (has('fullCap')) r.full_capacity = d.fullCap / 10;
            if (has('cellCount')) r.cell_count = d.cellCount;
            if (has('protection')) {
                r.protection = d.protection;
                r.protection_state = protectionNames(d.protection);
            }
            if (has('equilibrium')) {
                r.equilibrium = d.equilibrium;
                r.balancing = d.equilibrium !== 0;
            }
            if (has('cellMv')) r.cells = decodeCells(d.cellMv);
            if (has('online')) r.online = d.online === 1;
            return r;
        },
    },
};

function packExposes(n) {    const ep = `pack_${n}`;
    const num = (name, unit) => e.numeric(name, ea.STATE).withEndpoint(ep).withUnit(unit);
    return [
        num('voltage', 'V'),
        num('current', 'A'),
        num('power', 'W'),
        num('soc', '%'),
        num('cell_min', 'mV'),
        num('cell_max', 'mV'),
        num('cell_delta', 'mV'),
        num('mos_temperature', '°C'),
        num('temperature_1', '°C'),
        num('temperature_2', '°C'),
        num('cycles', ''),
        num('health', '%'),
        num('capacity', 'Ah'),
        num('full_capacity', 'Ah'),
        num('cell_count', ''),
        num('protection', ''),
        num('equilibrium', ''),
        e.text('cells', ea.STATE).withEndpoint(ep),
        e.text('protection_state', ea.STATE).withEndpoint(ep),
        e.binary('balancing', ea.STATE, true, false).withEndpoint(ep),
        e.binary('online', ea.STATE, true, false).withEndpoint(ep),
    ];
}

// Register the custom cluster on the herdsman Device object. The decoder for
// incoming frames reads `device.customClusters`, and it is NOT populated
// reliably by the extend alone: onEvent('start') only runs on z2m boot, and
// configure() runs after the first frames may already have arrived. Call it
// everywhere cheap; the method is idempotent.
function registerCluster(device) {
    try {
        device.addCustomCluster('ZXHBMS', clusterDefinition);
    } catch (e) {
        // already registered / concurrent — safe to ignore
    }
}

const definition = {
    zigbeeModel: ['ZXH-BMS'],
    model: 'ZXH-BMS-1',
    vendor: 'zxh',
    description: `ZXH BMS multi-pack gateway (${MAX_PACKS} LiFePO4 packs over BLE, one Zigbee endpoint each)`,
    extend: [deviceAddCustomCluster('ZXHBMS', clusterDefinition)],
    onEvent: (event) => {
        // hc 26 signature: single handler receiving {type, data:{device}}.
        // Must run on start (each z2m boot — customClusters are not
        // persisted) and again after interview/announce so freshly paired
        // devices decode immediately.
        const dev = event?.data?.device;
        if (dev && ['start', 'deviceInterview', 'deviceAnnounce', 'deviceJoined'].includes(event.type)) {
            registerCluster(dev);
        }
    },
    fromZigbee: [fz.zxbms],
    toZigbee: [],
    exposes: [].concat(...Array.from({length: MAX_PACKS}, (_, i) => packExposes(i + 1))),
    endpoint: (device) => {
        const map = {};
        for (const ep of device.endpoints) {
            if (ep.ID >= 1 && ep.ID <= MAX_PACKS) map[ep.ID] = `pack_${ep.ID}`;
        }
        return map;
    },
    configure: async (device, coordinatorEndpoint) => {
        registerCluster(device);
        // NOTE: the hc configure signature is (device, coordinatorEndpoint,
        // definition) — the third arg is NOT a logger (my earlier bug). The
        // device firmware pushes changed attributes on its own, so bind +
        // configureReporting are best-effort reliability only; never throw,
        // or z2m retries configure forever.
        const attrs = Object.entries(ATTR).map(([attrId, [name]]) => ({
            attribute: name,
            minimumReportInterval: 1,
            maximumReportInterval: 300,
            reportableChange: 0,
        }));
        for (const ep of device.endpoints) {
            if (ep.ID < 1 || ep.ID > MAX_PACKS) continue;
            try {
                await reporting.bind(ep, coordinatorEndpoint, ['ZXHBMS']);
            } catch (e) {
                // no coordinator binding; device unicasts reports anyway
            }
            try {
                await ep.configureReporting('ZXHBMS', attrs);
            } catch (e) {
                // not fatal: firmware self-reports on change
            }
        }
    },
};

export default definition;
