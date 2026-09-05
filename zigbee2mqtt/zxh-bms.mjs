// zigbee2mqtt external converter for the ZXH-BMS multi-pack gateway.
//
// Install: save as `external_converters/zxh-bms.mjs` next to your
// configuration.yaml (z2m >= 2.11 also needs `external_converters` enabled:
// https://www.zigbee2mqtt.io/guide/configuration/all-settings.html#enable-external-js),
// set MAX_PACKS below to match ZXH_PACK_COUNT in the firmware zxh_config.h,
// restart z2m, then pair (or re-interview) the gateway device.
//
// The firmware pushes an explicit attribute report for every value after
// each BLE poll cycle, so no configureReporting is required.

import {access as ea, presets as e} from 'zigbee-herdsman-converters/lib/exposes';

export const MAX_PACKS = 5; // must match the firmware build

const ZXH_CLUSTER = 0xff10;
const ATTR = {
    voltage: 0x0001, current: 0x0002, power: 0x0003, soc: 0x0004,
    cellMin: 0x0005, cellMax: 0x0006, cellDelta: 0x0007,
    mosTemp: 0x0008, temp1: 0x0009, temp2: 0x000a,
    cycles: 0x000b, health: 0x000c, capacity: 0x000d, fullCap: 0x000e,
    cellCount: 0x000f, protection: 0x0010, equilibrium: 0x0011,
    cellMv: 0x0012, online: 0x0013,
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
        cluster: ZXH_CLUSTER,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const d = msg.data;
            const r = {};
            if (ATTR.voltage in d) r.voltage = d[ATTR.voltage] / 100;
            if (ATTR.current in d) r.current = d[ATTR.current] / 100;
            if (ATTR.power in d) r.power = d[ATTR.power];
            if (ATTR.soc in d) r.soc = d[ATTR.soc];
            if (ATTR.cellMin in d) r.cell_min = d[ATTR.cellMin];
            if (ATTR.cellMax in d) r.cell_max = d[ATTR.cellMax];
            if (ATTR.cellDelta in d) r.cell_delta = d[ATTR.cellDelta];
            if (ATTR.mosTemp in d) r.mos_temperature = d[ATTR.mosTemp] / 10;
            if (ATTR.temp1 in d) r.temperature_1 = d[ATTR.temp1] / 10;
            if (ATTR.temp2 in d) r.temperature_2 = d[ATTR.temp2] / 10;
            if (ATTR.cycles in d) r.cycles = d[ATTR.cycles];
            if (ATTR.health in d) r.health = d[ATTR.health];
            if (ATTR.capacity in d) r.capacity = d[ATTR.capacity] / 10;
            if (ATTR.fullCap in d) r.full_capacity = d[ATTR.fullCap] / 10;
            if (ATTR.cellCount in d) r.cell_count = d[ATTR.cellCount];
            if (ATTR.protection in d) {
                r.protection = d[ATTR.protection];
                r.protection_state = protectionNames(d[ATTR.protection]);
            }
            if (ATTR.equilibrium in d) {
                r.equilibrium = d[ATTR.equilibrium];
                r.balancing = d[ATTR.equilibrium] !== 0;
            }
            if (ATTR.cellMv in d) r.cells = decodeCells(d[ATTR.cellMv]);
            if (ATTR.online in d) r.online = d[ATTR.online] === 1;
            return r;
        },
    },
};

function packExposes(n) {
    const ep = `pack_${n}`;
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

const definition = {
    zigbeeModel: ['ZXH-BMS'],
    model: 'ZXH-BMS-1',
    vendor: 'zxh',
    description: `ZXH BMS multi-pack gateway (${MAX_PACKS} LiFePO4 packs over BLE, one Zigbee endpoint each)`,
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
};

export default definition;
