// Runs a single-core UART test image in the rp2040js emulator and prints
// the console (UART0).  See tools/emulator/README.md.
//   node run_tests.cjs <image.bin> [max_wall_seconds]
const { RP2040, ConsoleLogger, LogLevel } = require('rp2040js');
const { bootromB1 } = require('./bootrom.cjs');
const fs = require('fs');

const mcu = new RP2040();
mcu.loadBootrom(bootromB1);
mcu.logger = new ConsoleLogger(LogLevel.Error);
mcu.flash.set(fs.readFileSync(process.argv[2]), 0);

let out = '';
mcu.uart[0].onByte = (v) => { const c = String.fromCharCode(v); out += c; process.stdout.write(c); };
mcu.core.PC = 0x10000000;

const limitMs = 1000 * Number(process.argv[3] || 120);
const t0 = Date.now();
while (!out.includes('test run end') && (Date.now() - t0) < limitMs) {
  for (let i = 0; i < 100000; i++) mcu.step();
}
const failed = (out.match(/\[TEST\] \S+ FAIL/g) || []).length;
const ended = out.includes('test run end');
console.log(`\n[emulator] ${ended ? 'finished' : 'TIMED OUT'}, ${failed} FAIL line(s)`);
process.exit(ended && failed === 0 ? 0 : 1);
