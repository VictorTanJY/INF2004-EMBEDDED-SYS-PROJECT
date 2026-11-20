// Runs a single-core UART test image in the rp2040js emulator and prints
// the console (UART0).  See tools/emulator/README.md.
//   node run_tests.cjs <image.bin> [max_wall_seconds]
//
// The simulated clock is ticked by the cycles of every instruction, and
// skipped to the next timer alarm while the core sleeps in WFI (the
// kernel's idle task) -- the same loop as rp2040js's own Simulator.
// RP2040.step() alone executes an instruction WITHOUT advancing time, so
// any test that sleeps (tk_dly_tsk) would never wake up.
const { Simulator, ConsoleLogger, LogLevel } = require('rp2040js');
const { bootromB1 } = require('./bootrom.cjs');
const fs = require('fs');

// Simulator owns the RP2040 and its SimulationClock (the clock class is
// not exported by rp2040js 1.1.1); we drive the loop ourselves.
const sim = new Simulator();
const mcu = sim.rp2040;
const clock = sim.clock;
mcu.loadBootrom(bootromB1);
mcu.logger = new ConsoleLogger(LogLevel.Error);
mcu.flash.set(fs.readFileSync(process.argv[2]), 0);

let out = '';
mcu.uart[0].onByte = (v) => { const c = String.fromCharCode(v); out += c; process.stdout.write(c); };
mcu.core.PC = 0x10000000;

const cycleNanos = 1e9 / 125000000;     // 125 MHz
const limitMs = 1000 * Number(process.argv[3] || 300);
const t0 = Date.now();
while (!out.includes('test run end') && (Date.now() - t0) < limitMs) {
  for (let i = 0; i < 100000; i++) {
    if (mcu.core.waiting) {
      clock.tick(clock.nanosToNextAlarm);
    } else {
      clock.tick(mcu.core.executeInstruction() * cycleNanos);
    }
  }
}
const failed = (out.match(/\[TEST\] \S+ FAIL/g) || []).length;
const ended = out.includes('test run end');
console.log(`\n[emulator] ${ended ? 'finished' : 'TIMED OUT'}, ${failed} FAIL line(s), ` +
            `${(Number(clock.nanos) / 1e9).toFixed(2)} s simulated`);
process.exit(ended && failed === 0 ? 0 : 1);
