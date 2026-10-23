# Running the on-target tests without a Pico (rp2040js)

The single-core UART test image can run in [rp2040js](https://github.com/wokwi/rp2040js),
an RP2040 emulator, so the `[TEST]` checks in `app_program/subsystemN/subsystemN_test.c`
can be run on a laptop or in CI before anyone flashes a board.

Limits: rp2040js emulates **core 0 only**, with no radio and no USB, so use
`SMP=0 CONSOLE=uart WIFI=none`. Anything that needs real sensors, motors, both
cores, WiFi or timing still has to be tested on the robot.

```sh
# 1. Build the test image (no Pico SDK needed for this variant)
cd build_make
make SMP=0 CONSOLE=uart WIFI=none TEST=1 -j8
arm-none-eabi-objcopy -O binary mtk3pico_smp0_uart_test.elf ../tools/emulator/fw.bin

# 2. One-time setup: the emulator and the RP2040 boot ROM image
cd ../tools/emulator
npm install rp2040js@1.1.1
git clone --depth 1 --branch v1.1.1 https://github.com/wokwi/rp2040js.git rp2040js-src
sed -e 's/^export const bootromB1/const bootromB1/' rp2040js-src/demo/bootrom.ts > bootrom.cjs
echo 'module.exports = { bootromB1 };' >> bootrom.cjs

# 3. Run: prints the console, exits 0 only if the run ended with no FAIL line
node run_tests.cjs fw.bin
```

`fw.bin`, `bootrom.cjs`, `node_modules/` and `rp2040js-src/` are not committed.
