const firmata = require('firmata');
const Serialport = require('serialport');
const config = require('./config.json');

const adxl345 = 0x53;
const sensitivity = 0.00390625;
const register = {
  POWER: 0x2D,
  RANGE: 0x31,
  READ: 0xB2,
};

const board = new firmata.Board(new Serialport(config.port, {baudRate: config.baudrate}), {skipCapabilities: true}, function(err) {
  if (err) {
    console.log(err);
    board.reset();
    return;
  }
});

board.on('ready', () => {
  console.log('connected...');
  console.log('board.firmware: ', board.firmware);

  board.i2cConfig(0);

  // Toggle power to reset
  board.i2cWrite(adxl345, register.POWER, 0);
  board.i2cWrite(adxl345, register.POWER, 8);
  board.i2cWrite(adxl345, register.RANGE, 8);

  board.i2cRead(adxl345, register.READ, 6, (data) => {
    const x = (data[1] << 8) | data[0];
    const y = (data[3] << 8) | data[2];
    const z = (data[5] << 8) | data[4];
    console.log('---');

    // Wrap and clamp 16 bits;
    const X = (x >> 15 ? ((x ^ 0xFFFF) + 1) * -1 : x) * sensitivity;
    const Y = (y >> 15 ? ((y ^ 0xFFFF) + 1) * -1 : y) * sensitivity;
    const Z = (z >> 15 ? ((z ^ 0xFFFF) + 1) * -1 : z) * sensitivity;

    console.log('X: ', X);
    console.log('Y: ', Y);
    console.log('Z: ', Z);
  });
});
