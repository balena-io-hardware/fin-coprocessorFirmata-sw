const firmata = require('firmata');
const Serialport = require('serialport');
const config = require('./config.json');

const board = new firmata.Board(new Serialport(config.port, {baudRate: config.baudrate}), {skipCapabilities: true}, function(err) {
  if (err) {
    console.log(err);
    board.reset();
    return;
  }
});

board.on('ready', () => {
  console.log('Ready');
  console.log('board.firmware: ', board.firmware);

  board.pinMode(3, board.MODES.ANALOG);
  // board.analogRead(3, () => {
  //   console.log("  ✔ received data (exiting)");
  //   console.log("------------------------------");
  //   process.exit();
  // }).catch((error) => {
  //     assert.isNotOk(error,'Promise error');
  //     done();
  //   });
});
