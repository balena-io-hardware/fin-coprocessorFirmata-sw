const firmata = require('firmata');
const Serialport = require('serialport');
const config = require('./config.json');

const blinkInterval = 500;
const pin = 14; // Devkit LED

const board = new firmata.Board(
    new Serialport(
        config.port,
        {baudRate: config.baudrate},
    ),
    {skipCapabilities: true},
    function(err) {
      if (err) {
        console.log(err);
        board.reset();
        return;
      }
    },
);

board.on('ready', () => {
  console.log('Ready');
  console.log('board.firmware: ', board.firmware);

  let state = 1;

  board.pinMode(pin, board.MODES.OUTPUT);

  setInterval(() => {
    board.digitalWrite(pin, (state ^= 1));
  }, blinkInterval);
});

