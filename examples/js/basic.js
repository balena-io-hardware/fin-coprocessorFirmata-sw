const firmata = require('firmata');
const Serialport = require('serialport');
const config = require('./config.json');

const board = new firmata.Board(
    new Serialport(
        config.port,
        {baudRate: config.baudrate},
    ),
    {skipCapabilities: false},
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
  console.log(board.pins);
},
);
