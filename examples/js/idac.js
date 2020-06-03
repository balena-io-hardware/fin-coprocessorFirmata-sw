const firmata = require('firmata');
const Serialport = require('serialport');
const config = require('./config.json');

const balenaSysex = 0x0B;
const balenaSysexSubCommand = 0x02;
const setRange = 0x01;
const range = 1;
const pin = 2;

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
  board.sysexCommand([balenaSysex,
    balenaSysexSubCommand,
    setRange,
    pin,
    range,
  ]);
},
);

board.on('string', (message) => {
  if (message.includes('Error')) {
    console.log('\x1b[31m%s\x1b[0m', message);
  } else {
    console.log(message);
  }
});
