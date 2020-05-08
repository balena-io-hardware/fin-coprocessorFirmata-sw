const firmata = require('firmata');
const Serialport = require('serialport');
const port = '/dev/ttyACM0';
const baud = 115200;

const balenaSysex = 0x0B;
const balenaSysexSubCommand = 0x00; // subcommand for requesting firmware version

const board = new firmata.Board(new Serialport(port, {baudRate: baud}), {skipCapabilities: true}, function(err) {
  if (err) {
    console.log(err);
    board.reset();
    return;
  }
});

board.on('ready', () => {
  console.log('Ready');
  console.log('board.firmware: ', board.firmware);

  board.sysexCommand([balenaSysex, balenaSysexSubCommand]);
  board.sysexResponse(balenaSysex, (data) => {
    const balenaVersion = firmata.decode(data);
    console.log('Balena Firmata Version: ', String.fromCharCode.apply(null, balenaVersion));
  });
});
