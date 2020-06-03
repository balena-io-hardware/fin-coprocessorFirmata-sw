const firmata = require('firmata');
const port = '/dev/ttyS0';

const balenaSysex = 0x0B;
const balenaSysexSleep = 0x01; // subcommand for requesting firmware version
const balenaSysexSleepInitDelay = 3; // time before triggering sleep (seconds)
const balenaSysexSleepPeriod = 262143; // period of time to sleep (seconds)

const board = new firmata.Board(port, {skipCapabilities: true}, function(err) {
  if (err) {
    console.log(err);
    board.reset();
    return;
  }
});


board.on('ready', () => {
  console.log('Ready');
  console.log('board.firmware: ', board.firmware);

  board.sysexCommand([
    balenaSysex,
    balenaSysexSleep,
    (balenaSysexSleepInitDelay & 0x7F),
    (balenaSysexSleepPeriod & 0x7F),
    ((balenaSysexSleepPeriod >> 7) & 0x7F),
    ((balenaSysexSleepPeriod >> 14) & 0x7F),
    ((balenaSysexSleepPeriod >> 21) & 0x7F),
  ]);
  board.sysexResponse(balenaSysex, (data) => {
    const balenaVersion = firmata.decode(data);
    console.log('Balena Firmata Version: ',
        String.fromCharCode.apply(null, balenaVersion),
    );
  });
});

console.log(balenaSysexSleepPeriod & 0x7F);
console.log((balenaSysexSleepPeriod >> 7) & 0x7F);
console.log((balenaSysexSleepPeriod >> 14) & 0x7F);
console.log((balenaSysexSleepPeriod >> 21) & 0x7F);

console.log(((((balenaSysexSleepPeriod >> 21) & 0x7F) << 21) & 0x01) |
(((balenaSysexSleepPeriod >> 14) & 0x7F) << 14) |
(((balenaSysexSleepPeriod >> 7) & 0x7F) << 7) |
((balenaSysexSleepPeriod & 0x7F)));
