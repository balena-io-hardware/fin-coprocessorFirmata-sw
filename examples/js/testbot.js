const firmata = require('firmata');
const port = '/dev/ttyS0';

const balenaSysex = 0x0B;
const balenaSysexSubCommand = 0x00;

const mcp4725 = {
  ADDR: 0x60,
  DAC: 0x40,
  DAC_EEPROM: 0x60,
};

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

  board.sysexCommand([balenaSysex, balenaSysexSubCommand]);
  board.sysexResponse(balenaSysex, (data) => {
    const balenaVersion = firmata.decode(data);
    console.log('Balena Firmata Version: ',
        String.fromCharCode.apply(null, balenaVersion),
    );
  });

  const target = 2857; // See testbot calculation for target value
  const dutPwEn = 14;

  board.pinMode(dutPwEn, board.MODES.OUTPUT);
  board.i2cConfig(0);

  board.i2cWrite(mcp4725.ADDR, mcp4725.DAC,
      [target >> 4, (target & 0x0F) << 4],
  );
  console.log('DAC Value Write: ', target);

  board.digitalWrite(dutPwEn, 1); // Enable DUT power

  board.i2cRead(mcp4725.ADDR, mcp4725.DAC, 3, (data) => {
    const output = (data[1] << 4) | (data[2] >> 4);
    console.log('DAC Value Read: ', output);
  });
});

board.on('string', (message) => {
  console.log(message);
});
