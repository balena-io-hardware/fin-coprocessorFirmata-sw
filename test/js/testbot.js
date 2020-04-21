let firmata = require('firmata')
const port = "/dev/ttyS0";

const balena_sysex = 0x0B;
const balena_sysex_sub_command = 0x00; // subcommand for requesting firmware version

const STRING_DATA = 0x71;

const mcp4725 = {
    ADDR: 0x60,
    DAC: 0x40,
    DAC_EEPROM: 0x60,
};

let board = new firmata.Board(port, {skipCapabilities: true},function(err) {
    if (err) {
        console.log(err);
        board.reset();
    return;
    }
});

board.on("ready", () => {
    console.log("Ready");
    console.log('board.firmware: ', board.firmware);

    board.sysexCommand([balena_sysex, balena_sysex_sub_command]);
    board.sysexResponse(balena_sysex, data => {
        const balena_version = firmata.decode(data);
        console.log("Balena Firmata Version: ", String.fromCharCode.apply(null, balena_version));
    })

    const target = 2857; // See testbot calculation for target value
    const dut_pw_en = 14;

    board.pinMode(dut_pw_en, board.MODES.OUTPUT);
    board.i2cConfig(0);

    board.i2cWrite(mcp4725.ADDR, mcp4725.DAC, [target >> 4, (target & 0x0F) << 4]);
    console.log("DAC Value Write: ", target);

    board.digitalWrite(dut_pw_en, 1); // Enable DUT power

    board.i2cRead(mcp4725.ADDR, mcp4725.DAC, 3, data => {
        const output = (data[1] << 4) | (data[2] >> 4);
        console.log("DAC Value Read: ", output);
    });
});

board.sysexResponse(STRING_DATA, data => {
    console.log(String.fromCharCode.apply(null, firmata.decode(data)));
});