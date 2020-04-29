let firmata = require('firmata')
const port = "/dev/ttyUSB1";

const balena_sysex = 0x0B;
const balena_sysex_sub_command = 0x00; // subcommand for requesting firmware version

let board = new firmata.Board(port, {skipCapabilities: true},function(err) {
    if (err) {
        console.log(err);
        board.reset();
    return;
    }
});

board.on("ready", () => {
    console.log("Ready");
    console.log("board.firmware: ", board.firmware);

    board.sysexCommand([balena_sysex, balena_sysex_sub_command]);
    board.sysexResponse(balena_sysex, data => {
        const balena_version = firmata.decode(data);
        console.log("Balena Firmata Version: ", String.fromCharCode.apply(null, balena_version));
    })
});