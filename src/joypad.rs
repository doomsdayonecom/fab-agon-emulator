use agon_ez80_emulator::gpio::GpioSet;
use sdl3::joystick::HatState;

const PIN_PORT1_DPAD_UP: u8 = 1;
const PIN_PORT1_DPAD_RIGHT: u8 = 7;
const PIN_PORT1_DPAD_DOWN: u8 = 3;
const PIN_PORT1_DPAD_LEFT: u8 = 5;

// note - joypad gpios are pulled high, so depressed state is 'false'

// set gpio joypad inputs to open switch position (high)
pub fn clear_state(gpios: &GpioSet) {
    gpios.c.set_input_pins(0xff);
    gpios.d.set_input_pin(4, true);
    gpios.d.set_input_pin(5, true);
    gpios.d.set_input_pin(6, true);
    gpios.d.set_input_pin(7, true);
}

pub fn on_axis_motion(gpios: &GpioSet, joystick_num: u32, axis_idx: u8, value: i16) {
    if axis_idx > 1 {
        return;
    }

    let pin_offset = (!(joystick_num as u8) & 1) + if axis_idx == 0 { 4 } else { 0 };

    gpios.c.set_input_pin(0 + pin_offset, value > -16384);
    gpios.c.set_input_pin(2 + pin_offset, value < 16384);
}

pub fn on_hat_motion(gpios: &GpioSet, joystick_num: u32, _hat_idx: u8, state: HatState) {
    // The pins of joystick port2 can be retrieved by subtracting 1 from the corresponding
    // pin of joystick port1. The pin_offset should toggle between 0 and 1 for increasing
    // joystick_nums, so every second joystick_num input registers for player2.
    let pin_offset = joystick_num as u8 & 1;

    match state {
        HatState::Centered => {
            gpios.c.set_input_pin(PIN_PORT1_DPAD_UP - pin_offset, true);
            gpios
                .c
                .set_input_pin(PIN_PORT1_DPAD_RIGHT - pin_offset, true);
            gpios
                .c
                .set_input_pin(PIN_PORT1_DPAD_DOWN - pin_offset, true);
            gpios
                .c
                .set_input_pin(PIN_PORT1_DPAD_LEFT - pin_offset, true);
        }
        HatState::Up => gpios.c.set_input_pin(PIN_PORT1_DPAD_UP - pin_offset, false),
        HatState::Right => gpios
            .c
            .set_input_pin(PIN_PORT1_DPAD_RIGHT - pin_offset, false),
        HatState::Down => gpios
            .c
            .set_input_pin(PIN_PORT1_DPAD_DOWN - pin_offset, false),
        HatState::Left => gpios
            .c
            .set_input_pin(PIN_PORT1_DPAD_LEFT - pin_offset, false),
        _ => {}
    }
}

pub fn on_button(gpios: &GpioSet, joystick_num: u32, button_idx: u8, is_down: bool) {
    let pin = 4 + (!(joystick_num as u8) & 1) + (button_idx & 1) * 2;
    gpios.d.set_input_pin(pin, !is_down);
}

/// Drive a joystick port from an RRDC virtual pad (SPEC /pad). `buttons` is the
/// contract's canonical mask (bit0 LEFT, 1 RIGHT, 2 UP, 3 DOWN, 4 A, 5 B, …);
/// the Agon DE-9 port is a d-pad + two fire buttons, so LEFT/RIGHT/UP/DOWN map
/// to the port-C d-pad pins and A/B to the two port-D button pins (the higher
/// SNES-style buttons have no Agon target and are ignored). Unplugged reads as
/// idle. Pin selection matches on_hat_motion / on_button so the injected pad
/// lands on the exact pins host joystick input does.
pub fn set_virtual_pad(gpios: &GpioSet, port: u32, connected: bool, buttons: u16) {
    const LEFT: u16 = 0x001;
    const RIGHT: u16 = 0x002;
    const UP: u16 = 0x004;
    const DOWN: u16 = 0x008;
    const A: u16 = 0x010;
    const B: u16 = 0x020;
    // pins are pulled high, so a released input is `true`.
    let held = |bit: u16| connected && (buttons & bit) != 0;

    let dpad_off = (port as u8) & 1; // matches on_hat_motion's pin_offset
    gpios.c.set_input_pin(PIN_PORT1_DPAD_UP - dpad_off, !held(UP));
    gpios.c.set_input_pin(PIN_PORT1_DPAD_RIGHT - dpad_off, !held(RIGHT));
    gpios.c.set_input_pin(PIN_PORT1_DPAD_DOWN - dpad_off, !held(DOWN));
    gpios.c.set_input_pin(PIN_PORT1_DPAD_LEFT - dpad_off, !held(LEFT));

    let btn_base = 4 + (!(port as u8) & 1); // matches on_button
    gpios.d.set_input_pin(btn_base, !held(A)); // button 0
    gpios.d.set_input_pin(btn_base + 2, !held(B)); // button 1
}
