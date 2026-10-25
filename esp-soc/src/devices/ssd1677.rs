//! Solomon Systech SSD1677 e-paper controller on 4-wire SPI with a D/C line and an active-high
//! BUSY output: the two 1-bit RAMs (B/W 24h and RED 26h) with their window, counters and data-entry
//! direction, auto-write fill, the update sequence (21h/22h/20h), a register LUT (32h), software
//! reset and deep sleep. The panel's ink is not simulated: an update with the DISPLAY bit puts the
//! RAM's picture on the glass at once and holds BUSY high for a fixed waveform time.
//!
//! What the glass shows after an update follows the waveform source that 22h selects (bit 4 loads
//! the OTP LUT; without it the LUT written to 32h is used):
//! - OTP waveforms (full, fast/differential): the B/W RAM, 1 white, 0 black.
//! - a register LUT whose 00 state is empty (all VSS) is an overlay that changes only the pixels
//!   it addresses: from the (B/W, RED) bits, 01 light gray, 10 mid gray, 11 dark gray, 00 unchanged.
//!   This is FreeInk's anti-aliasing LUT (freeink-sdk docs/grayscale-capabilities.md, "Overlay").
//! - any other register LUT drives every pixel to one of four tones: 00 white, 10 light gray,
//!   01 dark gray, 11 black (FreeInk's factory/absolute LUT after the driver complements the planes).
use crate::board::{BoardEdge, VirtualCycle};

pub const CMD_DRIVER_OUTPUT: u8 = 0x01;
pub const CMD_DEEP_SLEEP: u8 = 0x10;
pub const CMD_DATA_ENTRY: u8 = 0x11;
pub const CMD_SW_RESET: u8 = 0x12;
pub const CMD_MASTER_ACTIVATION: u8 = 0x20;
pub const CMD_UPDATE_CTRL1: u8 = 0x21;
pub const CMD_UPDATE_CTRL2: u8 = 0x22;
pub const CMD_WRITE_BW: u8 = 0x24;
pub const CMD_WRITE_RED: u8 = 0x26;
pub const CMD_WRITE_LUT: u8 = 0x32;
pub const CMD_RAM_X_RANGE: u8 = 0x44;
pub const CMD_RAM_Y_RANGE: u8 = 0x45;
pub const CMD_AUTO_WRITE_BW: u8 = 0x46;
pub const CMD_AUTO_WRITE_RED: u8 = 0x47;
pub const CMD_RAM_X_COUNTER: u8 = 0x4e;
pub const CMD_RAM_Y_COUNTER: u8 = 0x4f;

/// 22h bits: enable clock, enable analog, load temperature, load LUT from OTP, display mode 2,
/// DISPLAY, disable analog, disable clock.
const CTRL2_LOAD_LUT: u8 = 1 << 4;
const CTRL2_MODE2: u8 = 1 << 3;
const CTRL2_DISPLAY: u8 = 1 << 2;

/// Gray levels of the four tones the glass shows.
pub const BLACK: u8 = 0x00;
pub const DARK_GRAY: u8 = 0x55;
pub const MID_GRAY: u8 = 0x80;
pub const LIGHT_GRAY: u8 = 0xaa;
pub const WHITE: u8 = 0xff;

/// How long BUSY stays high, in milliseconds: the waveforms' rough length on an X4-class panel.
#[derive(Clone, Copy, Debug)]
pub struct Ssd1677Timing { pub full_ms: u64, pub fast_ms: u64, pub lut_ms: u64, pub power_ms: u64, pub reset_ms: u64, pub fill_ms: u64 }
impl Default for Ssd1677Timing {
    fn default() -> Self { Ssd1677Timing { full_ms: 1500, fast_ms: 350, lut_ms: 450, power_ms: 20, reset_ms: 2, fill_ms: 5 } }
}

pub struct Ssd1677 {
    /// gate lines (rows) and source lines (columns) of the glass
    pub width: usize, pub height: usize,
    /// row-major, one bit per pixel, MSB leftmost
    pub bw: Vec<u8>, pub red: Vec<u8>,
    /// what the glass shows, one gray level per pixel, in RAM coordinates
    pub glass: Vec<u8>,
    /// the D/C line: low = command, high = parameter or RAM data
    pub dc: bool,
    cmd: u8, args: Vec<u8>,
    entry: u8, x_start: u16, x_end: u16, y_start: u16, y_end: u16, x: u16, y: u16,
    ctrl1: u8, ctrl2: u8,
    /// driver output control's TB bit: the gate scan direction
    pub scan_tb: bool,
    lut: Vec<u8>, lut_loaded: bool,
    pub sleeping: bool,
    timing: Ssd1677Timing, cycles_per_ms: u64,
    busy_until: Option<VirtualCycle>, busy: bool, edges: Vec<BoardEdge>, busy_pin: u8,
    /// updates that changed the glass, and a counter that moves with every glass change
    pub refreshes: u64, pub version: u64,
    pub commands: u64,
}

impl Ssd1677 {
    /// A controller driving `width` × `height`, BUSY on `busy_pin`, time counted in bus cycles.
    pub fn new(width: usize, height: usize, busy_pin: u8, cycles_per_ms: u64) -> Self {
        let n = width.div_ceil(8) * height;
        let mut p = Ssd1677 { width, height, bw: vec![0xff; n], red: vec![0xff; n], glass: vec![WHITE; width * height], dc: false, cmd: 0, args: Vec::new(),
            entry: 3, x_start: 0, x_end: 0, y_start: 0, y_end: 0, x: 0, y: 0, ctrl1: 0, ctrl2: 0, scan_tb: false, lut: Vec::new(), lut_loaded: false,
            sleeping: false, timing: Ssd1677Timing::default(), cycles_per_ms, busy_until: None, busy: false, edges: Vec::new(), busy_pin, refreshes: 0, version: 0, commands: 0 };
        p.registers_reset();
        p
    }
    pub fn with_timing(mut self, timing: Ssd1677Timing) -> Self { self.timing = timing; self }

    fn registers_reset(&mut self) {
        (self.entry, self.x_start, self.x_end, self.y_start, self.y_end, self.x, self.y) = (3, 0, self.width as u16 - 1, 0, self.height as u16 - 1, 0, 0);
        (self.ctrl1, self.ctrl2, self.lut_loaded, self.cmd) = (0, 0, false, 0);
        self.args.clear();
    }

    /// RST low: registers to their defaults and out of deep sleep; the RAMs and the glass stay.
    pub fn hw_reset(&mut self, now: VirtualCycle) { self.registers_reset(); self.sleeping = false; self.end_busy_now(now); }

    /// The BUSY level now, for a board reconnecting its inputs.
    pub fn busy_input(&self) -> (u8, bool) { (self.busy_pin, self.busy) }

    fn start_busy(&mut self, now: VirtualCycle, ms: u64) {
        let until = now + ms * self.cycles_per_ms;
        if !self.busy { self.busy = true; self.edges.push(BoardEdge { cycle: now, pin: self.busy_pin, level: true }); }
        self.busy_until = Some(self.busy_until.map_or(until, |u| u.max(until)));
    }
    fn end_busy_now(&mut self, now: VirtualCycle) {
        self.busy_until = None;
        if self.busy { self.busy = false; self.edges.push(BoardEdge { cycle: now, pin: self.busy_pin, level: false }); }
    }

    /// When BUSY falls next.
    pub fn next_deadline(&self) -> Option<VirtualCycle> { self.busy_until }
    pub fn advance_to(&mut self, cycle: VirtualCycle) {
        if let Some(t) = self.busy_until.filter(|&t| t <= cycle) {
            self.busy_until = None; self.busy = false;
            self.edges.push(BoardEdge { cycle: t, pin: self.busy_pin, level: false });
        }
    }
    pub fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }

    /// Bytes clocked in on MOSI at `now`, commands or data depending on `dc`.
    pub fn bytes(&mut self, now: VirtualCycle, data: &[u8]) {
        for &b in data { self.byte(now, b); }
    }

    pub fn byte(&mut self, now: VirtualCycle, b: u8) {
        if !self.dc { self.command(now, b); return; }
        if self.sleeping { return; }
        match self.cmd {
            CMD_WRITE_BW | CMD_WRITE_RED => self.ram_write(b),
            CMD_WRITE_LUT => { if self.args.is_empty() { self.lut.clear(); } self.args.push(b); self.lut.push(b); self.lut_loaded = true; }
            _ => { self.args.push(b); self.parameter(now); }
        }
    }

    fn command(&mut self, now: VirtualCycle, c: u8) {
        self.commands += 1;
        if self.sleeping { return; }                            // only a hardware reset wakes it
        self.cmd = c;
        self.args.clear();
        match c {
            CMD_SW_RESET => { self.registers_reset(); self.start_busy(now, self.timing.reset_ms); }
            CMD_MASTER_ACTIVATION => self.activate(now),
            _ => {}
        }
    }

    /// A parameter byte of the command in progress.
    fn parameter(&mut self, now: VirtualCycle) {
        let a = &self.args;
        let word = |i: usize| a[i] as u16 | (a[i + 1] as u16) << 8;
        match (self.cmd, a.len()) {
            (CMD_DRIVER_OUTPUT, 3) => self.scan_tb = a[2] & 1 != 0,
            (CMD_DATA_ENTRY, 1) => self.entry = a[0] & 7,
            (CMD_RAM_X_RANGE, 4) => { self.x_start = word(0).min(self.width as u16 - 1); self.x_end = word(2).min(self.width as u16 - 1); }
            (CMD_RAM_Y_RANGE, 4) => { self.y_start = word(0).min(self.height as u16 - 1); self.y_end = word(2).min(self.height as u16 - 1); }
            (CMD_RAM_X_COUNTER, 2) => self.x = word(0).min(self.width as u16 - 1),
            (CMD_RAM_Y_COUNTER, 2) => self.y = word(0).min(self.height as u16 - 1),
            (CMD_UPDATE_CTRL1, 1) => self.ctrl1 = a[0],
            (CMD_UPDATE_CTRL2, 1) => self.ctrl2 = a[0],
            (CMD_DEEP_SLEEP, 1) => if a[0] & 3 != 0 { self.sleeping = true; },
            // A7 is the value the pattern starts with; FreeInk fills with a whole-RAM step (F7h).
            (CMD_AUTO_WRITE_BW, 1) => { let v = if a[0] & 0x80 != 0 { 0xff } else { 0 }; self.bw.fill(v); self.start_busy(now, self.timing.fill_ms); }
            (CMD_AUTO_WRITE_RED, 1) => { let v = if a[0] & 0x80 != 0 { 0xff } else { 0 }; self.red.fill(v); self.start_busy(now, self.timing.fill_ms); }
            _ => {}
        }
    }

    /// One RAM byte (8 pixels) at the counters, which then step in the data-entry direction:
    /// AM (bit 2) clear moves along X first; ID bit 0 increments X, bit 1 increments Y. X counts pixels.
    fn ram_write(&mut self, b: u8) {
        let stride = self.width.div_ceil(8);
        let (x, y) = (self.x as usize, self.y as usize);
        if x < self.width && y < self.height {
            let ram = if self.cmd == CMD_WRITE_BW { &mut self.bw } else { &mut self.red };
            ram[y * stride + x / 8] = b;
        }
        let (xinc, yinc, y_first) = (self.entry & 1 != 0, self.entry & 2 != 0, self.entry & 4 != 0);
        let step_x = |p: &mut Self| -> bool {
            let next = if xinc { p.x as i32 + 8 } else { p.x as i32 - 8 };
            let (lo, hi) = (p.x_start.min(p.x_end) as i32, p.x_start.max(p.x_end) as i32);
            if next < lo || next > hi { p.x = p.x_start; true } else { p.x = next as u16; false }
        };
        let step_y = |p: &mut Self| -> bool {
            let next = if yinc { p.y as i32 + 1 } else { p.y as i32 - 1 };
            let (lo, hi) = (p.y_start.min(p.y_end) as i32, p.y_start.max(p.y_end) as i32);
            if next < lo || next > hi { p.y = p.y_start; true } else { p.y = next as u16; false }
        };
        if y_first { if step_y(self) { step_x(self); } } else if step_x(self) { step_y(self); }
    }

    fn bit(ram: &[u8], stride: usize, x: usize, y: usize) -> bool { ram[y * stride + x / 8] & (0x80 >> (x % 8)) != 0 }

    /// 20h: run the sequence 22h selected. With DISPLAY set the RAM goes onto the glass.
    fn activate(&mut self, now: VirtualCycle) {
        let c = self.ctrl2;
        if c & CTRL2_DISPLAY == 0 { self.start_busy(now, self.timing.power_ms); return; }
        let stride = self.width.div_ceil(8);
        let otp = c & CTRL2_LOAD_LUT != 0 || !self.lut_loaded;
        let overlay = !otp && self.lut.iter().take(10).all(|&v| v == 0);
        for y in 0..self.height { for x in 0..self.width {
            let (b, r) = (Self::bit(&self.bw, stride, x, y), Self::bit(&self.red, stride, x, y));
            let g = &mut self.glass[y * self.width + x];
            *g = if otp { if b { WHITE } else { BLACK } }
                 else if overlay { match (b, r) { (false, false) => *g, (false, true) => LIGHT_GRAY, (true, false) => MID_GRAY, (true, true) => DARK_GRAY } }
                 else { match (b, r) { (false, false) => WHITE, (true, false) => LIGHT_GRAY, (false, true) => DARK_GRAY, (true, true) => BLACK } };
        } }
        if c & CTRL2_LOAD_LUT != 0 { self.lut_loaded = false; }
        self.refreshes += 1;
        self.version += 1;
        let ms = if !otp { self.timing.lut_ms } else if c & CTRL2_MODE2 != 0 { self.timing.fast_ms } else { self.timing.full_ms };
        self.start_busy(now, ms);
    }

    /// The glass as the viewer sees it, `width` × `height` RGB565, top row first. With TB clear
    /// the gates run bottom to top, so RAM row 0 is the bottom row.
    pub fn frame_rgb565(&self) -> Vec<u16> {
        let mut out = vec![0u16; self.width * self.height];
        for row in 0..self.height {
            let src = if self.scan_tb { row } else { self.height - 1 - row };
            for x in 0..self.width {
                let g = self.glass[src * self.width + x] as u16;
                out[row * self.width + x] = (g >> 3) << 11 | (g >> 2) << 5 | (g >> 3);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn send(p: &mut Ssd1677, cmd: u8, data: &[u8]) { p.dc = false; p.byte(0, cmd); p.dc = true; p.bytes(0, data); }

    /// FreeInk's full-frame window: X 0..799 increasing, Y 479..0 decreasing (Ssd1677Driver::setRamArea).
    fn window(p: &mut Ssd1677) {
        send(p, CMD_DATA_ENTRY, &[0x01]);
        send(p, CMD_RAM_X_RANGE, &[0, 0, 0x1f, 0x03]);
        send(p, CMD_RAM_Y_RANGE, &[0xdf, 0x01, 0, 0]);
        send(p, CMD_RAM_X_COUNTER, &[0, 0]);
        send(p, CMD_RAM_Y_COUNTER, &[0xdf, 0x01]);
    }

    #[test]
    fn full_update_shows_the_bw_ram_top_row_first() {
        let mut p = Ssd1677::new(800, 480, 6, 1000);
        window(&mut p);
        let mut frame = vec![0xffu8; 100 * 480];
        frame[0] = 0x7f;                                       // logical (0,0) black
        frame[100 * 479 + 99] = 0xfe;                          // logical (799,479) black
        send(&mut p, CMD_WRITE_BW, &frame);
        send(&mut p, CMD_UPDATE_CTRL2, &[0xf7]);
        p.dc = false; p.byte(100, CMD_MASTER_ACTIVATION);
        let f = p.frame_rgb565();
        assert_eq!((f[0], f[1], f[800 * 480 - 1], f[800 * 480 - 2]), (0, 0xffff, 0, 0xffff));
        assert_eq!(p.refreshes, 1);
        assert_eq!(p.take_edges(), [BoardEdge { cycle: 100, pin: 6, level: true }]);
        assert_eq!(p.next_deadline(), Some(100 + 1500 * 1000));
        p.advance_to(100 + 1500 * 1000);
        assert_eq!(p.take_edges(), [BoardEdge { cycle: 100 + 1500 * 1000, pin: 6, level: false }]);
    }

    #[test]
    fn power_only_sequence_leaves_the_glass() {
        let mut p = Ssd1677::new(16, 2, 6, 1);
        send(&mut p, CMD_AUTO_WRITE_BW, &[0x00]);
        send(&mut p, CMD_UPDATE_CTRL2, &[0xc0]);
        p.dc = false; p.byte(0, CMD_MASTER_ACTIVATION);
        assert!(p.glass.iter().all(|&g| g == WHITE));
        assert_eq!(p.refreshes, 0);
    }

    #[test]
    fn overlay_lut_keeps_unaddressed_pixels_and_grays_the_rest() {
        let mut p = Ssd1677::new(8, 1, 6, 1);
        send(&mut p, CMD_WRITE_BW, &[0b1000_0000]);
        send(&mut p, CMD_UPDATE_CTRL2, &[0xf7]);
        p.dc = false; p.byte(0, CMD_MASTER_ACTIVATION);           // pixel 0 white, 1..7 black
        send(&mut p, CMD_RAM_X_COUNTER, &[0, 0]);
        send(&mut p, CMD_WRITE_BW, &[0b0100_0000]);
        send(&mut p, CMD_RAM_X_COUNTER, &[0, 0]);
        send(&mut p, CMD_WRITE_RED, &[0b0110_0000]);
        let mut lut = vec![0u8; 112]; lut[10] = 0x54; lut[30] = 0xa2;
        send(&mut p, CMD_WRITE_LUT, &lut);
        send(&mut p, CMD_UPDATE_CTRL2, &[0xcc]);
        p.dc = false; p.byte(0, CMD_MASTER_ACTIVATION);
        assert_eq!(&p.glass[..4], &[WHITE, DARK_GRAY, LIGHT_GRAY, BLACK]);
    }

    #[test]
    fn deep_sleep_ignores_commands_until_hardware_reset() {
        let mut p = Ssd1677::new(8, 1, 6, 1);
        send(&mut p, CMD_DEEP_SLEEP, &[0x03]);
        send(&mut p, CMD_WRITE_BW, &[0x00]);
        assert_eq!(p.bw[0], 0xff);
        p.hw_reset(0);
        send(&mut p, CMD_WRITE_BW, &[0x00]);
        assert_eq!(p.bw[0], 0x00);
    }
}
