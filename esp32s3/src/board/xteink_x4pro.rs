//! Xteink X4 Pro e-reader (ESP32-S3, 16 MB flash, 8 MB octal PSRAM). Wiring from FreeInk's board
//! profile (freeink-sdk libs/hardware/BoardConfig/include/BoardConfig.h, XTEINK_X4_PRO) and its
//! bring-up notes (docs/xteink-x4pro-support.md):
//!   - SSD1677 800×480 e-paper on SPI2: SCLK 12, MOSI 11, CS 13 (a GPIO), DC 18, RST 14, BUSY 6 (high while busy)
//!   - GT911 touch at I2C 0x5D (SDA 39 / SCL 38), mounted portrait (reports X 0..479, Y 0..799); its
//!     capacitive Home pad is the HaveKey bit of status 0x814E. Powered while GPIO1 is high and GPIO2 low.
//!   - BM8563 RTC (PCF8563-compatible) at 0x51 and CW2017 fuel gauge at 0x63 on the same bus
//!   - SD card on the SDMMC host, 1-bit: CLK 41, CMD 42, D0 40 (GPIO5 is its active-low power enable)
//!   - buttons, active low: Left GPIO 0, Right GPIO 7, Power GPIO 3; charger STAT GPIO 21 (high = charging)
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use esp_periph::i2c::Reg8Device;
use esp_soc::board::{BoardEdge, BoardModel, VirtualCycle};
use esp_soc::devices::Ssd1677;

use crate::i2c::{Gt911, I2cDevice, TouchState};

pub const PIN_EPD_SCLK: u8 = 12;
pub const PIN_EPD_MOSI: u8 = 11;
pub const PIN_EPD_CS: u8 = 13;
pub const PIN_EPD_DC: u8 = 18;
pub const PIN_EPD_RST: u8 = 14;
pub const PIN_EPD_BUSY: u8 = 6;
pub const PIN_RAIL: u8 = 1;
pub const PIN_TOUCH_POWER_N: u8 = 2;
pub const PIN_SD_CLK: u8 = 41;
pub const PIN_SD_CMD: u8 = 42;
pub const PIN_SD_D0: u8 = 40;
pub const PIN_LEFT: u8 = 0;
pub const PIN_RIGHT: u8 = 7;
pub const PIN_POWER: u8 = 3;
pub const PIN_CHARGE_STAT: u8 = 21;

/// The glass as the device is held: portrait. The panel's native scan is 800×480 landscape,
/// mounted turned 90° (CrossPoint draws its portrait UI rotated into it).
pub const WIDTH: usize = 480;
pub const HEIGHT: usize = 800;

/// A device behind a switched supply: silent (NACK) while the rail is off.
struct Powered { dev: Box<dyn I2cDevice>, on: Arc<AtomicBool> }
impl I2cDevice for Powered {
    fn start(&mut self, read: bool) -> bool { self.on.load(Ordering::Relaxed) && self.dev.start(read) }
    fn write(&mut self, b: u8) -> bool { self.on.load(Ordering::Relaxed) && self.dev.write(b) }
    fn read(&mut self) -> u8 { if self.on.load(Ordering::Relaxed) { self.dev.read() } else { 0xff } }
}

pub struct XteinkX4Pro {
    pub panel: Ssd1677,
    cs_low: bool,
    rst_low: bool,
    now: VirtualCycle,
    touch: Arc<Mutex<TouchState>>,
    touch_power: Arc<AtomicBool>,
    pub gpio_events: u64,
}

impl Default for XteinkX4Pro { fn default() -> Self { Self::new() } }

impl XteinkX4Pro {
    pub fn new() -> Self {
        XteinkX4Pro { panel: Ssd1677::new(HEIGHT, WIDTH, PIN_EPD_BUSY, crate::periph::CPU_HZ / 1000), cs_low: false, rst_low: false, now: 0,
                      touch: Default::default(), touch_power: Default::default(), gpio_events: 0 }
    }
    fn touch_powered(&self) -> bool { self.touch_power.load(Ordering::Relaxed) }
    /// The landscape glass turned 90° clockwise into the portrait view: the view's (x, y) is the
    /// glass's (y, 479 - x).
    pub fn portrait_frame(&self) -> Vec<u16> {
        let land = self.panel.frame_rgb565();
        let mut out = vec![0u16; WIDTH * HEIGHT];
        for y in 0..HEIGHT { for x in 0..WIDTH { out[y * WIDTH + x] = land[(WIDTH - 1 - x) * HEIGHT + y]; } }
        out
    }
}

impl BoardModel for XteinkX4Pro {
    fn name(&self) -> &'static str { "xteink-x4pro" }
    fn gpio_output_at(&mut self, cycle: VirtualCycle, changes: &[(u8, bool)], enabled: u64, output: u64) {
        self.now = self.now.max(cycle);
        for &(pin, level) in changes {
            self.gpio_events += 1;
            match pin {
                PIN_EPD_DC => self.panel.dc = level,
                PIN_EPD_CS => self.cs_low = !level,
                PIN_EPD_RST => { if self.rst_low && level { self.panel.hw_reset(self.now); } self.rst_low = !level; }
                _ => {}
            }
        }
        let driven = |pin: u8| (enabled >> pin & 1 != 0).then_some(output >> pin & 1 != 0);
        let on = driven(PIN_RAIL) == Some(true) && driven(PIN_TOUCH_POWER_N) == Some(false);
        if self.touch_power.swap(on, Ordering::Relaxed) && !on { *self.touch.lock().expect("X4 Pro touch mutex poisoned") = TouchState::default(); }
    }
    fn gpio_events(&self) -> u64 { self.gpio_events }
    // The panel is write-only (no MISO); a transaction reaches it only while its CS is low.
    fn spi_transfer(&mut self, host: u8, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if host == 2 && self.cs_low && !self.rst_low { self.panel.bytes(self.now, tx); }
        vec![0xff; rx_len]
    }
    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        vec![
            (0, 0x5d, Box::new(Powered { dev: Box::new(Gt911::new(self.touch.clone(), 480, 800)), on: self.touch_power.clone() })),
            // PCF8563 registers: control 1/2, then BCD seconds (bit 7 VL clear: the time is valid),
            // minutes, hours, day, weekday, month, year: 2026-01-01 12:00:00, a Thursday.
            (0, 0x51, Box::new(Reg8Device::new("bm8563", &[(0x02, 0x00), (0x03, 0x00), (0x04, 0x12), (0x05, 0x01), (0x06, 0x04), (0x07, 0x01), (0x08, 0x26)]))),
            // CW2017 (freeink-sdk BatteryMonitor.cpp): VERSION 0Dh is the running engine, VCELL a
            // 14-bit count of 312.5 uV (3.95 V), SOC 87 %, MODE normal. The firmware uploads its
            // battery profile to 10h..5Fh and sets the update flag in 0Bh itself.
            (0, 0x63, Box::new(Reg8Device::new("cw2017", &[(0x00, 0x0d), (0x02, 0x31), (0x03, 0x60), (0x04, 87), (0x08, 0x00)]))),
        ]
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> { Some((WIDTH as u32, HEIGHT as u32, self.portrait_frame(), self.panel.version)) }
    fn display_version(&self) -> u64 { self.panel.version }
    fn display_frames(&self) -> u64 { self.panel.refreshes }
    // An e-paper frame changes a few times a second at most; rebuilding 800×480 often is waste.
    fn display_push_hz(&self) -> u64 { 10 }
    /// The UI's pointer on the portrait view. The GT911 is mounted portrait (X 0..479, Y 0..799)
    /// and reports in the view's own coordinates; the firmware swaps its axes and flips Y onto the
    /// landscape panel (BoardConfig swapXY, flipY).
    fn touch(&mut self, x: u16, y: u16, down: bool) {
        if !self.touch_powered() { return; }
        self.touch.lock().expect("X4 Pro touch mutex poisoned").update(x.min(WIDTH as u16 - 1), y.min(HEIGHT as u16 - 1), down);
    }
    fn key(&mut self, name: &str, down: bool) -> bool {
        if name != "home" { return false; }
        if self.touch_powered() { self.touch.lock().expect("X4 Pro touch mutex poisoned").key = down; }
        true
    }
    fn input_levels(&self) -> Vec<(u8, bool)> {
        vec![self.panel.busy_input(), (PIN_LEFT, true), (PIN_RIGHT, true), (PIN_POWER, true), (PIN_CHARGE_STAT, false)]
    }
    fn next_deadline(&self) -> Option<VirtualCycle> { self.panel.next_deadline() }
    fn advance_to(&mut self, cycle: VirtualCycle) { self.now = self.now.max(cycle); self.panel.advance_to(cycle); }
    fn take_edges(&mut self) -> Vec<BoardEdge> { self.panel.take_edges() }
    fn named_pin(&self, name: &str) -> Option<u8> {
        match name { "left" | "prev" => Some(PIN_LEFT), "right" | "next" => Some(PIN_RIGHT), "power" => Some(PIN_POWER), _ => None }
    }
    fn sd_socket(&self) -> Option<(u8, u8, [u8; 4])> { Some((PIN_SD_CLK, PIN_SD_CMD, [PIN_SD_D0, 0xff, 0xff, 0xff])) }
    fn report(&self) -> String {
        let p = &self.panel;
        format!("[x4pro] epd: {} commands, {} refreshes, sleeping={}; touch powered={}; gpio events {}", p.commands, p.refreshes, p.sleeping, self.touch_powered(), self.gpio_events)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Read GT911 status 0x814E through the board's I2C device: address phase, register pointer, read.
    fn gt911_status(dev: &mut dyn I2cDevice) -> Option<u8> {
        if !(dev.start(false) && dev.write(0x81) && dev.write(0x4e) && dev.start(true)) { return None; }
        Some(dev.read())
    }

    #[test]
    fn touch_controller_answers_only_while_powered_and_reports_the_home_key() {
        let mut b = XteinkX4Pro::new();
        let mut devs = b.i2c_devices();
        let gt = &mut devs[0].2;
        assert_eq!(gt911_status(gt.as_mut()), None);
        assert!(b.key("home", true));                           // unpowered: the key is lost, as on the board
        let on = 1 << PIN_RAIL | 1 << PIN_TOUCH_POWER_N;
        b.gpio_output_at(0, &[(PIN_RAIL, true), (PIN_TOUCH_POWER_N, false)], on, 1 << PIN_RAIL);
        assert_eq!(gt911_status(gt.as_mut()), Some(0x80));
        assert!(b.key("home", true));
        b.touch(10, 20, true);
        assert_eq!(gt911_status(gt.as_mut()), Some(0x91));
        assert!(!b.key("menu", true));
        b.gpio_output_at(0, &[(PIN_TOUCH_POWER_N, true)], on, on);
        assert_eq!(gt911_status(gt.as_mut()), None);
    }

    #[test]
    fn portrait_view_is_the_landscape_glass_turned_clockwise() {
        let mut b = XteinkX4Pro::new();
        b.panel.glass[0] = 0;                                   // RAM (0, 0): with TB clear, the glass's bottom-left
        let f = b.portrait_frame();
        assert_eq!(f.len(), WIDTH * HEIGHT);
        assert_eq!(f[0], 0);                                    // the view's top-left
        assert_eq!(f.iter().filter(|&&p| p == 0).count(), 1);
    }
}
