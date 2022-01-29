#![no_std]
#![no_main]

use bsp::hal::{
    clocks::{Clock, init_clocks_and_plls},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use cortex_m_rt::entry;
use core::panic::PanicInfo;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use nano_leb128::ULEB128;
use rp_pico as bsp;
use rp_pico::hal;

use crate::hal::gpio::PinId;
use crate::hal::spi::SpiDevice;
use crate::hal::typelevel::NoneT;
use rp2040_st7789::st7789::{OptionalOutputPin, Rotation, ST7789Display};
use rp2040_st7789::font::Font;

mod video;

#[panic_handler]
fn picnic(_info: &PanicInfo) -> ! {
    loop {}
}

struct DecodeInfo {
    read_array_index: usize,
    current_color: bool,
    remain_size: u32,
}

impl DecodeInfo {
    fn new() -> DecodeInfo {
        DecodeInfo {
            read_array_index: 0,
            current_color: true,
            remain_size: 0,
        }
    }

    fn get_next_color(&mut self) -> bool {
        loop {
            if self.remain_size > 0 {
                self.remain_size -= 1;
                return self.current_color;
            }
            if self.read_array_index >= video::VIDEO.len() {
                self.read_array_index = 0;
            }
            let (x, y) = ULEB128::read_from(&video::VIDEO[self.read_array_index..]).unwrap();
            self.read_array_index += y;
            self.remain_size = u64::from(x) as u32;
            self.current_color = !self.current_color;
        }
    }

    fn render<
        K: OptionalOutputPin,
        L: PinId,
        M: OptionalOutputPin,
        N: OptionalOutputPin,
        S: SpiDevice
    >(&mut self, display: &mut ST7789Display<K, L, M, N, S>) {
        let buf = &mut [0u8; 240 * 180 * 2];
        for i in 0..(240 * 180) {
            if self.get_next_color() {
                buf[i * 2] = 0xFF;
                buf[i * 2 + 1] = 0xFF;
            } else {
                buf[i * 2] = 0x00;
                buf[i * 2 + 1] = 0x00;
            }
        }
        display.draw_color_buf_raw(buf, 32, 0, 180, 240);
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // setup clock and mosi pin
    // TODO: Please replace it with your pin
    pins.gpio3.into_mode::<hal::gpio::FunctionSpi>();
    pins.gpio2.into_mode::<hal::gpio::FunctionSpi>();
    // setup spi
    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        (30u32 << 20u32).Hz(),
        &embedded_hal::spi::MODE_3,
    );

    // TODO: Please replace it with your pin
    let mut display = ST7789Display::new(
        pins.gpio0.into_push_pull_output(),
        pins.gpio1.into_push_pull_output(),
        NoneT,
        NoneT,
        spi,
        240,
        240,
        Rotation::Portrait,
        &mut delay,
    );

    let font = rp2040_st7789::fonts::VGA1_16X32;

    let text = "Bad Apple";
    let (w, h) = font.measure_text(text);
    display.draw_text((240 - w) / 2, (240 - h) / 2, text, &font, 0xffff, 0);

    delay.delay_ms(1000);
    display.fill(0);
    display.set_rotation(Rotation::Landscape);
    let mut info = DecodeInfo::new();
    loop {
        info.render(&mut display);
        delay.delay_ms(90);
    }
}
