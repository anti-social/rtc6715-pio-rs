#![no_std]

use embassy_rp::gpio::{Level, Output, Pin};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{Common, Config, Direction, Instance, Irq, PioPin, ShiftDirection, StateMachine};
use embassy_rp::Peripheral;
use embassy_time::Timer;
use fixed::FixedU32;
use fixed::types::extra::U8;

const SYNTHESIZER_REG_A: u8 = 0x0;
const SYNTHESIZER_REG_B: u8 = 0x1;
const READ_BIT: u8 = 0;
const WRITE_BIT: u8 = 1;
const ADDRESS_LEN: u8 = 4;
const _CMD_LEN: u8 = ADDRESS_LEN + 1;
const _DATA_LEN: u8 = 20;
const _PACKET_LEN: u8 = _CMD_LEN + _DATA_LEN;

const TICKS_IN_US: u32 = 125;
/// The spi clock divider sets 25 us clock period
const SPI_CLOCK_DIVIDER: FixedU32<U8> = FixedU32::from_bits((TICKS_IN_US * 25) << 8);
/// The delay clock divider sets 100 us clock period
const DELAY_CLOCK_DIVIDER: FixedU32<U8> = FixedU32::from_bits((TICKS_IN_US * 100) << 8);

#[allow(dead_code)]
pub enum VrxMode {
    Mix = 0,
    Diversity = 1,
}

fn freq_to_reg_value(freq: u16) -> u32 {
    let x = (freq as u32 - 479) / 2;
    ((x / 32) << 7) | (x % 32)
}

fn reg_value_to_freq(v: u32) -> u16 {
    let n = ((v >> 7) & 0b0111_1111) as u16;
    let a = (v & 0b0111_1111) as u16;
    479 + (n * 32 + a) * 2
}

pub struct Rtc6715Pins<CS, DIO, CLK>
where
    CS: Peripheral<P: Pin>,
    DIO: Peripheral<P: PioPin>,
    CLK: Peripheral<P: PioPin>,
{
    pub cs: CS,
    pub dio: DIO,
    pub clk: CLK,
}

/// Rtc6715 chip driven by PIO (uses non-standard SPI).
pub struct Rtc6715<'d, PIO: Instance, const SM: usize, const MODE_SM: usize> {
    cs: Output<'d>,
    spi_sm: StateMachine<'d, PIO, SM>,
    mode_sm: StateMachine<'d, PIO, MODE_SM>,
    irq: Irq<'d, PIO, 0>,
}

impl<'d, PIO, const SM: usize, const MODE_SM: usize> Rtc6715<'d, PIO, SM, MODE_SM>
where
    PIO: Instance,
{
    /// Create a new instance of PioSpi.
    pub fn new<CS, DIO, CLK>(
        common: &mut Common<'d, PIO>,
        mut spi_sm: StateMachine<'d, PIO, SM>,
        mut mode_sm: StateMachine<'d, PIO, MODE_SM>,
        irq: Irq<'d, PIO, 0>,
        pins: Rtc6715Pins<CS, DIO, CLK>,
    ) -> Self
    where
        CS: Peripheral<P: Pin> + 'd,
        DIO: Peripheral<P: PioPin> + 'd,
        CLK: Peripheral<P: PioPin> + 'd,
    {
        let program = pio_asm!(
            ".side_set 1"

            ".wrap_target"

            // write out 4-bit address
            "set x, 3       side 0"
            "addr:"
            "out pins, 1    side 0"
            "jmp x-- addr   side 1"

            // store r/w bit into x register
            "out x, 1       side 0"
            "jmp !x read    side 0"

            // write out r/w bit
            "write:"
            "set pins, 1    side 0"
            "set x, 19      side 1"
            // write out 20 data bits
            "odata:"
            "out pins, 1    side 0"
            "jmp x-- odata  side 1"
            // discard unused 7 bits
            "out null, 7    side 0"
            "jmp end        side 0"

            // write out r/w bit
            "read:"
            "set pins, 0    side 0"
            "set x, 19      side 1"
            // switch pin direction to input
            "set pindirs, 0 side 1"
            // read in 20 data bits
            "idata:"
            "in pins, 1     side 1"
            "jmp x-- idata  side 0"
            // restore pin direction to output
            "set pindirs, 1 side 0"
            // discard unused 27 bits
            "out null, 27   side 0"

            "end:"
            "push           side 0"

            ".wrap"
        );
        let loaded_program = common.load_program(&program.program);

        let pin_io = common.make_pio_pin(pins.dio);
        let pin_clk = common.make_pio_pin(pins.clk);

        let mut cfg = Config::default();
        cfg.use_program(&loaded_program, &[&pin_clk]);
        cfg.set_out_pins(&[&pin_io]);
        cfg.set_in_pins(&[&pin_io]);
        cfg.set_set_pins(&[&pin_io]);
        cfg.shift_out.direction = ShiftDirection::Right;
        cfg.shift_out.auto_fill = true;
        cfg.shift_in.direction = ShiftDirection::Right;
        cfg.clock_divider = SPI_CLOCK_DIVIDER;

        spi_sm.set_config(&cfg);

        spi_sm.set_pin_dirs(Direction::Out, &[&pin_clk, &pin_io]);
        spi_sm.set_pins(Level::Low, &[&pin_clk, &pin_io]);
        spi_sm.set_enable(false);

        let mode_program = pio_asm!(
            ".side_set 1"
            ".wrap_target"

            // Load delay counter into X
            "pull           side 0"
            "mov x, osr     side 0"

            // Delay loop
            "delay:"
            "jmp x-- delay  side 1"

            "irq 0          side 0"

            ".wrap"
        );
        let loaded_mode_program = common.load_program(&mode_program.program);

        let mut cfg = Config::default();
        cfg.use_program(&loaded_mode_program, &[&pin_clk]);
        cfg.clock_divider = DELAY_CLOCK_DIVIDER;

        mode_sm.set_config(&cfg);
        mode_sm.set_pin_dirs(Direction::Out, &[&pin_clk]);
        mode_sm.set_pins(Level::Low, &[&pin_clk]);
        mode_sm.set_enable(false);

        Self {
            cs: Output::new(pins.cs, Level::High),
            spi_sm,
            mode_sm,
            irq,
        }
    }

    async fn read_write(&mut self, data: u32) -> u32 {
        self.spi_sm.set_enable(true);

        self.spi_sm.tx().wait_push(data).await;

        self.spi_sm.rx().wait_pull().await;
        let res = self.spi_sm.rx().pull();

        self.spi_sm.set_enable(true);

        res
    }

    /// Write a register value
    /// Rtc6715 registers contain a 20-bit value
    pub async fn write_reg(&mut self, reg_addr: u8, value: u32) {
        let data = reg_addr as u32 |
            ((WRITE_BIT as u32) << ADDRESS_LEN) |
            (value << (ADDRESS_LEN + 1));

        let _ = self.read_write(data).await;
    }

    /// Read a register value
    /// Rtc6715 registers contain a 20-bit value
    pub async fn read_reg(&mut self, reg_addr: u8) -> u32 {
        let data = (reg_addr | (READ_BIT << ADDRESS_LEN)) as u32;

        self.read_write(data).await
    }

    pub async fn set_mode(&mut self, mode: VrxMode, freq: u16) {
        if let VrxMode::Mix = mode {
            self.mode_sm.set_enable(true);
            // 100 us * 1000 = 100 ms
            self.mode_sm.tx().push(1000);

            self.irq.wait().await;
            self.mode_sm.set_enable(false);

            Timer::after_millis(500).await;
        }

        self.write_reg(SYNTHESIZER_REG_A, 0x8).await;
        Timer::after_micros(500).await;
        self.set_freq(freq).await;
    }

    pub async fn get_freq(&mut self) -> u16 {
        self.cs.set_low();
        let reg_value = self.read_reg(SYNTHESIZER_REG_B).await;
        self.cs.set_high();
        reg_value_to_freq(reg_value)
    }

    pub async fn set_freq(&mut self, freq: u16) {
        let reg_value = freq_to_reg_value(freq);
        self.cs.set_low();
        self.write_reg(SYNTHESIZER_REG_A, 0x8).await;
        self.write_reg(SYNTHESIZER_REG_B, reg_value).await;
        self.cs.set_high();
    }
}
