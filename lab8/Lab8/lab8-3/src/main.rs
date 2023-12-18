#![no_std]
#![no_main]
use core::fmt::Write;
use esp32c3_hal::{
    clock::ClockControl,
    gpio::IO,
    i2c::I2C,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    Rtc,
    UsbSerialJtag,
};
use esp_backtrace as _;
use icm42670::{prelude::*, Address, Icm42670};
const ACCEL_THRESHOLD: f32 = 1.0;
#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio10,
        io.pins.gpio8,
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    )
    .unwrap();

    let mut icm = Icm42670::new(i2c, Address::Primary).unwrap();

    loop {
        let accel_norm = icm.accel_norm().unwrap();
        let gyro_norm = icm.gyro_norm().unwrap();
        let ax = accel_norm.x; // Replace with your actual value
        let ay = accel_norm.y; // Replace with your actual value


        if accel_norm.x < 0.5 && accel_norm.x > -0.5 && accel_norm.y < 0.5 && accel_norm.y > -0.5{
            writeln!(UsbSerialJtag,"GROUND");
        }
        else{
            if ay > 0.5{
                if ax > 0.5{
                    writeln!(UsbSerialJtag,"UP LEFT");
                }
                else if ax < -0.5{
                    writeln!(UsbSerialJtag,"UP RIGHT");
                }
                else{
                    writeln!(UsbSerialJtag,"UP");
                }
            }
            else if ay < -0.5{
                if ax > 0.5{
                    writeln!(UsbSerialJtag,"DOWN LEFT");
                }
                else if ax < -0.5{
                    writeln!(UsbSerialJtag,"DOWN RIGHT");
                }
                else{
                    writeln!(UsbSerialJtag,"DOWN");
                }
            }
                        
            else if ax > 0.5{
                writeln!(UsbSerialJtag,"LEFT");
            }
            else if ax < -0.5{
                writeln!(UsbSerialJtag,"RIGHT");
            }
        }
        
        /*
        writeln!(
            UsbSerialJtag,
            "ACCEL  =  X: {:+.04} Y: {:+.04} Z: {:+.04}\t\tGYRO  =  X: {:+.04} Y: {:+.04} Z: {:+.04}",
            accel_norm.x, accel_norm.y, accel_norm.z, gyro_norm.x, gyro_norm.y, gyro_norm.z
        )
        .ok();
        */
        //let mut usb_serial_jtag = Vec::new();
        /*
        let mut inclination_str = String::new();
        let accel_x = accel_norm.x; // Replace with your actual value
        let accel_y = accel_norm.y; // Replace with your actual value

        if accel_x > ACCEL_THRESHOLD {
            inclination_str = String::from("LEFT");
        } else if accel_x < -ACCEL_THRESHOLD {
            inclination_str = String::from("RIGHT");
        }   

        if accel_y > ACCEL_THRESHOLD {
            if !inclination_str.is_empty() {
                inclination_str.push(' ');
            }
            inclination_str.push_str("UP");
        } else if accel_y < -ACCEL_THRESHOLD {
            if !inclination_str.is_empty() {
                inclination_str.push(' ');
            }
            inclination_str.push_str("DOWN");
        }

    // Write to the serial port or another writable object
        if !inclination_str.is_empty() {
            writeln!(UsbSerialJtag, "Board inclination: {}", inclination_str).unwrap();
        } else {
            writeln!(UsbSerialJtag, "Board inclination: LEVEL").unwrap();
        }
        */
        delay.delay_ms(1000u32);
    }
}
