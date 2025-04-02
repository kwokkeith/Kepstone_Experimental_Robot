use std::thread;
use std::time::Duration;
use rppal::gpio::Gpio;
use rppal::i2c::I2c;
use rppal::pwm::{Channel, Pwm};
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};
use rppal::uart::{Parity, Uart};

pub fn motor_control(pwm: &Pwm, duty_cycle: f64) {
    //println!("bts7960_driver_control");

    //let gpio = Gpio::new()?;
    //let i2c = I2c::new()?;
    //let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 16_000_000, Mode::Mode0)?;
    //let uart = Uart::new(115_200, Parity::None, 8, 1)?;
    //
    //

    //let gpio = Gpio::new()?;
    //let mut pin = gpio.get(12)?.into_output();

    pwm.enable();

    const PWM_FREQ: f64 = 100.0;

    pwm.set_frequency(PWM_FREQ, duty_cycle);
    //thread::sleep(Duration::from_secs(1));

    //for duty in 0..=1000 {
    //    pwm0.set_frequency(PWM_FREQ, (duty as f64)/1000.0);
    //    println!("{:?}",duty);
    //}

    //for duty in (0..=1000).rev() {
    //    pwm0.set_frequency(PWM_FREQ, (duty as f64)/1000.0);
    //    thread::sleep(Duration::from_millis(10));
    //    println!("{:?}",duty);
    //}

    //println!("{:?}", pwm0.is_enabled());
    //while true {
    //    pin.set_high();
    //    pin.set_low();
    //    thread::sleep(Duration::from_secs(1));
    //}

}
