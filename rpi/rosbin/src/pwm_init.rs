use rppal::pwm::{Channel, Pwm};
use rppal::gpio::Gpio;

pub fn main() {
    println!("Initalizing PWM Channel 0...");
    let pwm0 = Pwm::new(Channel::Pwm0).unwrap();
    pwm0.enable();
    pwm0.set_frequency(50.0, 0.0);

    println!("Initalizing PWM Channel 1...");
    let pwm1 = Pwm::new(Channel::Pwm1).unwrap();
    pwm1.enable();
    pwm1.set_frequency(50.0, 0.0);

    let gpio = Gpio::new().unwrap();
    let mut pin14 = gpio.get(14).unwrap().into_output();
    pin14.set_low();

    std::thread::sleep(std::time::Duration::from_millis(5000));
    println!("PWM Channel 0 initialized!");
    println!("PWM Channel 1 initialized!");
}
