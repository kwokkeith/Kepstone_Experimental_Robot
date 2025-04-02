use std::thread;
use std::time::Duration;
use rppal::pwm::{Channel, Pwm};

fn main() {

    println!("hihi");
    const PWM_FREQ: f64 = 50.0;
    //let rate = rosrust::rate(100.0);

    let pwm = Pwm::new(Channel::Pwm0).unwrap();
    pwm.enable();

    loop {
    //while rosrust::is_ok() {
        for i in 50..=250 {
            let duty = i as f64/2000.0;
            pwm.set_frequency(PWM_FREQ, duty);
            thread::sleep(Duration::from_millis(50));
        }
        thread::sleep(Duration::from_millis(5000));
        for i in (50..=250).rev() {
            let duty = i as f64/2000.0;
            pwm.set_frequency(PWM_FREQ, duty);
            thread::sleep(Duration::from_millis(50));
        }
        thread::sleep(Duration::from_millis(5000));
    }
    //rosrust::spin();
}


