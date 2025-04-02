use std::thread;
use std::time::Duration;
use rppal::gpio::Gpio;
use rppal::pwm::{Channel, Pwm};

pub struct Servo {
    pwm: Pwm,
    pub current_power: u32,
    pwm_period_ms: f64,
    pwm_frequency: f64,
}

impl Servo {
    pub fn new(pwm: Pwm) -> Self {
        pwm.enable();
        let pwm_frequency = 50.0;
        Servo {
            pwm: pwm,
            current_power: 0u32,
            pwm_frequency: 50.0,
            pwm_period_ms: 1000.0/pwm_frequency,
        }
    }

    pub fn move_to_angle(&mut self, servo_angle: f64, publisher: &rosrust::Publisher<rosrust_msg::bumperbot_controller::VacuumPowerFeedback>) {

        let target_power = Self::calculate_power_from_angle(servo_angle);
        while self.current_power < target_power {
            self.current_power += 1;
            self.pwm.set_frequency(self.pwm_frequency, self.get_duty());
            let mut msg = rosrust_msg::bumperbot_controller::VacuumPowerFeedback::default();
            msg.message = "OK".to_owned();
            msg.power = self.current_power;
            publisher.send(msg);
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        while self.current_power > target_power {
            self.current_power -= 1;
            self.pwm.set_frequency(self.pwm_frequency, self.get_duty());
            let mut msg = rosrust_msg::bumperbot_controller::VacuumPowerFeedback::default();
            msg.message = "OK".to_owned();
            msg.power = self.current_power;
            publisher.send(msg);
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }

    fn calculate_power_from_angle(mut angle: f64) -> u32 {
        if angle < 0.0 { angle = 0.0 }
        if angle > 180.0 { angle = 180.0 }

        (angle/180.0 * 800.0) as u32
    }

    fn calculate_duty_from_angle(mut angle: f64) -> f64 {
        if angle < 0.0 { angle = 0.0 }
        if angle > 180.0 { angle = 180.0 }

        let duty_cycle = angle/180.0 * 0.1 + 0.025;
        //let pulse_width_ms = 20.0 * duty_cycle;
        duty_cycle
    }

    pub fn get_duty(&self) -> f64 {
        self.current_power as f64 / 800.0 * 0.1 + 0.025
    }

    pub fn get_power(&self) -> u32 {
        self.current_power.clone()
    }

    pub fn get_pulse_width_ms(&self) -> f64 {
        self.get_duty() * self.pwm_period_ms
    }
}
