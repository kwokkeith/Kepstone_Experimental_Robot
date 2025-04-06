use std::thread;
use std::time::Duration;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
//use rppal::gpio::Gpio;
use rppal::pwm::{Channel, Pwm};
use rosrust_msg::sensor_msgs::{Joy, BatteryState};
mod servo;

fn main() {

    println!("vacuum2");
    // Initialize node
    rosrust::init("dv8_rpi_vacuum");
    let pwm0 = Pwm::new(Channel::Pwm0).unwrap();
    let rate = rosrust::rate(100.0);

    // Atomic Reference Counted Variables
    let mut speed = Arc::new(AtomicU32::new(0));

    let pub_topic = "/vacuum_controller/vacuum_power_feedback";
    let sub_topic = "/vacuum_controller/vacuum_power";

    // Create publisher
    let publisher = rosrust::publish(&pub_topic, 100).unwrap();

    // Create subscriber
    println!("Subscribed to {}", &sub_topic);

    let mut sub_speed = speed.clone();
    // The subscriber is stopped when the returned object is destroyed
    let _subscriber = rosrust::subscribe(sub_topic, 100, move |v: rosrust_msg::std_msgs::UInt32| {
        // Callback for handling received messages
        sub_speed.store(v.data, Ordering::Relaxed);
        let mut msg = rosrust_msg::bumperbot_controller::VacuumPowerFeedback::default();
        msg.message = "Ok(())".to_owned();
        msg.power = v.data;
        publisher.send(msg);
    }).unwrap();

    while rosrust::is_ok() {
        servo::motor_control(&pwm0, speed.load(Ordering::Relaxed) as f64/800.0 * 180.0);
        rate.sleep();
    }

    rosrust::spin();
}
