use std::thread;
use std::time::Duration;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use rppal::gpio::Gpio;
use rppal::pwm::{Channel, Pwm};
use rosrust_msg::sensor_msgs::{Joy, BatteryState};
use crate::servo;

pub fn main() {

    // Initialize node
    rosrust::init("dv8_rpi_vacuum");
    let pwm0 = Pwm::new(Channel::Pwm0).unwrap();
    let pwm1 = Pwm::new(Channel::Pwm1).unwrap();
    pwm1.enable();

    let rate = rosrust::rate(100.0);
    let mut servo = servo::Servo::new(pwm0);

    // Atomic Reference Counted Variables
    let mut vac_power = Arc::new(AtomicU32::new(0));
    let mut roller_speed = Arc::new(AtomicU32::new(0));

    let pub_topic = "/vacuum_controller/vacuum_power_feedback";
    let pub_topic_2 = "/rollerbrush_controller/rollerbrush_power_feedback";
    let sub_topic = "/vacuum_controller/vacuum_power";
    let sub_topic_2 = "/rollerbrush_controller/rollerbrush_power";

    // Create publisher
    let publisher = rosrust::publish(&pub_topic, 100).unwrap();
    let publisher_2 = rosrust::publish(&pub_topic_2, 100).unwrap();
    servo.move_to_angle(0.0, &publisher);

    // Create subscriber
    println!("Subscribed to {} and {}", &sub_topic, &sub_topic_2);

    let mut sub_vac_power = vac_power.clone();
    // The subscriber is stopped when the returned object is destroyed
    let _subscriber = rosrust::subscribe(sub_topic, 100, move |v: rosrust_msg::std_msgs::UInt32| {
        // Callback for handling received messages
        println!("Received: {} RPM", &v.data);
        sub_vac_power.store(v.data, Ordering::Relaxed);
    }).unwrap();

    let mut sub_roller_speed = roller_speed.clone();
    let _subscriber2 = rosrust::subscribe(sub_topic_2, 100, move |mut v: rosrust_msg::std_msgs::Float32| {
        // Callback for handling received messages
        if v.data > 1.0 { v.data = 1.0; }
        if v.data < 0.0 { v.data = 0.0; }
        println!("Rollerbrush: {}", &v.data);
        sub_roller_speed.store((v.data * 100.0) as u32, Ordering::Relaxed);
    }).unwrap();

    let mut joy_vac_power = vac_power.clone();
    let mut joy_roller_speed = roller_speed.clone();
    let _subscriber_joy = rosrust::subscribe("/joy1/joy", 100, move |v: Joy| {
        // Callback for handling received messages
        let d_pad_UPDOWN = v.axes.get(4).unwrap();
        let curr_vac_pwr = joy_vac_power.load(Ordering::Relaxed);
        let curr_roller_spd = joy_roller_speed.load(Ordering::Relaxed);
        println!("Rollerbrush: {}%", &curr_roller_spd);
        match d_pad_UPDOWN {
            -1.0 => {
                if curr_vac_pwr < 800 {
                    joy_vac_power.fetch_add(50, Ordering::Relaxed);
                };
                if curr_roller_spd < 100 {
                    joy_roller_speed.fetch_add(10, Ordering::Relaxed);
                }
            },
            1.0 => {
                if curr_vac_pwr > 0 {
                    joy_vac_power.fetch_sub(50, Ordering::Relaxed);
                };
                if curr_roller_spd > 0 {
                    joy_roller_speed.fetch_sub(10, Ordering::Relaxed);
                }
            },
            _ => {}
        };
    }).unwrap();

    let mut msg = rosrust_msg::bumperbot_controller::RollerBrushPowerFeedback::default();
    while rosrust::is_ok() {
        let vac_watts = vac_power.load(Ordering::Relaxed);
        let roller_spd = roller_speed.load(Ordering::Relaxed);
        pwm1.set_frequency(50.0, roller_spd as f64 / 100.0);
        msg.message = "OK".to_owned();
        msg.pos = "DOWN".to_owned();
        msg.power = format!("{:.2}", roller_spd).parse().unwrap();
        publisher_2.send(msg.clone());
        servo.move_to_angle(vac_watts as f64/800.0 * 180.0, &publisher);
        rate.sleep();
    }

    rosrust::spin();
}
