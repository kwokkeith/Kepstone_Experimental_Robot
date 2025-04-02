use std::thread;
use std::time::Duration;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use rppal::gpio::Gpio;
use rppal::pwm::{Channel, Pwm};
use rosrust_msg::sensor_msgs::{Joy, BatteryState};
use crate::bts7960_motor_control;

pub fn main() {
    let mut speed = Arc::new(AtomicU32::new(0));
    let mut oldbatt = Arc::new(AtomicU32::new(0));
    let mut speed2 = speed.clone();
    let mut speed3 = speed.clone();
    let mut target_speed = 0;
    let mut current_speed = 0;
    let gpio = Gpio::new().unwrap();
    let mut sidebrush_actuate_down_pin = gpio.get(27).unwrap().into_output();
    let mut sidebrush_actuate_up_pin = gpio.get(22).unwrap().into_output();

    // Initialize node
    rosrust::init("dv8_rpi_sidebrush");
    let pub_topic = "/sidebrush_controller/sidebrush_speed_feedback";

    // Create publisher
    let publisher = rosrust::publish(&pub_topic, 100).unwrap();
    // Create object that maintains 10Hz between sleep requests
    let rate = rosrust::rate(100.0);

    let pwm0 = Pwm::new(Channel::Pwm0).unwrap();
       
    // Create subscriber
    println!("Subscribed to /sidebrush_controller/sidebrush_speed");
    // The subscriber is stopped when the returned object is destroyed
    let _subscriber_raii = rosrust::subscribe("/sidebrush_controller/sidebrush_speed", 100, move |v: rosrust_msg::std_msgs::UInt32| {
        // Callback for handling received messages
        speed.clone().store(v.data, Ordering::Relaxed);
    }).unwrap();

    let _subscriber_batt = rosrust::subscribe("/sam/battery_state", 100, move |v: BatteryState| {
        // Callback for handling received messages
        if oldbatt.load(Ordering::Relaxed) != v.percentage as u32 {
            oldbatt.store(v.percentage as u32, Ordering::Relaxed);
            println!("Battery: {}%", v.percentage);
        }
    }).unwrap();

    let _subscriber_joy = rosrust::subscribe("/joy1/joy", 100, move |v: Joy| {
        // Callback for handling received messages
        println!("Buttons: {:?}", v.buttons);
        let brushspeed = speed3.clone();
        let oldspeed = speed3.clone().load(Ordering::Relaxed);
        if v.buttons[3] == 1 && oldspeed > 0 {
            brushspeed.store(oldspeed - 50, Ordering::Relaxed);
        }
        if v.buttons[0] == 1 && oldspeed < 300 {
            brushspeed.store(oldspeed + 50, Ordering::Relaxed);
        }
    }).unwrap();

    while rosrust::is_ok() {
        target_speed = speed2.load(Ordering::Relaxed);
        let mut msg = rosrust_msg::bumperbot_controller::SideBrushSpeedFeedback::default();
        msg.message = "OK".to_owned();

        if current_speed == 0 {
           sidebrush_actuate_down_pin.set_high();
           sidebrush_actuate_up_pin.set_low();
        } else {
           sidebrush_actuate_down_pin.set_low();
           sidebrush_actuate_up_pin.set_high();
        }

        if current_speed < target_speed && current_speed < 300 {
            current_speed += 1;
            if current_speed == 0 {
                msg.pos = "UP".to_owned();
            } else { 
                msg.pos = "DOWN".to_owned();
            }
            println!("Target Speed = {} RPM, Current Speed = {} RPM", target_speed, current_speed);
            msg.speed = current_speed;
            publisher.send(msg.clone()).unwrap();
        } 
        if current_speed > target_speed && current_speed > 0 {
            current_speed -= 1;
            if current_speed == 0 {
                msg.pos = "UP".to_owned();
            } else { 
                msg.pos = "DOWN".to_owned();
            }
            println!("Target Speed = {} RPM, Current Speed = {} RPM", target_speed, current_speed);
            msg.speed = current_speed;
            publisher.send(msg).unwrap();
        } 
        bts7960_motor_control::motor_control(&pwm0, current_speed as f64/300.0);

        // Sleep to maintain 10Hz rate
        rate.sleep();
        //thread::sleep(Duration::from_millis(10));
    }
    rosrust::spin();
}
