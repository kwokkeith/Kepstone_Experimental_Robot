
fn main() {
    let topic = "/jon_is_a_fucker";

    // Initialize node
    rosrust::init("sidebrush_publisher");

    // Create subscriber
    println!("Subscribed to {}", &topic);
    // The subscriber is stopped when the returned object is destroyed
    let _subscriber_raii = rosrust::subscribe(&topic, 100, |v: rosrust_msg::bumperbot_controller::SideBrushSpeedFeedback| {
        // Callback for handling received messages
        println!("speed: {}", v.speed);
        println!("message: {}", v.message);
    }).unwrap();

    // Block the thread until a shutdown signal is received
    rosrust::spin();
}
