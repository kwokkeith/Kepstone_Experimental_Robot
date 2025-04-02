
fn main() {
    let topic = "/sidebrush_controller/sidebrush_speed";

    // Initialize node
    rosrust::init("sidebrush_publisher");

    // Create publisher
    let publisher = rosrust::publish(&topic, 100).unwrap();

    let mut count = 0;

    // Create object that maintains 10Hz between sleep requests
    let rate = rosrust::rate(10.0);

    // Breaks when a shutdown signal is sent
    while rosrust::is_ok() {
        // Create string message
        let mut msg = rosrust_msg::bumperbot_controller::SideBrushSpeedFeedback::default();
        msg.speed = 55u32;
        msg.message = "Err(e)".to_owned();

        // Send string message to topic via publisher
        publisher.send(msg).unwrap();

        // Sleep to maintain 10Hz rate
        rate.sleep();

        count += 1;
    }
}
