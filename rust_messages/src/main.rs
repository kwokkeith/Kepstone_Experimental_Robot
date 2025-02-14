/*
fn main() {
    // Initialize node
    rosrust::init("listener");

    // Create subscriber
    println!("Subscribed to litter_memory");
    // The subscriber is stopped when the returned object is destroyed
    let _subscriber_raii = rosrust::subscribe("litter_memory", 100, |v: rosrust_msg::std_msgs::String| {
        // Callback for handling received messages
        println!("Received: {}", v.data);
    }).unwrap();

    // Block the thread until a shutdown signal is received
    rosrust::spin();
}*/

fn main() {
    println!("Hello World");
}
