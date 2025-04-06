pub fn get_sidebrush_speed(speed: u32) {
    // Initialize node
    rosrust::init("subscriber");

    // Create subscriber
    println!("Subscribed to /sidebrush_controller/sidebrush_speed");
    // The subscriber is stopped when the returned object is destroyed
    let _subscriber_raii = rosrust::subscribe("/sidebrush_controller/sidebrush_speed", 100, |v: rosrust_msg::std_msgs::UInt32| {
        // Callback for handling received messages
        speed = v.data;
        println!("Received: {}", v.data);
    }).unwrap();

    // Block the thread until a shutdown signal is received
    rosrust::spin();
}
