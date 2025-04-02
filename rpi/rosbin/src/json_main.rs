
fn main() {

    // Initialize node
    rosrust::init("status_subscriber");

    let topic = "/i2r_system_monitor/status";

    // Create subscriber
    println!("Subscribed to {}", &topic);
    // The subscriber is stopped when the returned object is destroyed
    let _subscriber_raii = rosrust::subscribe(
        &topic,
        100,
        |v: rosrust_msg::std_msgs::String| {
            //println!("Received: {}", v.data);
            let j = json::parse(&v.data).unwrap();
            println!("{:#}",&j["payload"]["data"][1]["system_status_data"]["data"]["system_status"]);
    }).unwrap();

    // Block the thread until a shutdown signal is received
    rosrust::spin();
}
