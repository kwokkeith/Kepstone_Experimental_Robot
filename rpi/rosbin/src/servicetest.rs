use std::{time};
use rosrust_msg::std_msgs::UInt32;

pub fn get_sidebrush_speed() {
    let msg = rosrust_msg::std_srvs::TriggerReq::default();

    rosrust::init("servicetest");

    // Wait ten seconds for the service to appear
    rosrust::wait_for_service("/get_sidebrush_speed", Some(time::Duration::from_secs(10))).unwrap();

    // Create client for the service
    let client = rosrust::client::<UInt32>("/get_sidebrush_speed").unwrap();

    // Synchronous call that blocks the thread until a response is received
    dbg!(client.req(&msg).unwrap().unwrap());

    /*
    // Asynchronous call that can be resolved later on
    let retval = client.req_async(rosrust_msg::roscpp_tutorials::TwoIntsReq { a, b });
    rosrust::ros_info!("{} + {} = {}", a, b, retval.read().unwrap().unwrap().sum);
    */
}	
