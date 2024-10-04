// Auto-generated. Do not edit!

// (in-package navigation.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class GetPixelPoseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPixelPoseRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPixelPoseRequest
    let len;
    let data = new GetPixelPoseRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'navigation/GetPixelPoseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Request
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPixelPoseRequest(null);
    return resolved;
    }
};

class GetPixelPoseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pixel_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('pixel_pose')) {
        this.pixel_pose = initObj.pixel_pose
      }
      else {
        this.pixel_pose = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPixelPoseResponse
    // Serialize message field [pixel_pose]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.pixel_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPixelPoseResponse
    let len;
    let data = new GetPixelPoseResponse(null);
    // Deserialize message field [pixel_pose]
    data.pixel_pose = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'navigation/GetPixelPoseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7c5941ad5d679949278959effa1051da';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Response
    geometry_msgs/Point pixel_pose
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPixelPoseResponse(null);
    if (msg.pixel_pose !== undefined) {
      resolved.pixel_pose = geometry_msgs.msg.Point.Resolve(msg.pixel_pose)
    }
    else {
      resolved.pixel_pose = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetPixelPoseRequest,
  Response: GetPixelPoseResponse,
  md5sum() { return '7c5941ad5d679949278959effa1051da'; },
  datatype() { return 'navigation/GetPixelPose'; }
};
