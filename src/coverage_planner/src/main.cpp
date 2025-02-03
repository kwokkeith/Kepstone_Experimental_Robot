#include "coverage_planner.h"
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <ostream>
#include <string>
#define DENSE_PATH
#include <fstream>
#include <cmath>
#include <vector>
#include <utility>
#include <ros/ros.h>
#include <ros/package.h>
#include <unistd.h>
#include <limits.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <algorithm>

std::string PARAMETER_FILE_PATH;
std::string WAYPOINT_COORDINATE_FILE_PATH;
std::string EXTERNAL_POLYGON_FILE_PATH;
std::string REGION_OF_INTEREST_FILE_PATH;

std::string image_path;
uint robot_width;
uint robot_height;
uint open_kernel_width;
uint open_kernel_height;
uint dilate_kernel_width;
uint dilate_kernel_height;
int sweep_step;
bool show_cells;
bool mouse_select_start;
bool manual_orientation;
bool crop_region;
uint start_x;
uint start_y;
uint subdivision_dist;
std::vector<cv::Point> selected_points;
cv::Mat img_copy;
cv::Point top_left;
std::string mapName;
std::string anglesArray_msg;
std::vector<std::string> angles_array;
bool angles_array_received = false;
bool mapName_received = false;
bool editState_received = false;

Point_2 starting_point (0,0);

bool LoadParameters() {
  // Print the current working directory
  char cwd[PATH_MAX];
  if (getcwd(cwd, sizeof(cwd)) != NULL) {
    ROS_INFO("Current working directory: %s", cwd);
  } else {
    ROS_ERROR("Failed to get current working directory");
  }

  // Load parameters from config file
  std::ifstream in(PARAMETER_FILE_PATH);
  if (!in.is_open()) {
    ROS_ERROR("Failed to open parameter file: %s", PARAMETER_FILE_PATH.c_str());
    return false;
  }

  std::string param;
  ROS_INFO("Starting parameter load");

  while (in >> param) {
    ROS_INFO("Reading parameter: %s", param.c_str());
    if (param == "IMAGE_PATH") {
      in >> image_path;
      ROS_INFO("Loaded IMAGE_PATH: %s", image_path.c_str());

      // Resolve the image path relative to the current working directory
      std::string package_path = ros::package::getPath("coverage_planner");
      std::string GUI_package_path = ros::package::getPath("bumperbot_graphical_interface");
      image_path = package_path + "/" + image_path;
      ROS_INFO("Resolved IMAGE_PATH: %s", image_path.c_str());
    } else if (param == "ROBOT_SIZE") {
      in >> robot_width >> robot_height;
      ROS_INFO("Loaded ROBOT_SIZE: %u, %u", robot_width, robot_height);
    } else if (param == "MORPH_SIZE") {
      in >> open_kernel_width >> open_kernel_height;
      ROS_INFO("Loaded MORPH_SIZE: %u, %u", open_kernel_width, open_kernel_height);
    } else if (param == "OBSTACLE_INFLATION") {
      in >> dilate_kernel_width >> dilate_kernel_height;
      ROS_INFO("Loaded OBSTACLE_INFLATION: %u, %u", dilate_kernel_width, dilate_kernel_height);
    } else if (param == "SWEEP_STEP") {
      in >> sweep_step;
      ROS_INFO("Loaded SWEEP_STEP: %d", sweep_step);
    } else if (param == "SHOW_CELLS") {
      in >> show_cells;
      ROS_INFO("Loaded SHOW_CELLS: %d", show_cells);
    } else if (param == "MOUSE_SELECT_START") {
      in >> mouse_select_start;
      ROS_INFO("Loaded MOUSE_SELECT_START: %d", mouse_select_start);
    } else if (param == "START_POS") {
      in >> start_x >> start_y;
      ROS_INFO("Loaded START_POS: %u, %u", start_x, start_y);
    } else if (param == "SUBDIVISION_DIST") {
      in >> subdivision_dist;
      ROS_INFO("Loaded SUBDIVISION_DIST: %u", subdivision_dist);
    // } else if (param == "MANUAL_ORIENTATION") {
    //   in >> manual_orientation;
    //   ROS_INFO("Loaded MANUAL_ORIENTATION: %d", manual_orientation);
    } else if (param == "CROP_REGION") {
      in >> crop_region;
      ROS_INFO("Loaded CROP_REGION: %d", crop_region);
    } else {
      ROS_WARN("Unknown parameter: %s", param.c_str());
    }
  }

  in.close();
  ROS_INFO("Parameter load complete");
  return true;
}

void roiPointsCallback(const std_msgs::String::ConstPtr& msg) {
    std::istringstream iss(msg->data);
    selected_points.clear();
    int x, y;
    while (iss >> x >> y) {
        selected_points.push_back(cv::Point(x, y));
    }
}

void startingPointsCallback(const std_msgs::String::ConstPtr& msg){
  std::istringstream iss(msg->data);
  int x, y;
  iss >> x >> y;
  starting_point = Point_2(x, y);
  // /ROS_INFO("Received starting point: (%d, %d)", x, y);
}

void anglesArrayCallback(const std_msgs::String::ConstPtr& msg) {
    anglesArray_msg = msg->data; //Save map name
    ROS_INFO("Received anglesArray_msg: %s", msg->data.c_str());
    angles_array_received = true;

    angles_array.push_back(msg->data);
}

void mapNameCallback(const std_msgs::String::ConstPtr& msg) {
    mapName = msg->data; //Save map name
    ROS_INFO("Received map name: %s", msg->data.c_str());
    mapName_received = true;
}

void editStateCallback (const std_msgs::Bool::ConstPtr& msg){
  ROS_INFO("Received edit state: %d", msg->data);
  if (msg->data == true) {
    manual_orientation = true;
  } else {
    manual_orientation = false;
  }
  editState_received = true;
}

// void mouseCallback(int event, int x, int y, int flags, void* param) {
//   if (event == cv::EVENT_LBUTTONDOWN && selected_points.size() < 4) {
//     selected_points.push_back(cv::Point(x, y));
//     std::cout << "Point"<< selected_points.size() << ": " << x << y << std::endl;
//     cv::circle(img_copy, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
//     cv::imshow("Select 4 points", img_copy);
//   }
// }

// Function to crop and transform the image based on selected points
std::pair<cv::Mat, cv::Point> cropAndTransform(const cv::Mat& img) {
    // Find the bounding rectangle for the selected points
    std::vector<int> x_coords, y_coords;
    for (const auto& p : selected_points) {
        x_coords.push_back(p.x);
        y_coords.push_back(p.y);
    }

    // Determine the top-left and bottom-right corners
    int x_min = *std::min_element(x_coords.begin(), x_coords.end());
    int x_max = *std::max_element(x_coords.begin(), x_coords.end());
    int y_min = *std::min_element(y_coords.begin(), y_coords.end());
    int y_max = *std::max_element(y_coords.begin(), y_coords.end());

    cv::Mat cropped_img = img(cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min)).clone();
    top_left=cv::Point(x_min,y_min);
    return std::make_pair(cropped_img, top_left);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "coverage_planner_node");
  ros::NodeHandle nh;
  ros::Publisher point_pub = nh.advertise<std_msgs::String>("/canvas_messenger", 1000);
  ROS_INFO("Coverage Planner Node Started");
  // Get the parameter file path
  nh.getParam("parameter_file_path", PARAMETER_FILE_PATH);

  // Resolve file paths relative to the package directory
  std::string package_path = ros::package::getPath("coverage_planner");
  std::string GUI_package_path = ros::package::getPath("bumperbot_graphical_interface");
  
  EXTERNAL_POLYGON_FILE_PATH = package_path + "/result/ext_polygon_coord.txt";
  REGION_OF_INTEREST_FILE_PATH = package_path + "/result/roi_points.txt";

  // Load parameters from config file
  if (!LoadParameters()) {
    ROS_INFO("ERROR LOADING PARAMETERS");
    return EXIT_FAILURE;
  }

  ros::Subscriber sub = nh.subscribe("/new_map_name", 1, mapNameCallback);
  while (ros::ok() && !mapName_received) {
    ros::spinOnce();
    ros::Duration(0.1).sleep(); //Sleep for 100 ms
  };
  WAYPOINT_COORDINATE_FILE_PATH = GUI_package_path + "/web/ros-frontend/public/temp_zone/waypoints_"+mapName+".txt";
  
  ros:: Subscriber editStatesub = nh.subscribe("/edit_state", 1, editStateCallback);
  while (ros::ok() && !editState_received) {
    ros::spinOnce();
    ros::Duration(0.1).sleep(); //Sleep for 100 ms
    ROS_INFO(editState_received ? "Edit state received" : "Edit state not received");
  };

  // Read image to be processed
  cv::Mat original_img = cv::imread(image_path);
  //cv::imshow("Original Image", original_img);
  cv::Mat img = cv::imread(image_path);
  img_copy = img.clone();

  if(crop_region){
    // cv::imshow("Select 4 points", img_copy);
    // cv::setMouseCallback("Select 4 points", mouseCallback, nullptr);
    // cv::waitKey(0);
    // cv::destroyWindow("Select 4 points");

    ros::Subscriber roi_sub = nh.subscribe("/roi_points",4, roiPointsCallback); //only use this if ROI is selected from webapp!!
    while (selected_points.size() < 4 && ros::ok()) {
      ros::spinOnce();
      ros::Duration(0.1).sleep(); //Sleep for 100 ms
    };
    
    auto crop = cropAndTransform(img);
    cv::Mat result = crop.first;
    top_left=crop.second; //redundant?
    if (!result.empty()) {
        //cv::imshow("Cropped Image", result);
        //cv::waitKey(0);
        //cv::destroyWindow("Cropped Image");
        img=result;
    }
    start_x = top_left.x;
    start_y = top_left.y;
    // ROS_INFO("Using top-left corner as starting point: (%d, %d)", start_x, start_y);
  }

  std::cout << "Read map" << std::endl;
  std::cout << "Pre-Processing map image" << std::endl;

  // Image Pre-processing (Reduce noise of image)
  cv::Mat gray;
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

  cv::Mat img_ = gray.clone();

  // Takes every pixel and declare to be only black or white
  // Binarizes the image (Making contrast clear)
  cv::threshold(img_, img_, 250, 255, 0);

  // Robot radius (Size) is defined here
  std::cout << "--Applying morphological operations onto image--" << std::endl;

  // Makes kernel in an ellipse shape of a certain size
  // And runs through the entire image and sets each kernel batch, all pixels
  // in the kernel, to the minimum value of that kernel (0 for black)
  cv::Mat erode_kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(robot_width, robot_height),
      cv::Point(-1, -1)); // size: robot radius
  cv::morphologyEx(img_, img_, cv::MORPH_ERODE, erode_kernel);
  std::cout << "Erosion Kernel for robot size applied" << std::endl;

  //  Applied after the above erosion kernel to enhance image
  //  Can use MORPH_RECT, MORPH_ELLIPSE
  cv::Mat open_kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(open_kernel_width, open_kernel_height),
      cv::Point(-1, -1));
  cv::morphologyEx(img_, img_, cv::MORPH_OPEN, open_kernel);
  std::cout << "Open Kernel applied" << std::endl;

  // To inflate the obstacles on the map
  // Invert the image so that black walls become white
  std::cout << "--Inverting the image to apply dilation on black walls--" << std::endl;
  cv::bitwise_not(img_, img_);  // Invert the image

  // Inflate the walls (now white) by dilating them
  std::cout << "--Inflating walls by dilating the obstacles--" << std::endl;
  cv::Mat dilation_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_kernel_width, dilate_kernel_height), cv::Point(-1, -1));
  cv::dilate(img_, img_, dilation_kernel);
  std::cout << "Dilation applied to inflate the walls" << std::endl;

  // Invert the image back to original (black walls)
  std::cout << "--Reverting the image back to original (black walls)--" << std::endl;
  cv::bitwise_not(img_, img_);  // Invert the image back

  // TODO: SECOND RUN OF Preprocessing if needed

  /*// Applied after the above erosion kernel to enhance image*/
  /*// Can use MORPH_RECT, MORPH_ELLIPSE*/
  /*open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10),*/
  /*                                        cv::Point(-1, -1));*/
  /*cv::morphologyEx(img_, img_, cv::MORPH_OPEN, open_kernel);*/
  /*std::cout << "Open Kernel applied" << std::endl;*/
  /**/
  ROS_INFO("Displaying image for preprocessing");
  //cv::imshow("preprocess", img_);
  //cv::waitKey();
  std::string preprocess_img_path = package_path + "/result/processed_img.png";
  cv::imwrite(preprocess_img_path, img_);
  std::cout << "Preprocessed image saved to: " << preprocess_img_path << std::endl;
 //cv::destroyWindow("preprocess");

  std::cout << std::string(50, '-') << std::endl;

  std::vector<std::vector<cv::Point>> cnts;
  std::vector<cv::Vec4i> hierarchy; // index: next, prev, first_child, parent
  cv::findContours(img_, cnts, hierarchy, cv::RETR_TREE,
                    cv::CHAIN_APPROX_SIMPLE);

  std::vector<int> cnt_indices(cnts.size());
  std::iota(cnt_indices.begin(), cnt_indices.end(), 0);
  std::sort(cnt_indices.begin(), cnt_indices.end(), [&cnts](int lhs, int rhs) {
    return cv::contourArea(cnts[lhs]) > cv::contourArea(cnts[rhs]);
  });
  int ext_cnt_idx = cnt_indices.front();

  cv::Mat cnt_canvas = img.clone();
  cv::drawContours(cnt_canvas, cnts, ext_cnt_idx, cv::Scalar(0, 0, 255));
  std::vector<std::vector<cv::Point>> contours;
  contours.emplace_back(cnts[ext_cnt_idx]);

  // find all the contours of obstacle
  for (int i = 0; i < hierarchy.size(); i++) {
    if (hierarchy[i][3] == ext_cnt_idx) { // parent contour's index equals to
                                          // external contour's index
      contours.emplace_back(cnts[i]);
      cv::drawContours(cnt_canvas, cnts, i, cv::Scalar(255, 0, 0));
    }
  }
  //    cv::imshow("contours", cnt_canvas);

  cv::Mat cnt_img = cv::Mat(img.rows, img.cols, CV_8UC3);
  cnt_img.setTo(255);
  for (int i = 0; i < contours.size(); i++) {
    cv::drawContours(cnt_img, contours, i, cv::Scalar(0, 0, 0));
  }
  //    cv::imshow("only contours", cnt_img);

  cv::Mat poly_canvas = original_img.clone();
  std::vector<cv::Point> poly;
  std::vector<std::vector<cv::Point>> polys;

  for (auto &contour : contours) {
    cv::approxPolyDP(contour, poly, 3, true);
    if(crop_region){
      std::vector<cv::Point> translated_poly;
      for (const auto& point : poly) {
          translated_poly.push_back(point + top_left);
      }
      polys.emplace_back(translated_poly);
    } else {
      polys.emplace_back(poly);
    }
    poly.clear();
  }
  for (int i = 0; i < polys.size(); i++) {
    cv::drawContours(poly_canvas, std::vector<std::vector<cv::Point>>{polys[i]}, -1, cv::Scalar(255, 0, 255));
  }

  int publish_count = 0;

  while (ros::ok() && publish_count < 2)
  {
    std::ostringstream oss;
    for (const auto& poly : polys) {
      oss << "[";
      for (const auto& point : poly) {
        oss << point.x << " " << point.y << std::endl;
      }
      oss << "],";
    }
    std_msgs::String msg;
    msg.data = oss.str();
    point_pub.publish(msg);

    publish_count++;
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
  

  //cv::imshow("polygons", poly_canvas);
  //cv::waitKey();
  //cv::destroyWindow("polygons");

  cv::Mat poly_img = cv::Mat(img.rows, img.cols, CV_8UC3);
  poly_img.setTo(255);
  for (int i = 0; i < polys.size(); i++) {
    cv::drawContours(poly_img, polys, i, cv::Scalar(0, 0, 0));
  }

  // Extract the vertices of the external Polygons

  //    cv::imshow("only polygons", poly_img);
  //    cv::waitKey();/your-docusaurus-site.example.com

  // compute main direction

  // [0,180)
  std::vector<int> line_deg_histogram(180);
  double line_len; // weight
  double line_deg;
  int line_deg_idx;

  cv::Mat line_canvas = img.clone();
  auto ext_poly = polys.front();
  ext_poly.emplace_back(ext_poly.front());
  for (int i = 1; i < ext_poly.size(); i++) {
    line_len = std::sqrt(std::pow((ext_poly[i].x - ext_poly[i - 1].x), 2) +
                         std::pow((ext_poly[i].y - ext_poly[i - 1].y), 2));
    // y-axis towards up, x-axis towards right, theta is from x-axis to y-axis
    line_deg = std::round(atan2(-(ext_poly[i].y - ext_poly[i - 1].y),
                                (ext_poly[i].x) - ext_poly[i - 1].x) /
                          M_PI * 180.0);        // atan2: (-180, 180]
    line_deg_idx = (int(line_deg) + 180) % 180; // [0, 180)
    line_deg_histogram[line_deg_idx] += int(line_len);

    //          std::cout<<"deg: "<<line_deg_idx<<std::endl;
    //          cv::line(line_canvas, ext_poly[i], ext_poly[i-1],
    //          cv::Scalar(255,255,0)); cv::imshow("lines",line_canvas);
    //          cv::waitKey();
  }

  //    cv::waitKey();

  auto it =
      std::max_element(line_deg_histogram.begin(), line_deg_histogram.end());
  int main_deg = (it - line_deg_histogram.begin());
  std::cout << "main deg: " << main_deg << std::endl;

  // file stream to write external polygon vertices to
  std::ofstream out_ext_poly(EXTERNAL_POLYGON_FILE_PATH);

  // construct polygon with holes

  std::vector<cv::Point> outer_poly = polys.front();
  polys.erase(polys.begin());
  std::vector<std::vector<cv::Point>> inner_polys = polys;

  Polygon_2 outer_polygon;
  out_ext_poly << outer_poly.size() << std::endl;

  for (const auto &point : outer_poly) {
    outer_polygon.push_back(Point_2(point.x, point.y));
    out_ext_poly << point.x << " " << point.y << std::endl;
  }

  // close the file stream
  out_ext_poly.close();

  int num_holes = inner_polys.size();
  std::vector<Polygon_2> holes(num_holes);
  for (int i = 0; i < inner_polys.size(); i++) {
    for (const auto &point : inner_polys[i]) {
      holes[i].push_back(Point_2(point.x, point.y));
    }
  }

  PolygonWithHoles pwh(outer_polygon, holes.begin(), holes.end());

  std::cout << "constructed polygons" << std::endl;

  // cell decomposition
  // TODO: Bottleneck for memory space

  std::cout << "Performing cell decomposition" << std::endl;

  // To measure the time it takes to execute cell decomposition
  auto start_time = std::chrono::high_resolution_clock::now();
  std::vector<Polygon_2> bcd_cells;

  //    polygon_coverage_planning::computeBestTCDFromPolygonWithHoles(pwh,
  //    &bcd_cells);
  polygon_coverage_planning::computeBestBCDFromPolygonWithHoles(pwh,
                                                                &bcd_cells);

  // Helper: Compute centroid of polygon
  auto computeCentroid = [](const Polygon_2 &poly) -> Point_2 {
    double sum_x = 0, sum_y = 0;
    int count = 0;
    for (const auto &pt : poly) {
      sum_x += CGAL::to_double(pt.x());
      sum_y += CGAL::to_double(pt.y());
      ++count;
    }
    return Point_2(sum_x / count, sum_y / count);
  };

  // Helper: Compute Euclidean distance from (0,0)
  auto distanceFromOrigin = [](const Point_2 &pt) -> double {
    return std::sqrt(CGAL::to_double(pt.x()*pt.x() + pt.y()*pt.y()));
  };

  // After computing bcd_cells, sort them based on centroid distance from (0,0)
  std::sort(bcd_cells.begin(), bcd_cells.end(),
            [&](const Polygon_2 &a, const Polygon_2 &b) {
              return distanceFromOrigin(computeCentroid(a)) < distanceFromOrigin(computeCentroid(b));
            });                                                           

  auto end_time = std::chrono::high_resolution_clock::now();
  auto execution_time = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time);
  long double ms = execution_time.count();
  long double s = ms / 1000000;
  std::cout << "Cell decomposition complete in " << s << "s" << std::endl;

  // test decomposition
  if (show_cells) {
    std::vector<std::vector<cv::Point>> bcd_polys;
    std::vector<cv::Point> bcd_poly;

    for (const auto &cell : bcd_cells) {
      for (int i = 0; i < cell.size(); i++) {
        bcd_poly.emplace_back(cv::Point(CGAL::to_double(cell[i].x()),
                                        CGAL::to_double(cell[i].y())));
      }
      bcd_polys.emplace_back(bcd_poly);
      bcd_poly.clear();
    }

    for (int i = 0; i < bcd_polys.size(); i++) {
      cv::drawContours(poly_img, bcd_polys, i, cv::Scalar(255, 0, 255));
      cv::imshow("bcd", poly_img);
      cv::waitKey();
    }
    cv::imshow("bcd", poly_img);
    cv::waitKey();
  }

  auto cell_graph = calculateDecompositionAdjacency(bcd_cells);

  // Get starting point from mouse click
  Point_2 start;
  if (mouse_select_start) {
    std::cout << "Select starting point" << std::endl;

    ros::Subscriber start_point_sub = nh.subscribe("/start_point",1, startingPointsCallback);
    while (starting_point.x() == 0 && starting_point.y() == 0 && ros::ok()) {
      ros::spinOnce();
      ros::Duration(0.5).sleep(); //Sleep for 100 ms
    };
    //std::cout << "Starting point selected: (" << starting_point.x() << ", " << starting_point.y() << ")" << std::endl;
    start = starting_point;
    //start = getStartingPoint(poly_canvas);
  } else {
    start = Point_2(start_x, start_y);
    std::cout << "Starting point configured: (" << start.x() << ", " << start.y() << ")" << std::endl;
  }

  int starting_cell_idx = getCellIndexOfPoint(bcd_cells, start);
  auto cell_idx_path = getTravellingPath(cell_graph, starting_cell_idx) ;
  std::cout << "path length: " << cell_idx_path.size() << std::endl;
  std::cout << "start";
  for (auto &cell_idx : cell_idx_path) {
    std::cout << "->" << cell_idx;
  }
  std::cout << std::endl;

  // sweep_step (distance per step in sweep),
  // int sweep_step = 5;
  std::vector<std::vector<Point_2>> cells_sweeps;
  std::vector<std::vector<cv::Point>> all_bcd_poly_contours;

  if (manual_orientation) {
    //TODO This part still needs to be "sorted" 
    ros::Subscriber new_angle_array = nh.subscribe("/new_angle_array", 1, anglesArrayCallback);
    while (!angles_array_received && ros::ok()) {
      ros::spinOnce();
      ros::Duration(0.5).sleep();
    };

    // Store user-defined angles for sweep direction
    std::vector<double> polygon_sweep_directions;

    for (size_t i = 0; i < bcd_cells.size(); ++i) {
      // Instead of showing the polygon on screen, we build the contour for later publishing.
      std::vector<cv::Point> current_polygon;
      for (int j = 0; j < bcd_cells[i].size(); ++j) {
          current_polygon.push_back(cv::Point(CGAL::to_double(bcd_cells[i][j].x()), 
                                              CGAL::to_double(bcd_cells[i][j].y())));
      }
      // Save the contour for publishing
      all_bcd_poly_contours.push_back(current_polygon);
      
      // Compute best sweep direction
      Direction_2 best_sweep_dir;
      double min_altitude = polygon_coverage_planning::findBestSweepDir(bcd_cells[i], &best_sweep_dir);

      if (std::isnan(CGAL::to_double(best_sweep_dir.dx())) || std::isnan(CGAL::to_double(best_sweep_dir.dy()))) {
        std::cerr << "Invalid best sweep direction detected for polygon " << i << std::endl;
        continue;
      }

      double best_sweep_angle = std::atan2(CGAL::to_double(best_sweep_dir.dy()), CGAL::to_double(best_sweep_dir.dx())) * 180.0 / M_PI;
      std::cout << "Best sweep direction for polygon " << i+1 << " is: " << best_sweep_angle << " degrees" << std::endl;

      double user_angle = best_sweep_angle;  // default to best sweep angle

      if (!angles_array.empty()) {
        std::string json_str = angles_array[0];  // assuming single message JSON structure
        std::string key = "\"" + std::to_string(i) + "\"";
        std::size_t keyPos = json_str.find(key);
        if (keyPos != std::string::npos) {
          std::size_t angleKeyPos = json_str.find("\"angle\"", keyPos);
          if (angleKeyPos != std::string::npos) {
            std::size_t colonPos = json_str.find(":", angleKeyPos);
            if (colonPos != std::string::npos) {
              std::size_t commaPos = json_str.find(",", colonPos);
              std::size_t endPos = (commaPos != std::string::npos) ? commaPos : json_str.find("}", colonPos);
              if (endPos != std::string::npos) {
                std::string angleStr = json_str.substr(colonPos + 1, endPos - colonPos - 1);
                try {
                  double parsed_angle = std::stod(angleStr);
                  user_angle = parsed_angle;
                  std::cout << "Using angle from new_angle_array for polygon " << i + 1 
                            << ": " << user_angle << " degrees" << std::endl;
                } catch (const std::exception& e) {
                  std::cerr << "Invalid angle format in new_angle_array for polygon " << i + 1 
                            << ". Using best sweep angle." << std::endl;
                }
              }
            }
          } else {
            std::cout << "No angle entry found in new_angle_array for polygon " << i + 1 
                      << ". Using best sweep angle." << std::endl;
          }
        } else {
          std::cout << "No entry found for polygon " << i + 1 
                    << " in new_angle_array. Using best sweep angle." << std::endl;
        }
      } else {
        std::cout << "new_angle_array is empty. Using best sweep angle for polygon " << i + 1 << "." << std::endl;
      }
      polygon_sweep_directions.push_back(user_angle);
      
      // Continue processing the sweep for manual orientation as before...
      std::vector<Point_2> cell_sweep;
      double angle_in_radians = user_angle * (M_PI / 180.0);
      if (std::isnan(std::cos(angle_in_radians)) || std::isnan(std::sin(angle_in_radians))) {
        std::cerr << "Invalid sweep direction for polygon " << i << ". Using default direction." << std::endl;
        continue;
      }
      Direction_2 user_defined_dir(std::cos(angle_in_radians), std::sin(angle_in_radians));
      polygon_coverage_planning::visibility_graph::VisibilityGraph vis_graph(bcd_cells[i]);

      try {
        polygon_coverage_planning::computeSweep(bcd_cells[i], vis_graph, sweep_step, user_defined_dir, true, &cell_sweep);
        if (cell_sweep.empty()) {
          std::cerr << "Warning: Sweep for polygon " << i + 1 << " returned no points." << std::endl;
        } else {
          std::cout << "Successfully constructed sweep for polygon " << i + 1 << std::endl;
        }
        cells_sweeps.emplace_back(cell_sweep);
      } catch (const std::exception& e) {
        std::cerr << "Error constructing sweep for polygon " << i + 1 << ": " << e.what() << std::endl;
      }
    }
  }
  else {  // automatic orientation
    for (size_t i = 0; i < bcd_cells.size(); ++i) {
      std::vector<Point_2> cell_sweep;
      Direction_2 best_dir;
      polygon_coverage_planning::findBestSweepDir(bcd_cells[i], &best_dir);
      polygon_coverage_planning::visibility_graph::VisibilityGraph vis_graph(bcd_cells[i]);

      bool counter_clockwise = true;
      polygon_coverage_planning::computeSweep(bcd_cells[i], vis_graph, sweep_step, best_dir, counter_clockwise, &cell_sweep);
      cells_sweeps.emplace_back(cell_sweep);

      // Extract the contour for publishing
      std::vector<cv::Point> current_polygon;
      for (int j = 0; j < bcd_cells[i].size(); j++){
        current_polygon.push_back(cv::Point(CGAL::to_double(bcd_cells[i][j].x()), 
                                            CGAL::to_double(bcd_cells[i][j].y())));
      }
      all_bcd_poly_contours.push_back(current_polygon);
    }
  }

  // Publishing the contours (common to both manual and automatic orientation)
  int bcd_contour_publishCount = 0;
  while (ros::ok() && bcd_contour_publishCount < 2)
  {
    std::ostringstream oss;
    for (const auto& poly : all_bcd_poly_contours) {
      oss << "[";
      for (const auto& point : poly) {
        oss << point.x << " " << point.y << "\n";
      }
      oss << "],";
    }
    std_msgs::String msg;
    msg.data = oss.str();
    point_pub.publish(msg);

    bcd_contour_publishCount++;
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  auto cell_intersections = calculateCellIntersections(bcd_cells, cell_graph);

  std::vector<Point_2> way_points;

#ifdef DENSE_PATH
  Point_2 point = start;
  std::list<Point_2> next_candidates;
  Point_2 next_point;
  std::vector<Point_2> shortest_path;

  if (doReverseNextSweep(start, cells_sweeps[cell_idx_path.front()])) {
    shortest_path = getShortestPath(bcd_cells[cell_idx_path.front()], start,
                                    cells_sweeps[cell_idx_path.front()].back());
    way_points.insert(way_points.end(), shortest_path.begin(),
                      std::prev(shortest_path.end()));
  } else {
    shortest_path =
        getShortestPath(bcd_cells[cell_idx_path.front()], start,
                        cells_sweeps[cell_idx_path.front()].front());
    way_points.insert(way_points.end(), shortest_path.begin(),
                      std::prev(shortest_path.end()));
  }

  point = way_points.back();

  for (size_t i = 0; i < cell_idx_path.size(); ++i) {
    // has been cleaned?
    if (!cell_graph[cell_idx_path[i]].isCleaned) {
      // need to reverse?
      if (doReverseNextSweep(point, cells_sweeps[cell_idx_path[i]])) {
        way_points.insert(way_points.end(),
                          cells_sweeps[cell_idx_path[i]].rbegin(),
                          cells_sweeps[cell_idx_path[i]].rend());
      } else {
        way_points.insert(way_points.end(),
                          cells_sweeps[cell_idx_path[i]].begin(),
                          cells_sweeps[cell_idx_path[i]].end());
      }
      // now cleaned
      cell_graph[cell_idx_path[i]].isCleaned = true;
      // update current point
      point = way_points.back();
      // find shortest path to next cell
      if ((i + 1) < cell_idx_path.size()) {
        next_candidates =
            cell_intersections[cell_idx_path[i]][cell_idx_path[i + 1]];
        if (doReverseNextSweep(point, cells_sweeps[cell_idx_path[i + 1]])) {
          next_point =
              findNextGoal(point, cells_sweeps[cell_idx_path[i + 1]].back(),
                           next_candidates);
          shortest_path =
              getShortestPath(bcd_cells[cell_idx_path[i]], point, next_point);
          way_points.insert(way_points.end(), std::next(shortest_path.begin()),
                            std::prev(shortest_path.end()));
          shortest_path =
              getShortestPath(bcd_cells[cell_idx_path[i + 1]], next_point,
                              cells_sweeps[cell_idx_path[i + 1]].back());
        } else {
          next_point =
              findNextGoal(point, cells_sweeps[cell_idx_path[i + 1]].front(),
                           next_candidates);
          shortest_path =
              getShortestPath(bcd_cells[cell_idx_path[i]], point, next_point);
          way_points.insert(way_points.end(), std::next(shortest_path.begin()),
                            std::prev(shortest_path.end()));
          shortest_path =
              getShortestPath(bcd_cells[cell_idx_path[i + 1]], next_point,
                              cells_sweeps[cell_idx_path[i + 1]].front());
        }
        way_points.insert(way_points.end(), shortest_path.begin(),
                          std::prev(shortest_path.end()));
        point = way_points.back();
      }
    } else {
      shortest_path = getShortestPath(bcd_cells[cell_idx_path[i]],
                                      cells_sweeps[cell_idx_path[i]].front(),
                                      cells_sweeps[cell_idx_path[i]].back());
      if (doReverseNextSweep(point, cells_sweeps[cell_idx_path[i]])) {
        way_points.insert(way_points.end(), shortest_path.rbegin(),
                          shortest_path.rend());
      } else {
        way_points.insert(way_points.end(), shortest_path.begin(),
                          shortest_path.end());
      }
      point = way_points.back();

      if ((i + 1) < cell_idx_path.size()) {
        next_candidates =
            cell_intersections[cell_idx_path[i]][cell_idx_path[i + 1]];
        if (doReverseNextSweep(point, cells_sweeps[cell_idx_path[i + 1]])) {
          next_point =
              findNextGoal(point, cells_sweeps[cell_idx_path[i + 1]].back(),
                           next_candidates);
          shortest_path =
              getShortestPath(bcd_cells[cell_idx_path[i]], point, next_point);
          way_points.insert(way_points.end(), std::next(shortest_path.begin()),
                            std::prev(shortest_path.end()));
          shortest_path =
              getShortestPath(bcd_cells[cell_idx_path[i + 1]], next_point,
                              cells_sweeps[cell_idx_path[i + 1]].back());
        } else {
          next_point =
              findNextGoal(point, cells_sweeps[cell_idx_path[i + 1]].front(),
                           next_candidates);
          shortest_path =
              getShortestPath(bcd_cells[cell_idx_path[i]], point, next_point);
          way_points.insert(way_points.end(), std::next(shortest_path.begin()),
                            std::prev(shortest_path.end()));
          shortest_path =
              getShortestPath(bcd_cells[cell_idx_path[i + 1]], next_point,
                              cells_sweeps[cell_idx_path[i + 1]].front());
        }
        way_points.insert(way_points.end(), shortest_path.begin(),
                          std::prev(shortest_path.end()));
        point = way_points.back();
      }
    }
  }

  cv::Point p1, p2;
  //cv::namedWindow("cover", cv::WINDOW_NORMAL);
  //cv::imshow("cover", original_img);
  //cv::waitKey();

  // Open waypoint file to write coordinates
  std::ofstream out(WAYPOINT_COORDINATE_FILE_PATH);
  std::ostringstream out_oss;

for (size_t i = 1; i < way_points.size(); ++i) {
    cv::Point p1 = cv::Point(std::round(CGAL::to_double(way_points[i - 1].x())),
                                 std::round(CGAL::to_double(way_points[i - 1].y())));
    cv::Point p2 = cv::Point(std::round(CGAL::to_double(way_points[i].x())),
                                 std::round(CGAL::to_double(way_points[i].y())));

    // Ensure p1 and p2 are within valid bounds
    if (std::isnan(p1.x) || std::isnan(p1.y) || std::isnan(p2.x) || std::isnan(p2.y)) {
        std::cerr << "Invalid points detected: p1(" << p1.x << ", " << p1.y 
                  << ") p2(" << p2.x << ", " << p2.y << ")" << std::endl;
        continue;  // Skip this iteration if invalid points are found
    }

    std::vector<cv::Point> newPoints;

    // get number of subdivisions
    if (subdivision_dist > 0) {
      double euclidean_dist = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
      // std::cout << "euclidean_dist: " << euclidean_dist << std::endl;
      double number_of_subdivisions = std::round(euclidean_dist / subdivision_dist);
      // std::cout << "# of subdivisions: " << number_of_subdivisions << std::endl;


      if (number_of_subdivisions > 0) {
        // Compute the step increments based on the number of subdivisions
        double stepX = (p2.x - p1.x) / static_cast<double>(number_of_subdivisions + 1);
        double stepY = (p2.y - p1.y) / static_cast<double>(number_of_subdivisions + 1);

        // Add intermediate points
        for (int i = 1; i <= number_of_subdivisions; ++i) {
            cv::Point intermediatePoint;
            intermediatePoint.x = std::round(p1.x + stepX * i);
            intermediatePoint.y = std::round(p1.y + stepY * i);
            newPoints.push_back(intermediatePoint);
        }

        // Draw the initial line segment from p1 to the first interpolated point
        cv::line(original_img, p1, newPoints[0], cv::Scalar(0, 64, 255));
        for (size_t j = 0; j < newPoints.size() - 1; ++j) {
            cv::line(original_img, newPoints[j], newPoints[j + 1], cv::Scalar(0, 64, 255));  // Draw between subdivided points
        }
        cv::line(original_img, newPoints.back(), p2, cv::Scalar(0, 64, 255));  // Draw final segment to p2
      } else {
        // If subdivisions == 0, directly draw the line between p1 and p2
          cv::line(original_img, p1, p2, cv::Scalar(0, 64, 255));
      }
    }

    //cv::namedWindow("cover", cv::WINDOW_NORMAL);
    //cv::imshow("cover", original_img);
    //        cv::waitKey(50);
    cv::line(original_img, p1, p2, cv::Scalar(200, 200, 200));

    //Robot Waypoints for Navigation
    cv::Size sz = original_img.size();
    int imgHeight = sz.height;
    int y_center = sz.height / 2;
    // Write waypoints to a file (to be fed as coordinates for robot) (not used as of 2/2/202)
    out_oss << "[";
    if (i == 1) {
        out << p1.x << " " << (2* y_center - p1.y) << std::endl;
        out_oss << p1.x << " " << p1.y << std::endl;

    }
    for (const auto& point : newPoints) {
        out << point.x << " " << (2*y_center - point.y) << std::endl;
        out_oss << point.x << " " << point.y << std::endl; 
    }

    // For all other points we will just use p2,
    // we do not pass both p1 and p2 as it would duplicate the points
    out << p2.x << " " << (2*y_center-p2.y) << std::endl;
    out_oss << p2.x << " " << p2.y << std::endl;
    out_oss << "],";
  }
  out.close();

  std::string waypointData = out_oss.str();

  publish_count = 0;

  while(ros::ok() && publish_count < 2)
  {
    std_msgs::String msg;
    msg.data = waypointData;
    point_pub.publish(msg);

    publish_count++;
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  // std::string result_image_path = package_path + "/result/image_result.png";
  // std::string GUI_result_path = GUI_package_path + "/web/ros-frontend/public/temp_zone/image_" + mapName + ".png";
  
  // cv::imwrite(GUI_result_path, original_img);
  // std::cout << "Result image saved to: " << GUI_result_path << std::endl;
  ros::shutdown();
  // cv::waitKey();
#else
  cv::Point p1, p2;
  cv::namedWindow("cover", cv::WINDOW_NORMAL);
  cv::imshow("cover", img);
  cv::waitKey();

  Point_2 point = start;
  way_points.emplace_back(point);

  for (auto &idx : cell_idx_path) {
    if (!cell_graph[idx].isCleaned) {
      if (doReverseNextSweep(point, cells_sweeps[idx])) {
        way_points.insert(way_points.end(), cells_sweeps[idx].rbegin(),
                          cells_sweeps[idx].rend());

        temp_img = img.clone();
        cv::line(
            img,
            cv::Point(CGAL::to_double(point.x()), CGAL::to_double(point.y())),
            cv::Point(CGAL::to_double(cells_sweeps[idx].back().x()),
                      CGAL::to_double(cells_sweeps[idx].back().y())),
            cv::Scalar(255, 0, 0), 1);
        cv::namedWindow("cover", cv::WINDOW_NORMAL);
        cv::imshow("cover", img);
        //                cv::waitKey(500);
        img = temp_img.clone();

        for (size_t i = (cells_sweeps[idx].size() - 1); i > 0; --i) {
          p1 = cv::Point(CGAL::to_double(cells_sweeps[idx][i].x()),
                         CGAL::to_double(cells_sweeps[idx][i].y()));
          p2 = cv::Point(CGAL::to_double(cells_sweeps[idx][i - 1].x()),
                         CGAL::to_double(cells_sweeps[idx][i - 1].y()));
          cv::line(img, p1, p2, cv::Scalar(0, 64, 255));
          cv::namedWindow("cover2", cv::WINDOW_NORMAL);
          cv::imshow("cover2", img);
          //                    cv::waitKey(50);
          cv::line(img, p1, p2, cv::Scalar(200, 200, 200));
        }

      } else {
        way_points.insert(way_points.end(), cells_sweeps[idx].begin(),
                          cells_sweeps[idx].end());

        temp_img = img.clone();
        cv::line(
            img,
            cv::Point(CGAL::to_double(point.x()), CGAL::to_double(point.y())),
            cv::Point(CGAL::to_double(cells_sweeps[idx].front().x()),
                      CGAL::to_double(cells_sweeps[idx].front().y())),
            cv::Scalar(255, 0, 0), 1);
        cv::namedWindow("cover3", cv::WINDOW_NORMAL);
        cv::imshow("cover3", img);
        //                cv::waitKey(500);
        img = temp_img.clone();

        for (size_t i = 1; i < cells_sweeps[idx].size(); ++i) {
          p1 = cv::Point(CGAL::to_double(cells_sweeps[idx][i - 1].x()),
                         CGAL::to_double(cells_sweeps[idx][i - 1].y()));
          p2 = cv::Point(CGAL::to_double(cells_sweeps[idx][i].x()),
                         CGAL::to_double(cells_sweeps[idx][i].y()));
          cv::line(img, p1, p2, cv::Scalar(0, 64, 255));
          cv::namedWindow("cover4", cv::WINDOW_NORMAL);
          cv::imshow("cover4", img);
          //                    cv::waitKey(50);
          cv::line(img, p1, p2, cv::Scalar(200, 200, 200));
        }
      }

      cell_graph[idx].isCleaned = true;
      point = way_points.back();
    }
  }
  //cv::destroyWindow("cover");
  //cv::waitKey(1000);
  

#endif
  //ros::spin();
  return 0;
}
