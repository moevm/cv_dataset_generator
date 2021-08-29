#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

void delete_markers() {
    std::cout << "Deleting visualization" << std::endl;

    ignition::transport::Node node;
    ignition::msgs::Marker delete_marker;

    delete_marker.set_action(ignition::msgs::Marker::DELETE_ALL);
    node.Request("/marker", delete_marker);
}

std::vector<ignition::math::Pose3d> read_trajectory(std::string const& filename) {
    std::ifstream fin(filename);

    ignition::math::Pose3d point;
    std::vector<ignition::math::Pose3d> trajectory;
    while (fin >> point) {
        trajectory.push_back(point);
    }

    return trajectory;
}

void draw_trajectory(std::string const& filename) {
    auto trajectory = read_trajectory(filename);
    std::cout << "Visualizing trajectory of " << trajectory.size() << " points"
              << std::endl;

    ignition::transport::Node node;
    ignition::msgs::Marker line_marker;
    ignition::msgs::Marker point_marker;
    int id = 1;

    point_marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    point_marker.set_type(ignition::msgs::Marker::SPHERE);
    point_marker.mutable_material()->mutable_script()->set_name("Gazebo/Red");
    ignition::msgs::Set(point_marker.mutable_scale(),
                        ignition::math::Vector3d(0.05, 0.05, 0.05));

    for (auto& point : trajectory) {
        point_marker.set_id(id++);
        ignition::msgs::Set(point_marker.mutable_pose(), point);
        node.Request("/marker", point_marker);
    }

    line_marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    line_marker.set_type(ignition::msgs::Marker::LINE_LIST);
    line_marker.mutable_material()->mutable_script()->set_name("Gazebo/Orange");
    line_marker.set_id(id++);

    // TODO: добавить номера точек
    for (size_t i = 1; i < trajectory.size(); ++i) {
        ignition::msgs::Set(line_marker.add_point(), trajectory[i - 1].Pos());
        ignition::msgs::Set(line_marker.add_point(), trajectory[i].Pos());
    }

    node.Request("/marker", line_marker);
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: trajectory file or -d [--delete]" << std::endl;
        return 1;
    }
    using namespace std::string_literals;
    if (argv[1] == "-d"s || argv[1] == "--delete"s) {
        delete_markers();
    } else {
        draw_trajectory(argv[1]);
    }
}
