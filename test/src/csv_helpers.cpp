#include "csv_helpers.h"

#include <algorithm>
#include <ostream>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <ugl/math/vector.h>
#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/extended_pose.h>

#include "accuracy_test.h"

namespace invariant::test
{
namespace
{

constexpr auto delimiter = ' ';

std::ostream& operator<<(std::ostream& os, const Result& result)
{
    os << "time" << delimiter
       << "nees" << delimiter
       << "nis"  << delimiter
       << "pos_nees" << delimiter << "vel_nees" << delimiter << "rot_nees" << delimiter
       << "pos_err_x" << delimiter << "pos_err_y" << delimiter << "pos_err_z" << delimiter
       << "vel_err_x" << delimiter << "vel_err_y" << delimiter << "vel_err_z" << delimiter
       << "rot_err_x" << delimiter << "rot_err_y" << delimiter << "rot_err_z" << delimiter
       << "pos_x" << delimiter << "pos_y" << delimiter << "pos_z" << delimiter
       << "vel_x" << delimiter << "vel_y" << delimiter << "vel_z" << delimiter
       << "rot_x" << delimiter << "rot_y" << delimiter << "rot_z" << delimiter
       << '\n';

    auto write_vector = [&](const ugl::Vector3& vec) {
        os << vec.x() << delimiter << vec.y() << delimiter << vec.z() << delimiter;
    };

    const auto size = result.times.size();
    for (std::size_t i = 0; i < size; ++i)
    {
        os << result.times[i] << delimiter;
        os << result.nees_values[i] << delimiter;
        os << result.nis_values[i] << delimiter;
        os << result.position_nees[i] << delimiter;
        os << result.velocity_nees[i] << delimiter;
        os << result.rotation_nees[i] << delimiter;

        write_vector(result.position_errors[i]);
        write_vector(result.velocity_errors[i]);
        write_vector(result.rotation_errors[i]);

        write_vector(result.estimates[i].position());
        write_vector(result.estimates[i].velocity());
        write_vector(ugl::lie::SO3::log(result.estimates[i].rotation()));

        os << '\n';
    }

    return os;
}

void write_ground_truth(std::ostream& os, const std::vector<double>& times, const std::vector<ugl::lie::ExtendedPose>& states)
{
    os << "time" << delimiter
       << "pos_x" << delimiter << "pos_y" << delimiter << "pos_z" << delimiter
       << "vel_x" << delimiter << "vel_y" << delimiter << "vel_z" << delimiter
       << "rot_x" << delimiter << "rot_y" << delimiter << "rot_z" << delimiter
       << '\n';

    auto write_vector = [&](const ugl::Vector3& vec) {
        os << vec.x() << delimiter << vec.y() << delimiter << vec.z() << delimiter;
    };

    const auto size = times.size();
    for (std::size_t i = 0; i < size; ++i)
    {
        os << times[i] << delimiter;
        write_vector(states[i].position());
        write_vector(states[i].velocity());
        write_vector(ugl::lie::SO3::log(states[i].rotation()));
        os << '\n';
    }
}

} // namespace

void save_to_csv(const std::vector<Result>& results, std::string_view file_name_prefix)
{
    auto test_info = testing::UnitTest::GetInstance()->current_test_info();

    std::stringstream ss;
    ss << "# " << test_info->test_case_name() << '.' << test_info->name() << ": " << test_info->value_param() << '\n';
    ss << '\n';
    ss << "test_case_count" << '\n' << results.size() << '\n';
    ss << "rows_per_case" << '\n' << results[0].times.size() << '\n';
    ss << '\n';

    write_ground_truth(ss, results[0].times, results[0].ground_truth);
    ss << '\n';

    for (const auto& result : results)
    {
        ss << result << '\n';
    }

    std::string file_name = test_info->name();
    std::replace(std::begin(file_name), std::end(file_name), '/', '_');

    std::string subfolder = test_info->test_case_name();
    subfolder.erase(std::find(std::begin(subfolder), std::end(subfolder), '/'), std::end(subfolder));

    std::stringstream file_path;
    file_path << "/home/vk/mav_ws/src/invariant_kalman_filter/test/results/data/" << subfolder << '/' << file_name_prefix << file_name << ".csv";

    std::ofstream csv_file{file_path.str()};
    csv_file << ss.rdbuf();
}

} // namespace invariant::test
