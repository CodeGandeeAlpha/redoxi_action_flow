#pragma once

#include <PassengerFlow/PassengerFlow.h>
#include <boost/thread/synchronized_value.hpp>
#include <fstream>
#include <memory>
#include <psg_count/psg_count.hpp>
#include <rcpputils/asserts.hpp>
#include <thread>
#include <vineyard/client/client.h>
#include <vineyard/client/ds/object_meta.h>


namespace FlowRos2Pipeline
{
class RegionInfo
{
  public:
    virtual ~RegionInfo()
    {
    }
    std::string m_name;
    PassengerFlow::RegionType m_region_type;
};
using RegionInfoPtr = std::shared_ptr<RegionInfo>;

class DoorInOutRegionInfo : public RegionInfo
{
  public:
    virtual ~DoorInOutRegionInfo()
    {
    }
    std::vector<PassengerFlow::POINT> m_door_line_pixel_points;
    PassengerFlow::POINT m_door_in_pixel_point; // pixel point in door
    double m_certain_region_size = 0.7;         // m
    double m_likely_region_size = 0.3;          // m
};

class DisappearRegionInfo : public RegionInfo
{
  public:
    virtual ~DisappearRegionInfo()
    {
    }
    std::vector<PassengerFlow::POINT> m_door_line_pixel_points;
    PassengerFlow::POINT m_door_in_pixel_point;
    double m_region_size = 0.3;
};

class PassingRegionInfo : public RegionInfo
{
  public:
    virtual ~PassingRegionInfo()
    {
    }
    std::vector<PassengerFlow::POINT> m_region_pixel_points;
};

struct SceneParameter {
    double m_camera_fx, m_camera_fy, m_camera_ux, m_camera_uy;
    PassengerFlow::MATRIX_4d m_camera_extrinsic; // Tcw
    int m_image_width;
    int m_image_height;
    PassengerFlow::MATRIX_4d m_ground_to_world; // Twg
    std::string m_video_path;
    std::vector<RegionInfoPtr> m_regions;
};

class PSGCountImpl
{
  public:
    virtual ~PSGCountImpl()
    {
    }
    PSGCountImpl(PSGCount *node)
        : logger(node->get_logger())
    {
    }
    rclcpp::Logger logger;

    boost::synchronized_value<PSGCount::Map_Document_Waiting *> sync_document_waiting_map;
    boost::synchronized_value<PSGCount::Map_Document_Doing *> sync_document_doing_map;
    boost::synchronized_value<PSGCount::Map_Documents *> sync_documents_map;

    SceneParameter parse_config_file(const std::string &config_file_path)
    {
        std::ifstream fr;
        fr.open(config_file_path);
        SceneParameter output;

        rcpputils::assert_true(fr.is_open(), "parse_config_file(): failed to open config file");
        if (!fr.is_open())
            return output;
        else {
            nlohmann::json config;
            fr >> config;

            output.m_camera_fx = config["camera_fx"];
            output.m_camera_fy = config["camera_fy"];
            output.m_camera_ux = config["camera_ux"];
            output.m_camera_uy = config["camera_uy"];
            output.m_video_path = config["video_path"].get<std::string>();
            output.m_image_width = config["image_width"];
            output.m_image_height = config["image_height"];

            std::vector<double> Twc_vector = config["camera_extrinsic_inv"];
            std::vector<double> Twg_vector = config["ground_to_world"];
            PassengerFlow::MATRIX_4d Twc;
            Twc << Twc_vector[0], Twc_vector[1], Twc_vector[2], Twc_vector[3],
                Twc_vector[4], Twc_vector[5], Twc_vector[6], Twc_vector[7],
                Twc_vector[8], Twc_vector[9], Twc_vector[10], Twc_vector[11],
                Twc_vector[12], Twc_vector[13], Twc_vector[14], Twc_vector[15];
            output.m_camera_extrinsic = Twc.inverse();
            output.m_ground_to_world << Twg_vector[0], Twg_vector[1], Twg_vector[2], Twg_vector[3],
                Twg_vector[4], Twg_vector[5], Twg_vector[6], Twg_vector[7],
                Twg_vector[8], Twg_vector[9], Twg_vector[10], Twg_vector[11],
                Twg_vector[12], Twg_vector[13], Twg_vector[14], Twg_vector[15];
            int region_size = config["region_infos"].size();
            for (int i = 0; i < region_size; ++i) {
                auto region_info = config["region_infos"][i];
                if (region_info["region_type"] == PassengerFlow::RegionTypes::DoorInOut) {
                    auto region = std::make_shared<DoorInOutRegionInfo>();
                    region->m_name = region_info["region_name"].get<std::string>();
                    region->m_region_type = region_info["region_type"];
                    std::vector<int> region_points = region_info["points"];
                    PassengerFlow::POINT door_point1(region_points[0], region_points[1]);
                    PassengerFlow::POINT door_point2(region_points[2], region_points[3]);
                    PassengerFlow::POINT door_in_point(region_points[4], region_points[5]);
                    region->m_door_line_pixel_points.push_back(door_point1);
                    region->m_door_line_pixel_points.push_back(door_point2);
                    region->m_door_in_pixel_point = door_in_point;
                    region->m_certain_region_size = region_info["certain_region_size"];
                    region->m_likely_region_size = region_info["likely_region_size"];
                    output.m_regions.push_back(region);
                } else if (region_info["region_type"] == PassengerFlow::RegionTypes::DoorDisappear) {
                    auto region = std::make_shared<DisappearRegionInfo>();
                    region->m_name = region_info["region_name"].get<std::string>();
                    region->m_region_type = region_info["region_type"];
                    std::vector<int> region_points = region_info["points"];
                    PassengerFlow::POINT door_point1(region_points[0], region_points[1]);
                    PassengerFlow::POINT door_point2(region_points[2], region_points[3]);
                    PassengerFlow::POINT door_in_point(region_points[4], region_points[5]);
                    region->m_door_line_pixel_points.push_back(door_point1);
                    region->m_door_line_pixel_points.push_back(door_point2);
                    region->m_door_in_pixel_point = door_in_point;
                    region->m_region_size = region_info["region_size"];
                    output.m_regions.push_back(region);
                }
            }
            return output;
        }
    }

    void set_scene(const SceneParameter &scene_parameter, PassengerFlow::ScenePtr &out_scene,
                   PassengerFlow::CameraModelPtr &out_camera, PassengerFlow::GroundPtr &out_ground,
                   std::map<std::string, PassengerFlow::EventZonePtr> &out_event_zones)
    {
        set_camera_model(scene_parameter, out_camera);
        set_ground(scene_parameter, out_ground);
        out_scene->add_camera(out_camera);
        out_scene->add_ground(out_ground);
        for (auto &region_info : scene_parameter.m_regions) {
            auto event_zone = gen_event_zone(region_info, out_camera, out_ground);
            auto event_zone_name = region_info->m_name;
            out_event_zones[event_zone_name] = event_zone;
        }
    }

    void set_camera_model(const SceneParameter &scene_parameter, PassengerFlow::CameraModelPtr &cam)
    {
        PassengerFlow::fMATRIX_3 k;
        k << scene_parameter.m_camera_fx, 0, scene_parameter.m_camera_ux,
            0, scene_parameter.m_camera_fy, scene_parameter.m_camera_uy,
            0, 0, 1; // cam params
        cam->set_projection_matrix(k.transpose());
        cam->set_extrinsic_matrix(scene_parameter.m_camera_extrinsic.transpose());
        cam->set_image_size(scene_parameter.m_image_width, scene_parameter.m_image_height);
    }

    void set_ground(const SceneParameter &scene_parameter, PassengerFlow::GroundPtr &ground)
    {
        auto Tgw = scene_parameter.m_ground_to_world.inverse();
        ground->set_global_coordinate_frame(scene_parameter.m_ground_to_world);
        ground->set_ground_coordinate_frame(Tgw);
    }

    PassengerFlow::POINT get_point_on_ground_from_project_point(const PassengerFlow::CameraModelPtr &cam,
                                                                const PassengerFlow::GroundPtr &ground,
                                                                const PassengerFlow::POINT &point)
    {
        PassengerFlow::fVECTOR_3 point_ray, point_ray_p0;
        PassengerFlow::fVECTOR_2 point_uv_vector(point.x, point.y);
        cam->ray_from_projected_points(point_uv_vector, &point_ray_p0, &point_ray);

        auto ground_p0 = ground->get_p0();
        PassengerFlow::fVECTOR_3 ground_p0_vector{ground_p0.x, ground_p0.y, ground_p0.z};
        PassengerFlow::fVECTOR_3 ground_normal = ground->get_normal();

        auto door_point_in_world = PassengerFlow::get_line_surface_intersection(point_ray_p0, point_ray,
                                                                                ground_p0_vector, ground_normal);
        float temp_x = door_point_in_world(0), temp_y = door_point_in_world(1), temp_z = door_point_in_world(2);
        auto door_point_on_ground = ground->world_to_ground({temp_x, temp_y, temp_z});
        return door_point_on_ground;
    }

    PassengerFlow::EventZonePtr gen_event_zone(const RegionInfoPtr &region_info, const PassengerFlow::CameraModelPtr &cam,
                                               const PassengerFlow::GroundPtr &ground)
    {
        if (region_info->m_region_type == PassengerFlow::RegionTypes::DoorInOut) {
            auto door_in_region_info = RedoxiTrack::dyncast_with_check<DoorInOutRegionInfo>(region_info.get());
            auto event_zone = gen_door_in_out_zone(door_in_region_info, cam, ground);
            return event_zone;
        } else if (region_info->m_region_type == PassengerFlow::RegionTypes::DoorDisappear) {
            auto disappear_region_info = RedoxiTrack::dyncast_with_check<DisappearRegionInfo>(region_info.get());
            auto event_zone = gen_disappear_zone(disappear_region_info, cam, ground);
            return event_zone;
        } else if (region_info->m_region_type == PassengerFlow::RegionTypes::Passing) {
            auto passing_region_info = RedoxiTrack::dyncast_with_check<PassingRegionInfo>(region_info.get());
            auto event_zone = gen_passing_zone(passing_region_info, cam, ground);
            return event_zone;
        } else {
            RedoxiTrack::assert_throw(false, "please set region type");
            return PassengerFlow::EventZonePtr();
        }
    }

    PassengerFlow::EventZonePtr gen_door_in_out_zone(const DoorInOutRegionInfo *region_info, const PassengerFlow::CameraModelPtr &cam,
                                                     const PassengerFlow::GroundPtr &ground)
    {
        auto door_in_out_region = gen_door_in_out_region(region_info->m_door_line_pixel_points, region_info->m_door_in_pixel_point,
                                                         region_info->m_likely_region_size, region_info->m_certain_region_size, cam, ground);
        auto door_in_out_region_name = region_info->m_name + " : door in out region";
        door_in_out_region->set_name(door_in_out_region_name);

        PassengerFlow::DoorInOutEventZonePtr door_in_out_event_zone = std::make_shared<PassengerFlow::DoorInOutEventZone>();
        door_in_out_event_zone->set_region(door_in_out_region);
        door_in_out_event_zone->set_name(region_info->m_name);
        return door_in_out_event_zone;
    }

    PassengerFlow::RegionPtr gen_door_in_out_region(const std::vector<PassengerFlow::POINT> &door_line_points,
                                                    const PassengerFlow::POINT door_in_pixel_point,
                                                    const double buffer_area_length, const double certainly_area_length,
                                                    const PassengerFlow::CameraModelPtr &cam,
                                                    const PassengerFlow::GroundPtr &ground)
    {
        auto door_point1_uv = door_line_points[0];
        auto door_point2_uv = door_line_points[1];
        auto door_point1_on_ground = get_point_on_ground_from_project_point(cam, ground, door_point1_uv);
        auto door_point2_on_ground = get_point_on_ground_from_project_point(cam, ground, door_point2_uv);
        auto door_in_point_on_ground = get_point_on_ground_from_project_point(cam, ground, door_in_pixel_point);
        std::vector<PassengerFlow::POINT> door_points_on_ground{door_point1_on_ground, door_point2_on_ground};
        auto region = std::make_shared<PassengerFlow::DoorInOutRegion>(ground, door_points_on_ground,
                                                                       door_in_point_on_ground, buffer_area_length, certainly_area_length);
        return region;
    }

    PassengerFlow::EventZonePtr gen_disappear_zone(const DisappearRegionInfo *region_info, const PassengerFlow::CameraModelPtr &cam,
                                                   const PassengerFlow::GroundPtr &ground)
    {
        auto door_point1_uv = region_info->m_door_line_pixel_points[0];
        auto door_point2_uv = region_info->m_door_line_pixel_points[1];
        auto door_point1_on_ground = get_point_on_ground_from_project_point(cam, ground, door_point1_uv);
        auto door_point2_on_ground = get_point_on_ground_from_project_point(cam, ground, door_point2_uv);
        auto door_in_point_on_ground = get_point_on_ground_from_project_point(cam, ground, region_info->m_door_in_pixel_point);
        std::vector<PassengerFlow::POINT> door_points_on_ground{door_point1_on_ground, door_point2_on_ground};
        double disappear_size = region_info->m_region_size;
        auto disappear_region = std::make_shared<PassengerFlow::DisappearInOutRegion>(ground, door_points_on_ground, door_in_point_on_ground, disappear_size);

        std::string disappear_region_name = region_info->m_name + " : disappear region";
        disappear_region->set_name(disappear_region_name);

        auto disappear_event_zone = std::make_shared<PassengerFlow::DisappearInOutEventZone>();
        disappear_event_zone->set_region(disappear_region);
        disappear_event_zone->set_name(region_info->m_name);
        return disappear_event_zone;
    }

    PassengerFlow::EventZonePtr gen_passing_zone(const PassingRegionInfo *region_info, const PassengerFlow::CameraModelPtr &cam,
                                                 const PassengerFlow::GroundPtr &ground)
    {

        std::vector<PassengerFlow::POINT> region_points_on_ground;
        for (auto &point_in_pixel : region_info->m_region_pixel_points) {
            auto point_on_ground = get_point_on_ground_from_project_point(cam, ground, point_in_pixel);
            region_points_on_ground.push_back(point_on_ground);
        }

        auto passing_region = std::make_shared<PassengerFlow::PassingInOutRegion>(ground, region_points_on_ground);

        std::string passing_region_name = region_info->m_name + " : disappear region";
        passing_region->set_name(passing_region_name);

        auto passing_in_out_event_zone = std::make_shared<PassengerFlow::PassingInOutEventZone>();
        passing_in_out_event_zone->set_region(passing_region);
        passing_in_out_event_zone->set_name(region_info->m_name);
        return passing_in_out_event_zone;
    }

    std::shared_ptr<vineyard::Client> v6d_client;

    PassengerFlow::ScenePtr scene;
    PassengerFlow::GroundPtr ground;
    PassengerFlow::CameraModelPtr camera;
    std::map<std::string, PassengerFlow::EventZonePtr> event_zones;

    PassengerFlow::SingleGroundSpatialAnalyzer3Ptr spatial_analyzer;
    PassengerFlow::TrajectoryAnalyzerPtr trajectory_analyzer;

    std::shared_ptr<std::thread> step_thread;
    std::shared_ptr<std::thread> process_thread;
    bool step_running = false; // for stopping the step thread

    bool visualize_flag = true;
};
} // namespace FlowRos2Pipeline