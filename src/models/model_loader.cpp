#include "ed/models/model_loader.h"

#include <ros/console.h>

#include "ed/update_request.h"
#include "ed/entity.h"
#include "ed/relations/transform_cache.h"

#include <tue/filesystem/path.h>

#include "shape_loader.h"

#include <tue/config/reader.h>
#include <tue/config/writer.h>
#include <tue/config/configuration.h>

#include <sstream>
#include <iostream>

namespace ed
{

namespace models
{

// ----------------------------------------------------------------------------------------------------

ModelLoader::ModelLoader()
{
    const char * mpath = ::getenv("ED_MODEL_PATH");
    if (mpath)
    {
        std::stringstream ss(mpath);
        std::string item;
        while (std::getline(ss, item, ':'))
            model_paths_.push_back(item);
    }
}

// ----------------------------------------------------------------------------------------------------

ModelLoader::~ModelLoader()
{
}

// ----------------------------------------------------------------------------------------------------

std::string ModelLoader::getModelPath(const std::string& type) const
{
    for(std::vector<std::string>::const_iterator it = model_paths_.begin(); it != model_paths_.end(); ++it)
    {
        tue::filesystem::Path model_path(*it + "/" + type);
        if (model_path.exists())
            return model_path.string();
    }

    return "";
}

// ----------------------------------------------------------------------------------------------------

tue::config::DataConstPointer ModelLoader::loadModelData(const std::string& type, std::vector<std::string>& types, std::stringstream& error)
{
    std::map<std::string, ModelData>::iterator it = model_cache_.find(type);
    if (it != model_cache_.end())
    {
        types = it->second.second;
        const tue::config::DataConstPointer& data = it->second.first;
        return data;
    }

    tue::config::DataPointer data;

    std::string model_path = getModelPath(type);
    if (model_path.empty())
    {
        error << "ed::models::create() : Model '" << type << "' could not be found." << std::endl;
        return data;
    }

    tue::filesystem::Path model_cfg_path(model_path + "/model.yaml");
    if (!model_cfg_path.exists())
    {
        error << "ed::models::create() : ERROR loading configuration for model '" << type << "'; '" << model_cfg_path.string() << "' file does not exist." << std::endl;
        return data;
    }

    tue::Configuration model_cfg;
    if (!model_cfg.loadFromYAMLFile(model_cfg_path.string()))
    {
        error << "ed::models::create() : ERROR loading configuration for model '" << type << "'; '" << model_cfg_path.string() << "' failed to parse yaml file." << std::endl;
        return data;
    }

    std::string super_type;
    if (model_cfg.value("type", super_type, tue::OPTIONAL))
    {
        tue::config::DataConstPointer super_data = loadModelData(super_type, types, error);
        tue::config::DataPointer combined_data;
        combined_data.add(super_data);
        combined_data.add(model_cfg.data());

        types.push_back(super_type);

        data = combined_data;
    }
    else
    {
        data = model_cfg.data();
    }

    // If model loads a shape, set model path in shape data
    tue::config::ReaderWriter rw(data);
    if (rw.readGroup("shape"))
    {
        rw.setValue("__model_path__", model_path);
        rw.endGroup();
    }

    // Store data in cache
    model_cache_[type] = ModelData(data, types);

    return data;
}

// ----------------------------------------------------------------------------------------------------

bool ModelLoader::exists(const std::string& type) const
{
    std::map<std::string, ModelData>::const_iterator it = model_cache_.find(type);
    if (it != model_cache_.end())
        return true;

    std::string model_path = getModelPath(type);
    return !model_path.empty();
}

// ----------------------------------------------------------------------------------------------------

bool ModelLoader::create(const UUID& id, const std::string& type, UpdateRequest& req, std::stringstream& error)
{
    std::vector<std::string> types;
    tue::config::DataConstPointer data = loadModelData(type, types, error);
    if (data.empty())
        return false;

    if (!create(data, id, "", req, error))
        return false;

    types.push_back(type);
    for(std::vector<std::string>::const_iterator it = types.begin(); it != types.end(); ++it)
        req.addType(id, *it);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ModelLoader::create(const tue::config::DataConstPointer& data, UpdateRequest& req, std::stringstream& error)
{
    if (!create(data, "", "", req, error, ""))
        return false;

    return true;
}

// ----------------------------------------------------------------------------------------------------


bool ModelLoader::create(const tue::config::DataConstPointer& data, const UUID& id_opt, const UUID& parent_id,
                         UpdateRequest& req, std::stringstream& error, const std::string& model_path,
                         const geo::Pose3D& pose_offset)
{
    tue::config::Reader r(data);

    // Get Id
    UUID id;
    std::string id_str;
    if (r.value("id", id_str, tue::config::OPTIONAL))
    {
        if (parent_id.str().empty() || parent_id.str()[0] == '_')
            id = id_str;
        else
            id = parent_id.str() + "/" + id_str;
    }
    else if (!id_opt.str().empty())
    {
        id = id_opt;
    }
    else
    {
        id = ed::Entity::generateID();
    }

    // Get type. If it exists, first construct an entity based on the given type.
    std::string type;
    if (r.value("type", type, tue::config::OPTIONAL))
    {
        std::vector<std::string> types;
        tue::config::DataConstPointer super_data = loadModelData(type, types, error);

        if (super_data.empty())
            return false;

        tue::config::DataPointer data_combined;
        data_combined.add(super_data);
        data_combined.add(data);

        r = tue::config::Reader(data_combined);

        types.push_back(type);
        for(std::vector<std::string>::const_iterator it = types.begin(); it != types.end(); ++it)
            req.addType(id, *it);
    }

    // Set type
    req.setType(id, type);

    geo::Pose3D pose = geo::Pose3D::identity();

    // Get pose
    if (r.readGroup("pose"))
    {
        readPose(pose, r);
        r.endGroup();
    }

    pose = pose_offset * pose;

    req.setPose(id, pose);

    // Check the composition
    if (r.readArray("composition"))
    {
        while (r.nextArrayItem())
        {
            if (!create(r.data(), "", id, req, error, "", pose))
                return false;
        }

        r.endArray();
    }

    // Set shape
    if (r.readGroup("shape"))
    {
        std::string shape_model_path = model_path;
        r.value("__model_path__", shape_model_path);

        geo::ShapePtr shape = loadShape(shape_model_path, r, shape_cache_, error);
        if (shape)
            req.setShape(id, shape);
        else
            return false;

        r.endGroup();
    }

    if (r.readGroup("state_update"))
    {
        // Read ROI values to filter z values while calling /ed/kinect/state-update
        if (r.readGroup("ROI"))
        {
            float min;
            float max;
            std::string mode = "include";

            r.value("z_min", min);
            r.value("z_max", max);
            r.value("mode", mode ,tue::config::OPTIONAL);

            ed::ROIConstPtr roiPtr = ed::ROIConstPtr(new ed::ROI(min, max, mode == "include" ? true : false));
            req.setROI(id, roiPtr);
            std::cout << id << " ROI min:" << min << " max:" << max << " mode:" << mode << std::endl;
            r.endGroup();
        }
        // Read movement freedoms/restrictions to align to main group object while calling /ed/kinect/state-update
        if (r.readGroup("degrees_of_freedom"))
        {

            bool canRotate = false;
            bool canMove = false;
            std::string cur_str = "";
            r.value("rotation", cur_str,  tue::config::OPTIONAL);
            if(cur_str == "true" || cur_str == "" )
            {
              canRotate = true;
            }
            cur_str = "";
            r.value("translation", cur_str,  tue::config::OPTIONAL);
            if(cur_str == "true" || cur_str == "" )
            {
              canMove = true;
            }

            float x = 0;
            float y = 0;
            if (r.readGroup("translation_restriction"))
            {
                r.value("x", x);
                r.value("y", y);

                if( x == 0 and y == 0)
                {
                  if(canMove == true)
                  {
                    ROS_WARN("inside the definition of: %s , translation is allowed but translation_restriction set to fixed position, possible error inside Yaml" ,id.c_str());
                    canMove = false;
                  }
                }
                else
                {
                  if(canMove == false)
                    ROS_WARN("inside the definition of: %s , translation is forbidden but translation_restriction set direction Vector, possible error inside Yaml" ,id.c_str());

                }
                r.endGroup();
            }


            ed::MoveRestrictionsConstPtr moveRestrictionsPtr =
                ed::MoveRestrictionsConstPtr(new ed::MoveRestrictions(canRotate, canMove, x, y));
            req.setMoveRestrictions(id, moveRestrictionsPtr);
            std::cout << id << " freedoms x:" << x << " y:" << y  << " rotate:" << canRotate << " move:" << canMove << std::endl;
            r.endGroup();
        }

        // Read states for get-state request
        if (r.readGroup("state_definitions"))
        {
            float close;
            float open;
            std::string mode;

            r.value("close", close);
            r.value("open", open);
            r.value("mode", mode);

            bool angle = mode == "angle" ? true : false;
            bool position = !angle;

            ed::StateDefinitionConstPtr stateDefinitionPtr =
                ed::StateDefinitionConstPtr(new ed::StateDefinition(angle, position, close, open, close, open));
            req.setStateDefinition(id, stateDefinitionPtr);

            std::cout << id << " state_definitions close:" << close << " open:" << open << " mode:" << mode << std::endl;
            r.endGroup();
        }
        r.endGroup();
    }


    std::string stateUpdateGroup;
    // if state-update-group, store the group and prepare the entity for /ed/kinect/state-update which also needs the original position
    if (r.value("state-update-group", stateUpdateGroup, tue::config::OPTIONAL))
    {
        req.setStateUpdateGroup(id, stateUpdateGroup);
        // if original-pose inside YAML given read it, otherwise use pose value
        geo::Pose3D origPose;
        if (r.readGroup("original-pose"))
        {
            origPose = geo::Pose3D::identity();
            readPose(origPose, r);
            // move orig pose with the world
            origPose = pose_offset * origPose;
            r.endGroup();
        }
        else
        {
            origPose = geo::Pose3D(pose.R, pose.t);
        }
        req.setOriginalPose(id, origPose);
    }

    if (r.readArray("flags"))
    {
        while (r.nextArrayItem())
        {
            std::string flag;
            if (r.value("flag", flag))
                req.setFlag(id, flag);
        }
        r.endArray();
    }

    // Add additional data
    req.addData(id, r.data());

    return true;
}


void ModelLoader::readPose(geo::Pose3D& pose, tue::config::Reader& r)
{
    r.value("x", pose.t.x);
    r.value("y", pose.t.y);
    r.value("z", pose.t.z);

    double roll = 0, pitch = 0, yaw = 0;
    r.value("X", roll,  tue::config::OPTIONAL);
    r.value("Y", pitch, tue::config::OPTIONAL);
    r.value("Z", yaw,   tue::config::OPTIONAL);
    r.value("roll",  roll,  tue::config::OPTIONAL);
    r.value("pitch", pitch, tue::config::OPTIONAL);
    r.value("yaw",   yaw,   tue::config::OPTIONAL);

    // Set rotation
    pose.R.setRPY(roll, pitch, yaw);
}

} // end namespace models

} // end namespace ed
