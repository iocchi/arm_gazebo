#!/usr/bin/env python

# adapted from https://boschresearch.github.io/pcg_gazebo_pkgs/python_api/pcg_gazebo.simulation/

from __future__ import print_function

import time
import os
import sys

import rospy

import trajectory_msgs.msg
import geometry_msgs.msg

import tf.transformations as tft

from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState, GetWorldProperties, SetModelState, GetModelProperties
from gazebo_msgs.msg import ModelState



def get_gazebo_model_folders(dir_path):
    """Return the paths to all Gazebo model folders under the 
    directory `dir_path`.
    
    > *Input arguments*
    
    * `dir_path` (*type:* `str`): Path to the search directory.
    
    > *Returns*
    
    `dict`: Gazebo model paths ordered according to the 
    Gazebo model names.
    """
    import os
    assert os.path.isdir(dir_path), \
        'Invalid directory path, path={}'.format(dir_path)

    models_paths = dict()
    for item in os.listdir(dir_path):
        if os.path.isdir(os.path.join(dir_path, item)):
            subfolder_items = os.listdir(os.path.join(dir_path, item))
            has_config = False
            has_sdf = False
            sdf_files = list()

            for subitem in os.listdir(os.path.join(dir_path, item)):
                if os.path.isfile(os.path.join(dir_path, item, subitem)):
                    if '.config' in subitem:
                        has_config = True
                    if '.sdf' in subitem:
                        has_sdf = True
                        sdf_files.append(subitem)
            
            if has_config and has_sdf:
                models_paths[item] = dict(
                    path=os.path.join(dir_path, item), 
                    sdf=sdf_files)
            else:
                models_paths.update(get_gazebo_model_folders(os.path.join(dir_path, item)))        
    return models_paths


def get_gazebo_models():
    """Search for Gazebo models in the local `.gazebo/models` folder
    and in the ROS paths.
    > *Returns*
    `dict`: Information of all Gazebo models found
    """
    import rospkg
    import os

    finder = rospkg.RosPack()

    global GAZEBO_MODELS
    GAZEBO_MODELS = dict()

    # Load all models from /usr/share
    folders = os.listdir('/usr/share')
    gazebo_folder = None
    for folder in os.listdir('/usr/share'):
        if 'gazebo-' in folder:
            gazebo_folder = os.path.join('/usr', 'share', folder, 'models')
            break
    if gazebo_folder is not None:
        if os.path.isdir(gazebo_folder):
            GAZEBO_MODELS.update(get_gazebo_model_folders(gazebo_folder))

    # Load all models from ~/.gazebo/models
    home_folder = os.path.expanduser('~')
    gazebo_folder = os.path.join(home_folder, '.gazebo', 'models')
    if os.path.isdir(gazebo_folder):
        GAZEBO_MODELS.update(get_gazebo_model_folders(gazebo_folder))

    # Load all models from catkin packages
    for ros_pkg in finder.list():        
        ros_path = finder.get_path(ros_pkg)
        for folder in os.listdir(ros_path):
            if not os.path.isdir(os.path.join(ros_path, folder)):
                continue
            models = get_gazebo_model_folders(os.path.join(ros_path, folder))
            for tag in models:
                models[tag]['ros_pkg'] = ros_pkg
            GAZEBO_MODELS.update(models)
            
    return GAZEBO_MODELS


def print_models():
    models = get_gazebo_models()
    for tag in sorted(models):
        print("%s\t%s" %(tag, models[tag]['path']))




# global services
spawn_model = None
delete_model = None


def coords2pose(coords):
    """Creates a Pose object 'geometry_msgs.msg.Pose()' with the given coordinates in the format [x, y, z, roll, pitch, yaw]

    :param coords: Coordinates of the object in the format [x, y, z, roll, pitch, yaw]
    :return: Pose object
    """
    object_pose = geometry_msgs.msg.Pose()
    quaternion = tft.quaternion_from_euler(coords[3], coords[4], coords[5])
    object_pose.position.x = float(coords[0])
    object_pose.position.y = float(coords[1])
    object_pose.position.z = float(coords[2])
    object_pose.orientation.x = quaternion[0]
    object_pose.orientation.y = quaternion[1]
    object_pose.orientation.z = quaternion[2]
    object_pose.orientation.w = quaternion[3]
    return object_pose


def pose_str(pose):
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    a = tft.euler_from_quaternion(quat)
    return '%.4f %.4f %.4f  %.2f  %.2f  %.2f' \
        %(pose.position.x,pose.position.y,pose.position.z,a[0],a[1],a[2])


def spawn_object(path, name, pose, reference_frame = 'world', namespace = ''):
    """Spawn an sdf model in the world, with the specified pose and name. The name assigned to the object  already exist. The path indicates the sdf model to spawn.

    :param path: Path to the sdf model to spawn
    :param name: Name to assign to the object. Must not be already assigned to a model in the world.
    :param pose: Pose of the object. The type must be 'geometry_msgs.msg.Pose()'
    :param reference_frame:
    :param namespace:
    """

    global spawn_model

    world_objects = get_world_properties().model_names
    if name in world_objects:
        print('Error: object %s already exists!' %name)
        #sys.exit()
    else:
        print("Spawning object %s" %name)
        if spawn_model == None:
            rospy.wait_for_service("gazebo/spawn_sdf_model")
            #print("Service ready")
            spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        
        with open(path, "r") as model_file:
            description_xml = model_file.read().replace('\n', '')
            spawn_model(name, description_xml, namespace, pose, reference_frame)

        #print('Spawning complete')


def delete_object(name):
    """Removes the model with the specified 'name' from the world.

    :param path: Name of the model to remove from the world.
    """
    global delete_model
    if delete_model == None:
        rospy.wait_for_service("gazebo/delete_model")
        #print("Service ready")
        delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    print("Deleting object %s" %name)
    delete_model(name)


    #print('Deleting complete')


def get_world_properties():
    rospy.wait_for_service("gazebo/get_world_properties")
    #print("Service ready")
    get_properties = rospy.ServiceProxy("gazebo/get_world_properties", GetWorldProperties)
    return get_properties()


def get_model_state(name, ref_frame=""):
    rospy.wait_for_service("gazebo/get_model_state")
    #print("Service ready")
    state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    return state(name, ref_frame)

def get_model_properties(name):
    rospy.wait_for_service("gazebo/get_model_properties")
    #print("Service ready")
    properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
    return properties(name)

def move_object(name, coords):
    """Moves the object with the corresponding name to the given coordinates in the format [x, y, z, roll, pitch, yaw].

    :param name: Name of the model to move.
    :param coords: Desired coordinates of the object in the format [x, y, z, roll, pitch, yaw].
    """
    print('Moving object', name)
    world_objects = get_world_properties().model_names
    if name not in world_objects:
        print('Error, the object you are trying to move doesnt exists')
        sys.exit()

    model_state = ModelState()
    quaternion = tft.quaternion_from_euler(coords[3], coords[4], coords[5])
    model_state.model_name = name
    model_state.pose.position.x = float(coords[0])
    model_state.pose.position.y = float(coords[1])
    model_state.pose.position.z = float(coords[2])
    model_state.pose.orientation.x = quaternion[0]
    model_state.pose.orientation.y = quaternion[1]
    model_state.pose.orientation.z = quaternion[2]
    model_state.pose.orientation.w = quaternion[3]
    
    # rospy.wait_for_service('/gazebo/set_model_state')
    # print('Service ready')

    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    set_state(model_state)

    print('Moving object complete')


def list_object_names():
    return get_world_properties().model_names

def get_object_pose(id):
    return get_model_state(id).pose



def add_object(name, model, pose):
    d = get_gazebo_models()
    if model in d.keys():
        sdf = os.path.join(d[model]['path'], d[model]['sdf'][0])        
        object_pose = coords2pose(pose)
        spawn_object(sdf, name, object_pose)
    else:
        print("Model %s unknown" %model)

def add_objects(infile):
    with open(infile,'r') as f:
        l = f.readline()
        while l!='' and l[0:4] != '#END':
            l = l.strip()
            if len(l)>1 and l[0]!='#':
                v = l.split()
                if len(v)>7:
                    try:
                        name = v[0]
                        model = v[1]
                        pose = [ float(x) for x in v[2:] ]
                        add_object(name, model, pose)
                        time.sleep(0.2)
                    except Exception as e:
                        print("%s%s\n" %(l,e))
                else:
                    print("Parse error: %s" %l)
            l = f.readline()
        f.close()


# name = 'abc*'
# delete all objects starting with abc
def delete_all_objects_like(name):
    l = list_object_names()
    for obj in l:
        if obj.startswith(name[:-1]):
            delete_object(obj)

def del_objects(infile):
    with open(infile,'r') as f:
        l = f.readline()
        while l!=''  and l[0:4] != '#END':
            l = l.strip()
            if len(l)>1 and l[0]!='#':
                v = l.split()
                name = v[0]
                if '*' in name:
                    delete_all_objects_like(name)
                else:
                    delete_object(name)
            l = f.readline()
        f.close()



