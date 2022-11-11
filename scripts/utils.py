#!/usr/bin/env python

#
# Author : Mario Fiorino
#

import time
import os
import sys


import control_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryAction, JointTrajectoryGoal # Controller messages
from std_msgs.msg import Float64 # 64-bit floating point numbers
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


from sensor_msgs.msg import JointState
from math import pi

import rospy, sys, numpy as np

from std_msgs.msg import Header

from gazebo_msgs.srv import SpawnModel, DeleteModel

import tf.transformations as tft

from gazebo_msgs.srv import GetModelState, GetWorldProperties, SetModelState, GetModelProperties
from gazebo_msgs.msg import ModelState


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


### Set and get the state of the led of an object
def set_led_state(object_name, new_state):
    """Sets the state of the led of an object, given the name of the object.
    The world should contain a led with name {object_name}_led_{"red"/"green"}

    :param object_name: string containing the name of an object that has a led
    :param state: "red" or "green" -> the new state of the led
    """
    assert new_state in ["red", "green"], "'new_state' should be either 'red' or 'green'"

    curr_led_state = get_led_state(object_name)
    if curr_led_state is None or curr_led_state == new_state:
        return

    model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    led_name = object_name + "_led_" + curr_led_state
    
    if new_state == 'red':
        led_coordinates = model_state(led_name, "")
        pose = led_coordinates.pose
        sdf = os.getenv("PGLAB_HOME")+'/pg_lab_gazebo/models/red_led.sdf'
        name = object_name + "_led_red"
        spawn_object(sdf, name, pose)

    else:
        delete_object(led_name)


def get_led_state(object_name):
    """Returns the state of the led of an object, given the name of the object.
    The world should contain a led with name {object_name}_led_{"red"/"green"}

    :param object_name: string containing the name of an object that has a led
    :return: string containing the state of the led (either "red" or "green")
             if the led model is not found it returns None
    """
    world_objects = get_world_properties().model_names
    if object_name+"_led_red" in world_objects:
        return "red"
    elif object_name+"_led_green" in world_objects:
        return "green"
    else:
        print('No led found for the model %s' % object_name)
        return None


def list_object_names():
    return get_world_properties().model_names

def get_object_pose(id):
    return get_model_state(id).pose

############# Work in progress to trigger the change of the color #############
def detect_collision(name):
    model_state = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
    print(model_state(name))
    
