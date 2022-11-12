#!/usr/bin/env python

# adapted from https://boschresearch.github.io/pcg_gazebo_pkgs/python_api/pcg_gazebo.simulation/

from __future__ import print_function


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

if __name__ == '__main__':
    models = get_gazebo_models()
    for tag in sorted(models):
        print('{}: {}/{}'.format(tag, models[tag]['path'], models[tag]['sdf'][0]))

