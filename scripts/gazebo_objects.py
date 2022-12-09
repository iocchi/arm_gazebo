#!/usr/bin/env python

from gazebo_models import *


if __name__ == '__main__':
    
    if len(sys.argv)==1 or sys.argv[1]=='-h':
            print("Options")
            print("-h\thelp")
            print("-l\tlist objects")
            print("-m\tlist available models")
            print("-a <obj>  <type> <x> <y> <z> <yaw> <pitch> <roll>|<filename>\tadd an object or all objects in config file")
            print("-d <obj>|<filename>\tdelete one object or all objects in config file")
            print("-w\tworld properties")
            print("-s <obj>\tobject properties and state")

    else:
        if sys.argv[1]=='-m':
            print_models()

        elif sys.argv[1]=='-l':
            l = list_object_names()
            for obj in l:
                p = get_object_pose(obj)
                #s = get_model_state(obj)
                print("%s  %s" %(obj,pose_str(p)))

        elif sys.argv[1]=='-w':
            print(get_world_properties())

        elif sys.argv[1]=='-a' and len(sys.argv)>2:
            name = sys.argv[2]
            if os.path.isfile(name):
                add_objects(name)
            elif len(sys.argv)>9:
                model = sys.argv[3]
                pose = [ float(x) for x in sys.argv[4:] ]
                add_object(name, model, pose)

        elif sys.argv[1]=='-d' and len(sys.argv)>2:
            name = sys.argv[2]
            if os.path.isfile(name):
                del_objects(name)
            else:
                delete_object(name)

        elif sys.argv[1]=='-s' and len(sys.argv)>2:
            name = sys.argv[2]
            print("Object %s" %name)
            properties = get_model_properties(name)
            state = get_model_state(name)
            print("---------------------------\nProperties\n---------------------------")
            print(properties)
            print("---------------------------\n")
            print("---------------------------\nState\n---------------------------")
            print(state)
            print("---------------------------\n")


