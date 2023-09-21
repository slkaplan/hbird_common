import yaml
from planner_utils import *
from path_planner import PathPlanner


def main():
    """Main function that runs the simulation"""
    
    # get configuration file
    config_file = "../config/planner_config.yaml"
    try:
        with open(config_file, "r") as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
    except FileNotFoundError:
        print(f"The file '{config_file}' does not exist.")
    except yaml.YAMLError as e:
        print(f"Error parsing the YAML file: {e}")


    # create the environment object
    env = Environment(config)

    
    # initialize planner
    path_planner = PathPlanner(env)


    # call path planner 
    path = path_planner.plan()


    # initialize and call the visualizer
    viz = Visualizer(env)
    viz.plot(path)



if __name__ == "__main__":
    main()