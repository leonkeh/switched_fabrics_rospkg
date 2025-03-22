import yaml

yaml_file = '/home/leon/dinova_ws/experiments/experiment_settings.yaml'
with open(yaml_file, 'r') as file:
    data = yaml.safe_load(file)
settings = data["settings"]

obstacle_data = [j for obst in settings["obstacles"] for j in [str(obst[0]), str(obst[1]), str(obst[2])]]

print(*obstacle_data)

print(3 // 3, 6 // 3, 7//3)