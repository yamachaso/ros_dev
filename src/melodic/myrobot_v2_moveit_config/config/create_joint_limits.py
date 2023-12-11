import yaml

scaling = 0.3


with open('joint_limits_max.yaml', 'r') as yml:
  config = yaml.safe_load(yml)


for i in config["joint_limits"]:
  for j in config["joint_limits"]["{}".format(i)]:
    if j == "joint_back":
      continue
    if j[0] == 'h':
      continue
    config["joint_limits"]["{}".format(i)]["{}".format(j)] *= scaling

# config["joint_limits"]["joint_back"]["max_velocity"] = 0.3
config["joint_limits"]["joint_back"]["max_acceleration"] = 0.2


with open('joint_limits.yaml','w') as f:
   yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
