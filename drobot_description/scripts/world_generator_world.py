def generate_world_file(world_name, output_path):
    world_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{world_name}">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
"""
    with open(output_path, 'w') as file:
        file.write(world_content)
    print(f"World file '{output_path}' generated successfully.")