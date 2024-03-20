import json
import os
import re

# Function to recursively modify the JSON structure
def modify_json(data, filename):
    if isinstance(data, dict):
        for key, value in data.items():
            if key == 'x':
                data[key] = 16.54 - value
            elif key == 'rotation' or key == 'rotationDegrees':
                data[key] = (180 - value) % 360
                if data[key] > 180:
                    data[key] -= 360
            elif key == 'folder':
                data[key] = 'RED'
            elif key == 'pathName':
                data[key] = data[key].replace('BLUE', 'RED')
            else:
                modify_json(value, filename)
    elif isinstance(data, list):
        for item in data:
            modify_json(item, filename)

def rotate_end_state(data, filename):
    if re.match(r'(RED|BLUE)-(M|N|A|B)(\d)?-S.*\.path', filename):
        # add 90 degrees to the end state rotation
        # data['goalEndState']['rotation'] = (data['goalEndState']['rotation'] - 45) % 360
        # if data['goalEndState']['rotation'] > 180:
        #     data['goalEndState']['rotation'] -= 360
        pass

# Directory containing JSON files
directory = '.'

# Iterate through each JSON file in the directory
for filename in os.listdir(directory):
    if (filename.endswith('.path') or filename.endswith('.auto')) and filename.startswith('BLUE'):
        file_path = os.path.join(directory, filename)
        with open(file_path, 'r') as file:
            # Load JSON data
            json_data = json.load(file)
        
        # Modify JSON data
        modify_json(json_data, filename)

        rotate_end_state(json_data, filename)

        # remove "BLUE" from start of filename and write "RED"
        file_path = file_path.replace('BLUE', 'RED')
        
        # Write modified JSON data back to the file
        with open(file_path, 'w') as file:
            json.dump(json_data, file, indent=2)

print("All JSON files modified successfully.")
