import os

input_folder_path = "./4team_data/Annotations"
output_folder_path = "./4team_data/changed_Annotations"

# Ensure output folder exists
os.makedirs(output_folder_path, exist_ok=True)
for filename in os.listdir(input_folder_path):
    if filename.endswith(".txt"):
        print(filename)
        input_file_path = os.path.join(input_folder_path, filename)
        output_file_path = os.path.join(output_folder_path, filename)
        with open(input_file_path, "r") as input_file, open(output_file_path, "w") as output_file:
            for line in input_file:
                values = line.split()
                current_value = int(values[0])
                if current_value == 2:
                    values[0] = '3'
                elif current_value == 3:
                    values[0] = '2'
                elif current_value == 4:
                    values[0] = '5'
                elif current_value == 5:
                    values[0] = '7'
                elif current_value == 7:
                    values[0] = '4'
                # elif current_value == 9:  # 여기서부터 수정 (ignore)
                    # values[0] = '8'
                modified_line = ' '.join(map(str, values))
                output_file.write(modified_line + "\n")

print("Conversion complete.")