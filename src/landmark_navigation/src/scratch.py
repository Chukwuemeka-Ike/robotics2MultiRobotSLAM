import yaml
import sys

# Get the landmarks_file argument.
if len(sys.argv) != 2:
    print(
        "Incorrect number of arguments. Using default landmarks file "
        "landmarks/landmarks.yaml"
    )
    landmarks_file = "landmarks/landmarks.yaml"
else:
    landmarks_file = sys.argv[1]
print(len(sys.argv))
print(landmarks_file)

# Load the landmarks file.
with open(landmarks_file, 'r') as file:
    landmarks_dict = yaml.safe_load(file)
landmarks = list(landmarks_dict.keys())

# Create the landmarks list portion of the CLI message.
landmarks_msg = ""
for landmark_idx, landmark in enumerate(landmarks):
    if landmark_idx < len(landmarks)-1:
        landmarks_msg += "\t{}\n".format(landmark)
    else:
        landmarks_msg += "\t{}".format(landmark)


msg = """
The map has the following landmarks:
{}

Type the desired landmark # to start navigation to it.

CTRL-C to quit.
""".format(landmarks_msg)
print(msg)