import pyaudio 
import os

# speech model recognition name
model_name = 'model_11'
model_path = os.path.join(os.path.dirname(__file__),f'{model_name}.h5')
# list labels
labels = ['background','close','pen','phone','stop','track']
labels.sort()

# mic recording parameters
FRAMES_PER_BUFFER = 3200
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

duration = 1 # duration of audio file to detect