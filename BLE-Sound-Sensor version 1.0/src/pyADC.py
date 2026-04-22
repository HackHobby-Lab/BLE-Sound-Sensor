import numpy as np
from scipy.io.wavfile import write
import re

# Load data from your logged file
with open("E:/BleSoundSensor/April/adc/src/coolStream.txt", "r") as f:
    raw = f.read()

# Extract all valid integers (including negatives)
data = list(map(int, re.findall(r'-?\d+', raw)))

# Convert to NumPy array
audio_data = np.array(data, dtype=np.int16)  # Ensure 16-bit PCM format

# Save to WAV (3000 Hz, mono)
write("output.wav", 1500, audio_data)
