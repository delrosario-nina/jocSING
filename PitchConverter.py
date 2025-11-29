import librosa
import numpy as np

# ==== SETTINGS ====
INPUT_FILE = "0101.mp3"     # Change to your song file
OUTPUT_FILE = "0101_pitch.lrc"
FRAME_HOP = 512             # Hop length (smaller = more precise)
SR = 22050                  # Sampling rate for analysis
PITCH_MIN = 50.0            # Lowest pitch to consider (Hz)
PITCH_MAX = 2000.0          # Highest pitch to consider (Hz)

# ==== LOAD AUDIO ====
print("Loading audio...")
y, sr = librosa.load(INPUT_FILE, sr=SR, mono=True)

# ==== ESTIMATE PITCH ====
print("Estimating pitch...")
f0, voiced_flag, voiced_probs = librosa.pyin(
    y,
    fmin=PITCH_MIN,
    fmax=PITCH_MAX,
    sr=sr,
    frame_length=2048,
    hop_length=FRAME_HOP
)

# ==== CONVERT TO TIMESTAMPS ====
times = librosa.times_like(f0, sr=sr, hop_length=FRAME_HOP)

# ==== WRITE TO FILE ====
print(f"Writing results to {OUTPUT_FILE}...")
with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
    for t, pitch, voiced in zip(times, f0, voiced_flag):
        if voiced and pitch is not None:
            minutes = int(t // 60)
            seconds = t % 60
            f.write(f"{minutes:02d}:{seconds:05.2f},{pitch:.2f}\n")

print("✅ Done! Pitch data exported successfully.")
