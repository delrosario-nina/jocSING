import librosa
import numpy as np
import sys

def extract_pitch_from_mp3(input_mp3, output_txt, hop_length=512):
    """
    Extract pitch from MP3 file and save to TXT file with timestamps.
    
    Parameters:
    - input_mp3: Path to input MP3 file
    - output_txt: Path to output TXT file
    - hop_length: Number of samples between successive frames (affects time resolution)
                  Smaller values = more frequent pitch measurements
                  Default 512 ≈ 11.6ms intervals at 44.1kHz sample rate
    """
    
    print(f"Loading audio file: {input_mp3}")
    # Load the audio file
    y, sr = librosa.load(input_mp3, sr=None)
    
    print("Extracting pitch information...")
    # Extract pitch using piptrack (pitch tracking)
    pitches, magnitudes = librosa.piptrack(y=y, sr=sr, hop_length=hop_length)
    
    # Get the pitch with the highest magnitude at each time frame
    pitch_data = []
    for t in range(pitches.shape[1]):
        index = magnitudes[:, t].argmax()
        pitch = pitches[index, t]
        if pitch > 0:  # Only consider non-zero pitches
            time = librosa.frames_to_time(t, sr=sr, hop_length=hop_length)
            pitch_data.append((time, pitch))
    
    # Write to output file
    print(f"Writing pitch data to: {output_txt}")
    with open(output_txt, 'w') as f:
        for time, pitch in pitch_data:
            minutes = int(time // 60)
            seconds = time % 60
            timestamp = f"[{minutes:02d}:{seconds:05.2f}]"
            f.write(f"{timestamp}{pitch:.2f}\n")
    
    print(f"Successfully extracted {len(pitch_data)} pitch samples!")
    print(f"Output saved to: {output_txt}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python pitch_extractor.py <input.mp3> [output.txt] [hop_length]")
        print("\nExample: python pitch_extractor.py song.mp3 output.txt 512")
        print("\nParameters:")
        print("  input.mp3    - Path to input MP3 file (required)")
        print("  output.txt   - Path to output TXT file (optional, default: pitch_output.txt)")
        print("  hop_length   - Samples between frames (optional, default: 512)")
        print("                 Smaller = more frequent measurements (e.g., 256, 128)")
        print("                 512 ≈ 11.6ms intervals, 256 ≈ 5.8ms, 128 ≈ 2.9ms at 44.1kHz")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "pitch_output.txt"
    hop = int(sys.argv[3]) if len(sys.argv) > 3 else 512
    
    try:
        extract_pitch_from_mp3(input_file, output_file, hop_length=hop)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)