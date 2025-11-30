import librosa
import soundfile as sf
import numpy as np
from pydub import AudioSegment
import os

def change_pitch(input_file, output_file, semitones):
    """
    Change the pitch of an MP3 file.
    
    Parameters:
    - input_file: Path to the input MP3 file
    - output_file: Path to save the output file
    - semitones: Number of semitones to shift (positive = higher, negative = lower)
                 Example: 2 = shift up 2 semitones, -3 = shift down 3 semitones
    """
    
    print(f"Loading audio file: {input_file}")
    
    # Convert MP3 to WAV temporarily (librosa works better with WAV)
    audio = AudioSegment.from_mp3(input_file)
    temp_wav = "temp_audio.wav"
    audio.export(temp_wav, format="wav")
    
    # Load the audio file with librosa
    y, sr = librosa.load(temp_wav, sr=None)
    
    print(f"Shifting pitch by {semitones} semitones...")
    
    # Shift the pitch
    y_shifted = librosa.effects.pitch_shift(y, sr=sr, n_steps=semitones)
    
    # Save the pitch-shifted audio
    print(f"Saving to: {output_file}")
    
    # Determine output format from file extension
    output_ext = os.path.splitext(output_file)[1].lower()
    
    if output_ext == '.mp3':
        # Save as WAV first, then convert to MP3
        temp_output_wav = "temp_output.wav"
        sf.write(temp_output_wav, y_shifted, sr)
        shifted_audio = AudioSegment.from_wav(temp_output_wav)
        shifted_audio.export(output_file, format="mp3", bitrate="192k")
        os.remove(temp_output_wav)
    else:
        # Save directly as WAV or other format
        sf.write(output_file, y_shifted, sr)
    
    # Clean up temporary file
    os.remove(temp_wav)
    
    print("Done!")

if __name__ == "__main__":
    # Change these parameters
    input_mp3 = "Buko_backtrack.mp3"  # Your input MP3 file
    output_mp3 = "0110.mp3"  # Output file name
    pitch_shift = -2  # Shift down by 2 semitones (change as needed)
    
    # Check if input file exists
    if not os.path.exists(input_mp3):
        print(f"Error: Input file '{input_mp3}' not found!")
        print("Please update the 'input_mp3' variable with your MP3 file path.")
    else:
        change_pitch(input_mp3, output_mp3, pitch_shift)