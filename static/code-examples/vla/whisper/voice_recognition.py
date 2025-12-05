#!/usr/bin/env python3
"""
Whisper Voice Recognition for Robotics

Convert speech to text using OpenAI Whisper for robot commands.

Dependencies: openai-whisper, numpy, pyaudio
"""

import whisper
import numpy as np
import pyaudio
import wave
import tempfile
import os

class WhisperVoiceRecognizer:
    """
    Real-time voice recognition using Whisper.
    """

    def __init__(self, model_size="base"):
        """
        Initialize Whisper model.

        Args:
            model_size (str): Model size (tiny, base, small, medium, large)
        """
        print(f"Loading Whisper {model_size} model...")
        self.model = whisper.load_model(model_size)
        print("Model loaded successfully")

        # Audio settings
        self.sample_rate = 16000
        self.chunk_size = 1024
        self.channels = 1

    def record_audio(self, duration=5):
        """
        Record audio from microphone.

        Args:
            duration (int): Recording duration in seconds

        Returns:
            str: Path to recorded audio file
        """
        audio = pyaudio.PyAudio()

        print(f"Recording for {duration} seconds...")

        stream = audio.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        frames = []
        for _ in range(0, int(self.sample_rate / self.chunk_size * duration)):
            data = stream.read(self.chunk_size)
            frames.append(data)

        print("Recording complete")

        stream.stop_stream()
        stream.close()
        audio.terminate()

        # Save to temporary file
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
        wf = wave.open(temp_file.name, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(audio.get_sample_size(pyaudio.paInt16))
        wf.setframerate(self.sample_rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return temp_file.name

    def transcribe(self, audio_path):
        """
        Transcribe audio file to text.

        Args:
            audio_path (str): Path to audio file

        Returns:
            dict: Transcription result with text and confidence
        """
        print("Transcribing audio...")
        result = self.model.transcribe(audio_path)

        print(f"Transcription: {result['text']}")

        return result

    def recognize_command(self, duration=5):
        """
        Record and recognize voice command.

        Args:
            duration (int): Recording duration

        Returns:
            str: Recognized command text
        """
        audio_file = self.record_audio(duration)

        try:
            result = self.transcribe(audio_file)
            command = result['text'].strip()
            return command
        finally:
            # Clean up temporary file
            os.remove(audio_file)

def main():
    """
    Demo: Voice command recognition.
    """
    recognizer = WhisperVoiceRecognizer(model_size="base")

    print("\nVoice Command Recognition Demo")
    print("Speak a command after the beep...")

    command = recognizer.recognize_command(duration=5)

    print(f"\nRecognized command: '{command}'")
    print(f"\nYou could now:")
    print(f"  1. Parse command with LLM")
    print(f"  2. Map to ROS 2 actions")
    print(f"  3. Execute robot behavior")

if __name__ == "__main__":
    main()
