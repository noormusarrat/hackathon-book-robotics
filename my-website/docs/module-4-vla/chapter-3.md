---
sidebar_position: 3
title: "Chapter 24: Voice Input Processing"
description: "Implementing voice recognition and processing systems for humanoid robots"
---

# Chapter 24: Voice Input Processing

## 1. Why this concept matters for humanoids

Voice input processing is fundamental to creating natural human-robot interaction for humanoid robots. Unlike traditional robots that require physical interfaces or pre-programmed commands, humanoid robots with voice input capabilities can respond to natural human speech, making them more intuitive and accessible. This capability enables seamless communication in environments where physical interfaces might be impractical, and allows robots to understand and respond to complex, nuanced verbal commands in real-time. Voice input processing is essential for social robots, personal assistants, and collaborative robots that work alongside humans.

## 2. Theory

Voice input processing for robotics involves converting human speech into text that can be understood and acted upon by the robot. This process typically involves several stages:

**Audio Capture**: Recording speech from the environment using microphones. For humanoid robots, this often involves microphone arrays to enable spatial audio processing and noise cancellation.

**Preprocessing**: Cleaning the audio signal by removing background noise, normalizing volume, and segmenting speech from non-speech audio. This step is crucial for improving recognition accuracy in real-world environments.

**Feature Extraction**: Converting the audio signal into features that can be processed by speech recognition models. Common approaches include Mel-frequency cepstral coefficients (MFCCs) or spectrograms.

**Automatic Speech Recognition (ASR)**: Converting the audio features into text using machine learning models. Modern ASR systems use deep neural networks trained on large datasets of speech.

**Post-processing**: Cleaning and formatting the recognized text to improve accuracy and prepare it for language understanding systems.

For robotics applications, voice input processing must be robust to environmental conditions, responsive to real-time input, and integrated with the robot's overall language understanding system.

## 3. Implementation

Let's implement a voice input processing node for our humanoid robot:

```python
# vla_robot_control/vla_robot_control/voice_processor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import numpy as np
import webrtcvad
import collections
import threading
import queue
import speech_recognition as sr
from vosk import Model, KaldiRecognizer
import json
import time

class VoiceProcessor(Node):
    def __init__(self, node):
        super().__init__('voice_processor')
        self.node = node

        # Initialize speech recognition
        self.setup_speech_recognition()

        # Audio parameters
        self.rate = 16000  # Sample rate
        self.chunk = 1024  # Audio chunk size
        self.channels = 1  # Mono audio
        self.format = pyaudio.paInt16

        # Voice activity detection
        self.vad = webrtcvad.Vad(2)  # Aggressiveness mode 2
        self.frame_duration = 30  # ms
        self.frame_size = int(self.rate * self.frame_duration / 1000)

        # Audio buffer for voice activity detection
        self.audio_buffer = collections.deque(maxlen=30)  # 30 frames = 900ms
        self.speech_buffer = collections.deque(maxlen=100)  # 3s of speech

        # Setup publisher for recognized text
        self.text_pub = self.node.create_publisher(String, '/vla/command', 10)

        # Setup audio stream
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk,
            stream_callback=self.audio_callback
        )

        # Thread-safe queue for processing
        self.audio_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.process_audio, daemon=True)
        self.processing_thread.start()

        self.get_logger().info('Voice Processor initialized')

    def setup_speech_recognition(self):
        """
        Setup speech recognition system
        """
        try:
            # Option 1: Use Vosk for offline speech recognition
            # self.model = Model("model")  # Download and specify model path
            # self.recognizer = KaldiRecognizer(self.model, self.rate)

            # Option 2: Use speech_recognition library with various backends
            self.rec = sr.Recognizer()
            self.microphone = sr.Microphone()

            # Adjust for ambient noise
            with self.microphone as source:
                self.rec.adjust_for_ambient_noise(source)

        except Exception as e:
            self.node.get_logger().warn(f'Could not initialize speech recognition: {e}')
            # Fallback to online recognition
            pass

    def audio_callback(self, in_data, frame_count, time_info, status):
        """
        Callback for audio stream
        """
        # Add audio data to processing queue
        self.audio_queue.put(in_data)
        return (None, pyaudio.paContinue)

    def process_audio(self):
        """
        Process audio data in a separate thread
        """
        while rclpy.ok():
            try:
                # Get audio data from queue
                if not self.audio_queue.empty():
                    audio_data = self.audio_queue.get(timeout=0.1)

                    # Convert to numpy array for processing
                    audio_array = np.frombuffer(audio_data, dtype=np.int16)

                    # Perform voice activity detection
                    if self.is_speech(audio_array):
                        # Add to speech buffer
                        self.speech_buffer.extend(audio_array)
                    else:
                        # If we have accumulated speech, process it
                        if len(self.speech_buffer) > 0:
                            self.process_speech_buffer()
                            self.speech_buffer.clear()
                else:
                    time.sleep(0.01)  # Sleep briefly if no data
            except queue.Empty:
                continue
            except Exception as e:
                self.node.get_logger().error(f'Error processing audio: {e}')

    def is_speech(self, audio_data):
        """
        Detect if the audio frame contains speech using WebRTC VAD
        """
        # VAD requires 16kHz, 16-bit, mono audio
        # We need to check if this frame contains speech
        try:
            # Convert to bytes for VAD
            audio_bytes = audio_data.tobytes()

            # The VAD needs smaller frames (10, 20, or 30ms)
            # For our 1024 samples at 16kHz, that's about 64ms chunks
            frame_size = self.frame_size * 2  # 2 bytes per sample

            speech_detected = False
            for i in range(0, len(audio_bytes), frame_size):
                frame = audio_bytes[i:i+frame_size]
                if len(frame) == frame_size:  # Ensure frame is the right size
                    if self.vad.is_speech(frame, self.rate):
                        speech_detected = True
                        break

            return speech_detected
        except Exception as e:
            self.node.get_logger().warn(f'VAD error: {e}')
            return False

    def process_speech_buffer(self):
        """
        Process accumulated speech buffer and recognize text
        """
        if len(self.speech_buffer) == 0:
            return

        # Convert speech buffer to audio data
        speech_data = np.array(self.speech_buffer).astype(np.int16)

        # Convert to audio data for speech recognition
        audio_bytes = speech_data.tobytes()

        try:
            # Create audio data object for speech recognition
            audio_data = sr.AudioData(audio_bytes, self.rate, 2)  # 2 bytes per sample

            # Recognize speech using Google Speech Recognition (or alternatives)
            # For offline recognition, you might use:
            # text = self.recognize_offline(audio_data)

            # For online recognition:
            text = self.rec.recognize_google(audio_data)

            if text and len(text.strip()) > 0:
                self.node.get_logger().info(f'Recognized: {text}')

                # Publish recognized text
                msg = String()
                msg.data = text
                self.text_pub.publish(msg)

        except sr.UnknownValueError:
            self.node.get_logger().info('Speech recognition could not understand audio')
        except sr.RequestError as e:
            self.node.get_logger().error(f'Speech recognition error: {e}')
        except Exception as e:
            self.node.get_logger().error(f'Error in speech processing: {e}')

    def recognize_offline(self, audio_data):
        """
        Offline speech recognition using Vosk
        """
        # This would use the Vosk model for offline recognition
        # result = self.recognizer.AcceptWaveform(audio_data.get_wav_data())
        # if result:
        #     result_json = json.loads(self.recognizer.Result())
        #     return result_json.get("text", "")
        pass

def main(args=None):
    rclpy.init(args=args)
    # Create a dummy node to pass to VoiceProcessor
    dummy_node = Node('dummy_node')
    voice_processor = VoiceProcessor(dummy_node)

    try:
        # Keep the node running
        rclpy.spin(dummy_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_processor.stream.stop_stream()
        voice_processor.stream.close()
        voice_processor.audio.terminate()
        dummy_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Create a configuration file for voice processing:

```yaml
# vla_robot_control/config/voice_processing.yaml
voice_processing:
  sample_rate: 16000
  chunk_size: 1024
  channels: 1
  format: paInt16
  vad_aggressiveness: 2
  silence_threshold: 0.5
  min_speech_duration: 0.5  # seconds
  max_speech_duration: 5.0  # seconds
  language: "en-US"
  sensitivity: 0.5
  noise_suppression: 2
  auto_gain_control: true
  voice_activity_timeout: 3.0  # seconds of silence before considering speech ended
```

Create a launch file for the voice processing system:

```xml
<!-- vla_robot_control/launch/voice_processing.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('vla_robot_control'),
        'config',
        'voice_processing.yaml'
    )

    return LaunchDescription([
        Node(
            package='vla_robot_control',
            executable='voice_processor',
            name='voice_processor',
            parameters=[config],
            output='screen'
        )
    ])
```

## 4. Hardware/GPU Notes

Voice input processing for humanoid robots has specific hardware requirements:

- **Microphone Array**: Preferably 4-8 microphones for beamforming and noise cancellation
- **Audio Processing**: Dedicated audio processing unit or DSP for real-time processing
- **CPU Requirements**: Multi-core processor for concurrent audio processing and robot control
- **Memory**: 2-4GB RAM for audio buffers and recognition models (offline)
- **Network**: Stable connection for online speech recognition (if used)

**Offline vs Online Recognition**:
- **Offline**: Higher initial setup but no network dependency, privacy guaranteed
- **Online**: Lower hardware requirements but needs network connection, potential privacy concerns

**Audio Quality Considerations**:
- Use high-quality microphones with wide frequency response
- Implement noise cancellation for real-world environments
- Consider directional microphones for focused speech capture

## 5. Simulation Path

To implement voice processing in simulation:

1. **Audio Simulation Setup**:
   ```bash
   # Use Gazebo audio plugin or simulate audio input
   # Create virtual microphone that receives audio from simulation
   ```

2. **Voice Command Injection**:
   ```python
   # Simulate voice commands by publishing directly to the command topic
   ros2 topic pub /vla/command std_msgs/String "data: 'move forward'"
   ```

3. **Testing Framework**:
   ```python
   # Create test scenarios with pre-recorded audio or text commands
   import unittest
   from std_msgs.msg import String

   class TestVoiceProcessor(unittest.TestCase):
       def test_voice_command_recognition(self):
           # Simulate voice input and verify correct text output
           pass
   ```

4. **Integration Testing**:
   - Test voice command flow from audio input to robot action
   - Validate response times and accuracy in simulated environment
   - Verify safety mechanisms work with voice commands

## 6. Real-World Path

For real-world deployment:

1. **Hardware Integration**:
   - Mount microphone array on the robot in optimal position
   - Ensure proper audio routing and amplification
   - Test audio quality in target environments

2. **Calibration**:
   - Adjust sensitivity based on environment noise levels
   - Calibrate for different speakers and accents
   - Set appropriate thresholds for voice activity detection

3. **Deployment Considerations**:
   - Implement fallback mechanisms for recognition failures
   - Add privacy controls for sensitive environments
   - Ensure compliance with audio recording regulations

4. **Performance Optimization**:
   - Optimize for real-time processing (aim for 200ms response)
   - Implement caching for common commands
   - Add adaptive noise cancellation

## 7. Spec-Build-Test checklist

- [ ] Voice processing node implemented and running
- [ ] Audio capture working with microphone array
- [ ] Voice activity detection functioning correctly
- [ ] Speech recognition converting audio to text
- [ ] Recognized text published to command topic
- [ ] Noise cancellation implemented and tested
- [ ] Response time under 500ms for real-time interaction
- [ ] Accuracy tested with various speakers and accents
- [ ] Fallback mechanisms for recognition failures
- [ ] Privacy controls implemented for audio processing

## 8. APA citations

1. Hinton, G., Deng, L., Yu, D., Dahl, G. E., Mohamed, A. R., Jaitly, N., ... & Kingsbury, B. (2012). Deep neural networks for acoustic modeling in speech recognition: The shared views of four research groups. *IEEE Signal Processing Magazine*, 29(6), 82-97.

2. Panayotov, V., Chen, G., Povey, D., & Khudanpur, S. (2015). Librispeech: an ASR corpus based on public domain audio books. *2015 IEEE International Conference on Acoustics, Speech and Signal Processing (ICASSP)*, 5206-5210.

3. Saon, G., Soltau, H., Povey, D., & Khudanpur, S. (2020). A streaming transformer transducer approach for end-to-end speech recognition. *arXiv preprint arXiv:2006.14941*.

4. Vinyals, O., Senior, A., Vozila, K., & Jaitly, N. (2017). Grammar as a foreign language. *Advances in Neural Information Processing Systems*, 30.

5. Zhang, Y., Pezeshki, M., Brakel, P., Zhang, S., Bengio, Y., & Pal, C. (2017). Towards end-to-end speech recognition with deep convolutional neural networks. *arXiv preprint arXiv:1701.02720*.