#!/usr/bin/env python3
"""
ASR to navigation bridge
- Listens to microphone (if speech_recognition is installed) and/or subscribes to `/asr/text` (std_msgs/String).
- When recognized text contains the keyword "大门" -> publish PoseStamped to `/goal_pose` with (0,0,0)
- When recognized text contains "A107" (or variations) -> publish PoseStamped to `/goal_pose` with (3,3,3)

Usage:
  python3 asr_nav.py

Notes:
- If `speech_recognition` is not installed, the node will still work as a subscriber to `/asr/text`.
- If you use the microphone path, you may need to install `SpeechRecognition` and a microphone driver (pyaudio etc.)
"""

import threading
import re
import time
import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# Try Vosk + sounddevice first (recommended)
HAS_VOSK = False
HAS_SD = False
VOSK_MODEL = None
try:
    import vosk
    HAS_VOSK = True
except Exception:
    HAS_VOSK = False

try:
    import sounddevice as sd
    HAS_SD = True
except Exception:
    HAS_SD = False

# optional: speech_recognition fallback
try:
    import speech_recognition as sr
    HAS_SR = True
except Exception:
    HAS_SR = False

# Chinese numerals map for simple normalization
CN_NUM_MAP = {
    '零': '0', '一': '1', '二': '2', '三': '3', '四': '4', '五': '5',
    '六': '6', '七': '7', '八': '8', '九': '9', '〇': '0', '○': '0'
}


def normalize_text(text: str) -> str:
    """Lower and normalize common variations for matching."""
    if not text:
        return ''
    s = text.strip()
    # replace Chinese numerals
    for k, v in CN_NUM_MAP.items():
        s = s.replace(k, v)
    # remove spaces
    s = re.sub(r"\s+", '', s)
    return s


class AsrNavNode(Node):
    def __init__(self):
        super().__init__('asr_nav')
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(String, '/asr/text', self.asr_text_cb, 10)

        self.get_logger().info('ASR navigation node started')
        # Prefer Vosk + sounddevice if available
        self._stop_listen = threading.Event()
        if HAS_VOSK and HAS_SD:
            # find model path from env or common places
            model_env = os.environ.get('VOSK_MODEL_PATH')
            candidates = [model_env, os.path.join(os.path.expanduser('~'), 'vosk-model'), os.path.join(os.getcwd(), 'model'), 'model']
            model_path = None
            for p in candidates:
                if not p:
                    continue
                if os.path.isdir(p):
                    model_path = p
                    break

            if model_path is None:
                self.get_logger().warn('Vosk available but no model directory found. Set VOSK_MODEL_PATH or place model in ./model or ~/vosk-model. Microphone disabled for Vosk.')
            else:
                try:
                    self.vosk_model = vosk.Model(model_path)
                    self.get_logger().info(f'Vosk model loaded from {model_path}, microphone listening enabled via Vosk')
                    self._thread = threading.Thread(target=self._vosk_listen_loop, daemon=True)
                    self._thread.start()
                    # mark as used
                    HAS_VOSK = True
                except Exception as e:
                    self.get_logger().error(f'Failed to load Vosk model from {model_path}: {e}')

        # fallback to speech_recognition if Vosk not enabled
        if not getattr(self, 'vosk_model', None) and HAS_SR:
            self.get_logger().info('speech_recognition available: microphone listening enabled')
            self._thread = threading.Thread(target=self._mic_listen_loop, daemon=True)
            self._thread.start()

        if not getattr(self, 'vosk_model', None) and not HAS_SR:
            self.get_logger().warn('No microphone ASR available. Publish ASR text to /asr/text to trigger goals.')

    def _mic_listen_loop(self):
        recognizer = sr.Recognizer()
        mic = None
        try:
            mic = sr.Microphone()
        except Exception as e:
            self.get_logger().error(f'Failed to open microphone: {e}')
            return

        with mic as source:
            recognizer.adjust_for_ambient_noise(source, duration=1.0)
            self.get_logger().info('Microphone ready, listening...')
            while rclpy.ok() and not self._stop_listen.is_set():
                try:
                    # listen for a phrase (blocking)
                    audio = recognizer.listen(source, phrase_time_limit=5)
                    # try Google (online) first
                    try:
                        text = recognizer.recognize_google(audio, language='zh-CN')
                    except Exception:
                        # fallback to sphinx if available
                        try:
                            text = recognizer.recognize_sphinx(audio, language='zh-CN')
                        except Exception as e:
                            self.get_logger().warn(f'ASR failed: {e}')
                            continue

                    self.get_logger().info(f'ASR(mic) recognized: "{text}"')
                    self.process_recognized_text(text)
                except Exception as e:
                    self.get_logger().error(f'Microphone listening error: {e}')
                    time.sleep(0.5)

    def _vosk_listen_loop(self):
        # Uses sounddevice RawInputStream to feed Vosk recognizer
        try:
            rec = vosk.KaldiRecognizer(self.vosk_model, 16000)
        except Exception as e:
            self.get_logger().error(f'Failed to create Vosk recognizer: {e}')
            return

        def callback(indata, frames, time_info, status):
            if status:
                self.get_logger().debug(f'sounddevice status: {status}')
            try:
                if rec.AcceptWaveform(indata.tobytes()):
                    res = rec.Result()
                    # result is a JSON string
                    try:
                        j = json.loads(res)
                        text = j.get('text', '')
                        if text:
                            self.get_logger().info(f'Vosk recognized: "{text}"')
                            self.process_recognized_text(text)
                    except Exception:
                        pass
                else:
                    # partial = rec.PartialResult()
                    pass
            except Exception as e:
                self.get_logger().error(f'Vosk processing error: {e}')

        try:
            with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16', channels=1, callback=callback):
                self.get_logger().info('Vosk microphone loop started (press Ctrl-C to stop)')
                while rclpy.ok() and not self._stop_listen.is_set():
                    time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f'sounddevice input stream failed: {e}')

    def asr_text_cb(self, msg: String):
        txt = msg.data
        self.get_logger().info(f'ASR(topic) received: "{txt}"')
        self.process_recognized_text(txt)

    def process_recognized_text(self, txt: str):
        if not txt:
            return
        s = normalize_text(txt)
        s_upper = s.upper()

        # match '大门'
        if '大门' in txt or '大門' in txt or '大门' in s or '大門' in s:
            self.get_logger().info('Keyword "大门" detected -> publishing goal (0,0,0)')
            self.publish_goal(0.0, 0.0, 0.0)
            return

        # match A107 variants: A107, A 107, A一零七, a107
        # normalize: remove spaces, convert chinese numerals, uppercase
        # Examples to match: 'A107', 'A107教室', '教室A107', 'A一零七', 'A107教'
        # find pattern A followed by digits
        m = re.search(r'A\s*([0-9零一二三四五六七八九〇○]+)', txt, re.I)
        if not m:
            # try on normalized string
            m2 = re.search(r'A([0-9]+)', s_upper)
            if m2:
                digits = m2.group(1)
            else:
                digits = None
        else:
            digits_raw = m.group(1)
            # replace chinese numerals
            digits = ''.join(CN_NUM_MAP.get(ch, ch) for ch in digits_raw)

        if digits:
            try:
                if digits == '107':
                    self.get_logger().info('Keyword "A107" detected -> publishing goal (3,3,3)')
                    self.publish_goal(3.0, 3.0, 3.0)
                    return
            except Exception:
                pass

        # also handle explicit 'A107' in recognized normalized text
        if 'A107' in s_upper:
            self.get_logger().info('Keyword "A107" detected in normalized text -> publishing goal (3,3,3)')
            self.publish_goal(3.0, 3.0, 3.0)
            return

    def publish_goal(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # identity orientation
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)

    def destroy_node(self):
        if HAS_SR and hasattr(self, '_stop_listen'):
            self._stop_listen.set()
            if hasattr(self, '_thread'):
                self._thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AsrNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
