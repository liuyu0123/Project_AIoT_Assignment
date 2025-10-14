import rclpy
from rclpy.node import Node
from autopatrol_interfaces.srv import SpeachText
import espeakng


class Speaker(Node):
    def __init__(self, node_name='speaker'):
        super().__init__(node_name)
        self.speech_service = self.create_service(SpeachText, 'speach_text', self.speak_text_callback)
        self.speaker = espeakng.Speaker()
        self.speaker.voice = 'zh'


    def speak_text_callback(self, request, response):
        self.get_logger().info('Received text: %s' % request.text)
        self.speaker.say(request.text)
        self.speaker.wait()
        response.result = True
        return response



def main(args=None):
    rclpy.init(args=args)
    speaker = Speaker('speaker')
    rclpy.spin(speaker)
    rclpy.shutdown()