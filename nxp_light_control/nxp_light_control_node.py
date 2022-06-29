#!/usr/bin/env python3
import rclpy
import numpy as np
import time
from rclpy.node import Node
from tflite_msgs.msg import TFLite, TFInference
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist

class NXPLightControl(Node):
    def __init__(self):
        super().__init__('nxp_light_control_node')

        led_pattern_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Twist command output topic name.')

        command_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Twist command input topic name.')

        tf_lite_topic_0_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='TFLite_0 returns topic.')

        
        tf_lite_topic_1_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='TFLite_1 returns topic.')

        threshold_0_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Threshold_0 value for inference score.')

        threshold_1_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Threshold_1 value for inference score.')

        blink_delay_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Blink delay in seconds.')

        self.declare_parameter("led_pattern_topic", "/led_pattern", 
            led_pattern_topic_descriptor)

        self.declare_parameter("cmd_topic", "/cmd_vel", 
            command_topic_descriptor)

        self.declare_parameter("tflite_topic_0", "/TFLiteSim", 
            tf_lite_topic_0_descriptor)

        self.declare_parameter("tflite_topic_1", "/TFLiteReal", 
            tf_lite_topic_1_descriptor)

        self.declare_parameter("threshold_0", 0.5, 
            threshold_0_descriptor)

        self.declare_parameter("threshold_1", 0.5, 
            threshold_1_descriptor)

        self.declare_parameter("blink_delay", 0.3, 
            blink_delay_descriptor)

        self.TFLiteTopic0 = self.get_parameter("tflite_topic_0").value
        self.TFLiteTopic1 = self.get_parameter("tflite_topic_1").value
        self.Threshold0 = float(self.get_parameter("threshold_0").value)
        self.Threshold1 = float(self.get_parameter("threshold_1").value)
        self.CmdSubTopic = self.get_parameter("cmd_topic").value
        self.patternTopic = self.get_parameter("led_pattern_topic").value
        self.BlinkDelaySec = float(self.get_parameter("blink_delay").value)

        self.TFLiteSub0 = self.create_subscription(TFLite, '{:s}'.format(self.TFLiteTopic0), self.TFLiteCallback0, 1)
        self.TFLiteSub1 = self.create_subscription(TFLite, '{:s}'.format(self.TFLiteTopic1), self.TFLiteCallback1, 1)
        self.CmdSub = self.create_subscription(Twist, '{:s}'.format(self.CmdSubTopic), self.CMDCallback, 1)

        self.PatternPub = self.create_publisher(String,'{:s}'.format(self.patternTopic), 0)

        self.BlinkPatternFinished = True
        self.StopMotors0 = False
        self.StopMotors1 = False

    def CMDCallback(self, msgVel):
        linearX = msgVel.linear.x
        angularZ = msgVel.angular.z

        if self.StopMotors0 or self.StopMotors1:
            self.Pattern("SADFACE")
            return
        if linearX > 0.1:
            self.Pattern("FORWARDGREEN")
        if linearX < -0.1:
            self.Pattern("REVERSE")
        if linearX >= -0.1 and linearX <= 0.1:
            self.Pattern("STOP")
        if angularZ < -0.3:
            self.Pattern("RIGHTYELLOW")
        if angularZ > 0.3:
            self.Pattern("LEFTYELLOW")
        return

    def TFLiteCallback0(self, msgTFLite):
        for inference in msgTFLite.inference:
            if inference.score >= self.Threshold0:
                if inference.label.upper() == "PERSON":
                    self.StopMotors0 = True
                    self.Pattern("PERSON")
                    return
        self.StopMotors0 = False
        return

    def TFLiteCallback1(self, msgTFLite):
        for inference in msgTFLite.inference:
            if inference.score >= self.Threshold1:
                if inference.label.upper() == "PERSON":
                    self.StopMotors1 = True
                    self.Pattern("PERSON")
                    return
        self.StopMotors1 = False
        return

    def Pattern(self, pattern):
        if self.BlinkPatternFinished and pattern == "PERSON":
            self.BlinkPatternFinished = False
            msg1 = String()
            msg1.data = "PERSON"
            msg2 = String()
            msg2.data = "HAZARD"
            self.PatternPub.publish(msg2)
            time.sleep(self.BlinkDelaySec)
            self.PatternPub.publish(msg1)
            time.sleep(self.BlinkDelaySec)
            self.BlinkPatternFinished = True
        elif self.BlinkPatternFinished and pattern != "PERSON":
            self.BlinkPatternFinished = False
            msg3 = String()
            msg3.data = pattern
            self.PatternPub.publish(msg3)
            time.sleep(self.BlinkDelaySec)
            self.BlinkPatternFinished = True
        return

def main(args=None):
    rclpy.init()
    NXPLC = NXPLightControl()
    rclpy.spin(NXPLC)
    NXPLC.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
