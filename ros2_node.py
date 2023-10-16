import rclpy
import threading
import time

from rclpy.node import Node
import utils


class TopicStructure:
    def __init__(self, name, tp):
        self.name = name
        self.type = tp
        self.children = []
        self.parent = None

    def addChild(self, child):
        self.children.append(child)
        child.parent = self

    def childCount(self):
        return len(self.children)

    def row(self):
        if self.parent:
            return self.parent.children.index(self)
        return 0


class Ros2Utils:
    def __init__(self, node):
        self.node = node

    def get_running_topics(self):
        result = []

        topics_info = self.node.get_topic_names_and_types()
        for topic_info_name, _ in topics_info:
            result.append(topic_info_name)

        return result

    def get_node_type(self, topic_name):
        topic_type = None
        topic_type_name = None
        topics_info = self.node.get_topic_names_and_types()
        for topic_info_name, topic_info_types in topics_info:
            if topic_info_name == topic_name:
                topic_type_name = topic_info_types[0]
                topic_type = utils.get_topic_class(topic_info_types[0])
                break

        return topic_type_name, topic_type

    def check_field_appearence(self, field: str):
        ts = TopicStructure(None, None)

        parts = field.split(".")
        topic = parts[0]
        fields = parts[1:]

        _, topic_type = self.get_node_type(topic)
        if topic_type:
            self.analyze_type(topic_type, ts)

            child = ts
            for field_item in fields:
                found = False
                for child_item in child.children:
                    if child_item.name == field_item:
                        child = child_item
                        found = True
                        break

                if not found:
                    return False

            return True

        return False

    def analyze_type(self, topic_type, topic_struct: TopicStructure):
        dummy_msg = topic_type()
        fields = dummy_msg.get_fields_and_field_types()
        for field, field_type in fields.items():
            sub_topic_struct = TopicStructure(field, field_type)
            temp_type = self.__check_template(field_type)
            if temp_type:
                field_type = temp_type
            index = field_type.find("/")
              # Only classes contains this symbol
            if index > 0:
                field_property_type = utils.get_topic_class(field_type)
                self.analyze_type(field_property_type, sub_topic_struct)

            topic_struct.children.append(sub_topic_struct)
            sub_topic_struct.parent = topic_struct

        return topic_struct
    
    def __check_template(self, type:str):
        i1 = type.find("<")
        i2 = type.find(">")

        if i1 != -1 and i2 != -1:
            new_type = type[i1+1:i2]
            return new_type


class Ros2Node(Node):
    def __init__(self):
        super().__init__("topic_monitor_node")
        self.callback = None
        self.status_callback = None
        self.stop_request = True
        self.thread = threading.Thread(target=self.__thread_loop)
        self.topic_subscriptions = {}
        self.ros2_utils = Ros2Utils(self)

    def add_subscription(self, topic_name):
        for key_topic in self.topic_subscriptions:
            if key_topic == topic_name:
                found = True
                print(f"[DEBUG] Already subscribed to {topic_name}")
                self.__push_status(f"Already subscribed to '{topic_name}'")
                return

        topic_type_name, topic_type = self.ros2_utils.get_node_type(topic_name)
        if topic_type_name and topic_type:
            subscription = self.create_subscription(
                topic_type,
                topic_name,
                lambda msg: self.__subscription_callback(msg, topic_type),
                10,
            )
            self.topic_subscriptions[topic_name] = subscription
        elif not topic_type_name:
            print(f"[DEBUG] Topic type name not found for {topic_name}")
            self.__push_status(f"Topic type name not found for '{topic_name}'")
        else:
            print(f"[DEBUG] Topic type not found for {topic_name}")
            self.__push_status(f"Topic type not found for '{topic_name}'")

        self.__push_status(f"Subscribed to '{topic_name}'")
        print(f"[DEBUG] Subscribed to '{topic_name}'")

    def remove_subscription(self, topic_name):
        key = None
        for key_topic in self.topic_subscriptions:
            if key_topic == topic_name:
                key = key_topic
                break
        if key:
            print(f"[DEBUG] Subscription {topic_name} removed")
            self.__push_status(f"Subscription '{topic_name}' removed")
            self.destroy_subscription(self.topic_subscriptions[key])
            self.topic_subscriptions.pop(key)
        else:
            self.__push_status(f"Subscription '{topic_name}' not found")
            print(f"[DEBUG] Subscription {topic_name} not found")

    def start_node(self):
        print("[DEBUG] Try to start node")
        self.stop_request = False
        if not self.thread.is_alive():
            self.thread.start()
        else:
            print("[DEBUG] Node already started")

    def stop_node(self):
        print("[DEBUG] Stop node")
        self.stop_request = True
        if self.thread.is_alive():
            self.thread.join()

    def set_status_callback(self, callback):
        self.status_callback = callback


    def set_callback(self, callback):
        self.callback = callback

    def __thread_loop(self):
        print("[DEBUG] Start node loop")

        while not self.stop_request:
            rclpy.spin_once(self, timeout_sec=0.5)
            time.sleep(0.1)  # 100ms

        print("[DEBUG] Stop node loop")

    def __subscription_callback(self, msg, topic_type):
        if self.callback:
            self.callback(msg, topic_type)


    def __push_status(self, status):
        if self.status_callback:
            self.status_callback(status)