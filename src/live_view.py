#!/usr/bin/env python3

import os
import time
import cv2
import rospy
import threading
import queue
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Define the directory and file to monitor
IMAGE_DIR = "/home/reece/Downloads/CrSDK_v1.13.00_20241016a_Linux64PC/build"
IMAGE_FILE = "LiveView000000.JPG"
IMAGE_PATH = os.path.join(IMAGE_DIR, IMAGE_FILE)

class ImageMonitor(FileSystemEventHandler):
    """ Watches for file modifications and adds them to the processing queue """
    def __init__(self, image_queue):
        self.image_queue = image_queue

    def on_modified(self, event):
        """ Callback when the image file is updated """
        if event.src_path.endswith(IMAGE_FILE):
            rospy.loginfo("New image detected, adding to queue...")
            
            # Read the image immediately and store it in the queue
            img = cv2.imread(IMAGE_PATH, cv2.IMREAD_UNCHANGED)
            if img is not None:
                self.image_queue.put(img)  # Store actual image, not just timestamp
            else:
                rospy.logwarn("Failed to read image, skipping...")

class ImagePublisher:
    """ ROS node that watches for new images and publishes them to a topic """
    def __init__(self):
        rospy.init_node("image_publisher")

        self.pub = rospy.Publisher("/sony_live_view", Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=50)  # Large enough buffer to prevent loss

        self.event_handler = ImageMonitor(self.image_queue)
        self.observer = Observer()
        self.observer.schedule(self.event_handler, path=IMAGE_DIR, recursive=False)
        self.observer.start()

        # Start a separate thread for publishing images from the queue
        self.processing_thread = threading.Thread(target=self.process_queue)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        rospy.loginfo(f"Monitoring {IMAGE_FILE} for changes...")

    def process_queue(self):
        """ Continuously processes images from the queue and publishes them """
        while not rospy.is_shutdown():
            try:
                img = self.image_queue.get(timeout=1)  # Blocks until an image is available

                # Convert and publish
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                ros_image = self.bridge.cv2_to_imgmsg(img_rgb, encoding="rgb8")
                self.pub.publish(ros_image)
                rospy.loginfo("Published image from queue.")

                self.image_queue.task_done()  # Mark task as completed

            except queue.Empty:
                pass  # No new images, continue looping

    def shutdown(self):
        """ Clean shutdown handling """
        rospy.loginfo("Shutting down Image Publisher")
        self.observer.stop()
        self.observer.join()

def main():
    try:
        image_publisher = ImagePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        image_publisher.shutdown()

if __name__ == "__main__":
    main()
