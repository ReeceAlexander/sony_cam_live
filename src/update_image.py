#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import pexpect
import threading

class RemoteCliController:
    def __init__(self):
        rospy.init_node('update_image')

        # Start the RemoteCli using pexpect
        self.child = pexpect.spawn('/home/reece/Downloads/CrSDK_v1.13.00_20241016a_Linux64PC/build/RemoteCli', encoding='utf-8')
        self.child.logfile = open('/tmp/RemoteCli.log', 'w')  # Log to file
        self.child.timeout = None

        # Start a thread to read output
        self.output_thread = threading.Thread(target=self.read_output)
        self.output_thread.daemon = True
        self.output_thread.start()

        self.camera_connected = False
        self.photo_taken = False
        self.live_view_updated = False
        self.pause_live_view = False

        # Subscribe to the topic
        rospy.Subscriber('/camera_cntl', String, self.command_callback, queue_size=1)

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)

    def read_output(self):
        """ Continuously reads and logs output from RemoteCli. """
        while not rospy.is_shutdown():
            try:
                line = self.child.readline()
                if line:
                    if "Complete download." in line:
                        self.pause_live_view = False
                    
                    rospy.loginfo(line.strip())
            except Exception as e:
                rospy.logerr(f"Error reading output: {e}")
                break

    def send_sequence(self, sequence):
        """ Sends a sequence of commands to RemoteCli with a delay between each. """
        for cmd in sequence:
            rospy.loginfo(f"Sending command: {cmd}")
            self.child.sendline(cmd)
            # rospy.sleep(1)  # Adjust sleep as needed

    def command_callback(self, msg):
        """ Callback function triggered when a message is received. """
        command = msg.data
        rospy.loginfo(f"Received command: {command}")

        if command == "connect":
            self.send_sequence(["1", "1"])# Connect to the camera, and enter the <<REMOTE-MENU>>
            self.camera_connected = True # Signal that the camera is connected

        elif command == "capture":
            if self.live_view_updated: # If the current location is the << Other Menu >>
                self.pause_live_view = True # Disable live view to enable navigation
                self.send_sequence(["0", "1", "3", "y"])# Navigate to the << Shutter/Rec Operation Menu >> from the << Other Menu >>, via the <<REMOTE-MENU>>, and take a photo using "Shutter Half and Full Release in AF mode"
                self.live_view_updated = False # Signal that the current location is not the << Other Menu >>
                self.photo_taken = True # Signal that the current location is the << Shutter/Rec Operation Menu >>

            else: # If the current location is the << Shutter/Rec Operation Menu >>
                self.send_sequence(["1", "3", "y"])# Enter the << Shutter/Rec Operation Menu >> from the <<REMOTE-MENU>>, and take a photo using "Shutter Half and Full Release in AF mode"
                self.photo_taken = True # Signal that the current location is the << Shutter/Rec Operation Menu >>

        else:
            rospy.logwarn(f"Unknown command: {command}")

    def shutdown(self):
        """ Shutdown handler to clean up processes. """
        rospy.loginfo("Shutting down Remote CLI controller")
        if self.child:
            self.child.terminate()

    def run(self):
        """ Main loop running at 30 FPS when no callback is triggered. """
        
        rate = rospy.Rate(30)  # 30 Hz loop

        while not rospy.is_shutdown():
            if self.camera_connected and self.pause_live_view == False:
                rospy.loginfo("Running live feed at ~30 FPS")

                if self.photo_taken == True: # If the current location
                    self.send_sequence(["0", "7", "1"])# Navigate from the photo menu to the live view menu, and update the live view
                    self.photo_taken = False
                    self.live_view_updated = True

                elif self.live_view_updated == True: 
                    self.send_sequence(["1"])# From the live view menu, update the live view
                    self.live_view_updated = True

                elif self.live_view_updated == False and self.photo_taken == False:
                    self.send_sequence(["7", "1"])# Navigate from the main menu to the live menu, and update the live view
                    self.live_view_updated = True

            else:
                rospy.loginfo("No live feed running - camera not connected.")

            rate.sleep()

def main():
    try:
        cli_controller = RemoteCliController()
        cli_controller.run()  # Start the main loop
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
