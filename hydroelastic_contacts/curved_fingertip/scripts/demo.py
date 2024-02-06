import rospy

import numpy as np
import matplotlib.pyplot as plt

from mujoco_ros_msgs.srv import SetMocapState, SetMocapStateRequest
from geometry_msgs.msg import PoseStamped
from tactile_msgs.msg import TactileState

# Taken from https://matplotlib.org/stable/users/explain/animations/blitting.html
class BlitManager:
    def __init__(self, canvas, animated_artists=()):
        """
        Parameters
        ----------
        canvas : FigureCanvasAgg
            The canvas to work with, this only works for subclasses of the Agg
            canvas which have the `~FigureCanvasAgg.copy_from_bbox` and
            `~FigureCanvasAgg.restore_region` methods.

        animated_artists : Iterable[Artist]
            List of the artists to manage
        """
        self.canvas = canvas
        self._bg = None
        self._artists = []

        for a in animated_artists:
            self.add_artist(a)
        # grab the background on every draw
        self.cid = canvas.mpl_connect("draw_event", self.on_draw)

    def on_draw(self, event):
        """Callback to register with 'draw_event'."""
        cv = self.canvas
        if event is not None:
            if event.canvas != cv:
                raise RuntimeError
        self._bg = cv.copy_from_bbox(cv.figure.bbox)
        self._draw_animated()

    def add_artist(self, art):
        """
        Add an artist to be managed.

        Parameters
        ----------
        art : Artist

            The artist to be added.  Will be set to 'animated' (just
            to be safe).  *art* must be in the figure associated with
            the canvas this class is managing.

        """
        if art.figure != self.canvas.figure:
            raise RuntimeError
        art.set_animated(True)
        self._artists.append(art)

    def _draw_animated(self):
        """Draw all of the animated artists."""
        fig = self.canvas.figure
        for a in self._artists:
            fig.draw_artist(a)

    def update(self):
        """Update the screen with animated artists."""
        cv = self.canvas
        fig = cv.figure
        # paranoia in case we missed the draw event,
        if self._bg is None:
            self.on_draw(None)
        else:
            # restore the background
            cv.restore_region(self._bg)
            # draw all of the animated artists
            self._draw_animated()
            # update the GUI state
            cv.blit(fig.bbox)
        # let the GUI event loop process anything it has to do
        cv.flush_events()

if __name__ == "__main__":
    rospy.init_node("measure", anonymous=True)
    rospy.wait_for_service("/mujoco_server/set_mocap_state", 2)

    mocap_client = rospy.ServiceProxy('/mujoco_server/set_mocap_state', SetMocapState)
    mocap_msg = SetMocapStateRequest()

    p = PoseStamped()
    p.pose.position.x = -0.02 
    p.pose.position.y = 0 
    p.pose.position.z = 0.07

    p.pose.orientation.w = 0.271
    p.pose.orientation.x = 0.653
    p.pose.orientation.y = 0.653
    p.pose.orientation.z = 0.271

    mocap_msg.mocap_state.name = ["mocap"]
    mocap_msg.mocap_state.pose = [p]

    baro = np.array(rospy.wait_for_message("/baro", TactileState).sensors[0].values)
    baro = np.ones_like(baro) * 2

    fig, ax = plt.subplots()
    ln = ax.bar(np.arange(len(baro)), baro.reshape(-1), animated=True)
    bm = BlitManager(fig.canvas, [*ln.patches])
    plt.show(block=False)
    plt.pause(0.1)

    # stort lower at a more adequate pose
    mocap_client(mocap_msg)

    while not rospy.is_shutdown():
        for step in np.linspace(0, np.pi, 1000):
            mocap_msg.mocap_state.pose[0].pose.position.x = -0.02 + np.sin(step) * 0.025
            mocap_msg.mocap_state.pose[0].pose.position.z = 0.07 - np.sin(step) * 0.012

            mocap_client(mocap_msg)
            baro = np.array(rospy.wait_for_message("/baro", TactileState).sensors[0].values)
            for bar, val in zip(ln, baro.reshape(-1)):
                bar.set_height(val)
            bm.update()


