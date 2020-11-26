import numpy as np
import rospy
from grid import Grid
from std_msgs.msg import String
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

class GridVisualiser:
    """ Visualiser class for the grid. """

    def __init__(self, input_grid):
        self.grid = input_grid
        self.fig, self.ax = plt.subplots()
        self.fig.set_size_inches(6, 6)
        self.cmap = plt.get_cmap('coolwarm')

        plt.xlabel('gx')
        plt.ylabel('gy')
        # plt.gca().invert_xaxis()
        plt.title('Object of interest metric map (absolute)')

    def setup_frame(self):
        self.ax.set_xlim(0, self.grid.eff_size)
        self.ax.set_ylim(0, self.grid.eff_size)

    def update_grid(self, new_grid):
        rospy.loginfo("Type1:" + str(type(self.grid)))
        rospy.loginfo("Type2:" + str(type(self.grid.grid)))
        rospy.loginfo("TYpe3:" + str(type(new_grid)))
        self.grid.grid = new_grid

    def plot_grid(self, frame):
        grid_2d = np.reshape(self.grid.grid, (self.grid.eff_size, self.grid.eff_size))
        self.ax.pcolormesh(grid_2d, cmap=self.cmap, vmin=-1., vmax=2.)

def msg_to_array(msg):
    return np.fromstring(msg, dtype=int).ravel().tolist()

def get_grid_plot(msg):
    global grid_vis
    grid_vis.update_grid(msg_to_array(msg.data))
    rospy.loginfo("Updated grid")

if __name__ == '__main__':
    try:
        global grid_vis

        rospy.init_node('grid_plot_node', anonymous=True)
        rospy.loginfo("Waiting for first message")
        grid_msg = rospy.wait_for_message('/grid_plot', String, timeout=1500)
        rospy.loginfo("Waited for first message")
        grid_vis = GridVisualiser(Grid(map_arr=msg_to_array(grid_msg.data)))

        rospy.Subscriber('grid_plot', String, get_grid_plot)  
        animate = FuncAnimation(grid_vis.fig, grid_vis.plot_grid, init_func=grid_vis.setup_frame)
        rospy.loginfo("Showing graph")
        plt.show()
        rospy.loginfo("Showed graph")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('ROSInterruptException encountered at %s' % rospy.get_time())

