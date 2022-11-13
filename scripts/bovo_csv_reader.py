import rospy 
import csv
from pathlib import Path 
from geometry_msgs.msg import PoseStamped
def read_csv(filename):
    with open(filename) as csv_file:
        csv_reader = csv.DictReader(csv_file, delimiter=',')
        fields = ['.pose.pose.position.x', '.pose.pose.position.y']
        for row in csv_reader:
            data = [row[f] for f in fields]
            data = list(map(float, data))
            yield data 
      
def get_pose_msg(data:list):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = data[0]
    msg.pose.position.y = data[1]
    return msg 
    
if __name__ == "__main__":
    rospy.init_node("bovo_csv_reader", anonymous=True, disable_signals=True)
    pub_roomba20 = rospy.Publisher("roomba20/goal", PoseStamped, queue_size=10)    
    pub_roomba21 = rospy.Publisher("roomba21/goal", PoseStamped, queue_size=10)
    path = Path(".")
    sync_data = []
    for filename in path.glob('*.csv'):
        print(filename.name)
        data_gen = read_csv(filename=filename.name)
        sync_data.append(data_gen)


    dt = 0.01
    rate = rospy.Rate(1.0/dt)
    for i,(x, y) in enumerate(zip(*sync_data)):
        pub_roomba20.publish(get_pose_msg(x))
        pub_roomba21.publish(get_pose_msg(y))
        rospy.loginfo(f"publising goal t = {i * dt:.3f} s")
        rate.sleep()
    