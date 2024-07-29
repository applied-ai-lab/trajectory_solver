import rospy
from scipy.spatial.transform import Rotation as R

from trajectory_solver.EeTrajOpt import EndPointPose
from trajectory_solver_ros.msg import EndPointPoseMsg


class TrajSolverEndPointPub:
    def __init__(self, topic_name: str="", frame_id: str="") -> None:
        self._pub =  rospy.Publisher(topic_name, EndPointPoseMsg, queue_size=1)
        self._msg = EndPointPoseMsg()

        self._msg.header.stamp = rospy.Time.now()
        self._msg.frame_id = frame_id

    def publish(self, time_now: rospy.Time, pose: EndPointPose) -> None:
        self._msg.header.stamp = time_now

        # First Pose
        self._msg.init_pose.position.x = pose.init_pose.pos[0]
        self._msg.init_pose.position.y = pose.init_pose.pos[1]
        self._msg.init_pose.position.z = pose.init_pose.pos[2]

        init_quat = R.from_euler("xyz", pose.init_pose.pos[3:]).as_quat()

        self._msg.init_pose.orientation.x = init_quat[0]
        self._msg.init_pose.orientation.y = init_quat[1]
        self._msg.init_pose.orientation.z = init_quat[2]
        self._msg.init_pose.orientation.w = init_quat[3]

        # Mid Pose
        self._msg.mid_pose.position.x = pose.mid_pose.pos[0]
        self._msg.mid_pose.position.y = pose.mid_pose.pos[1]
        self._msg.mid_pose.position.z = pose.mid_pose.pos[2]

        mid_quat = R.from_euler("xyz", pose.mid_pose.pos[3:]).as_quat()

        self._msg.mid_pose.orientation.x = mid_quat[0]
        self._msg.mid_pose.orientation.y = mid_quat[1]
        self._msg.mid_pose.orientation.z = mid_quat[2]
        self._msg.mid_pose.orientation.w = mid_quat[3]

        # End Pose
        self._msg.end_pose.position.x = pose.end_pose.pos[0]
        self._msg.end_pose.position.y = pose.end_pose.pos[1]
        self._msg.end_pose.position.z = pose.end_pose.pos[2]

        end_quat = R.from_euler("xyz", pose.end_pose.pos[3:]).as_quat()

        self._msg.end_pose.orientation.x = end_quat[0]
        self._msg.end_pose.orientation.y = end_quat[1]
        self._msg.end_pose.orientation.z = end_quat[2]
        self._msg.end_pose.orientation.w = end_quat[3]

        # Publish msg
        self._pub.publish(self._msg)
        return 
        



    
        
