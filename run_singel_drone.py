import argparse
from swarm.scripts.solutions.following.follow_marker import FollowMarkerWithCoords

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Follow Marker with Initial Position")
    parser.add_argument("-x", "--x_position", type=float, default=0, help="Initial X Position")
    parser.add_argument("-y", "--y_position", type=float, default=0, help="Initial Y Position")
    parser.add_argument("-z", "--z_position", type=float, default=0, help="Initial Z Position")
    args = parser.parse_args()

    initial_position = (args.x_position, args.y_position, args.z_position)
    marker_follower = FollowMarkerWithCoords(initial_position)
    marker_follower.main()
