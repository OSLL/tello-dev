from solutions.following.follow_marker import FollowMarkerWithCoords


if __name__ == "__main__":
    initial_position = (0, 0, 0)
    marker_follower = FollowMarkerWithCoords(initial_position)
    marker_follower.main()
