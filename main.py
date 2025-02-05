from SupportClasses.DeviceManager import Stages, Waypoint


if __name__ == "__main__":
    # Choose whether to use the real stage or the simulator
    use_simulator = True

    waypoints = Waypoint('multi_layer_toolpath.csv')
    waypoint_list = waypoints.import_waypoints_from_csv()

    stage = Stages(waypoints, simulate=use_simulator)

    if waypoint_list:
        stage.move("Waypoint Following with PID Control", interpolation_type="linear", plot=True)
    else:
        print("No waypoints available.")
