INITIAL_POSE = {
    "x": -6.854203446770072,
    "y": 0.23443792543333908,
    "orientation": [0.0, 0.0, -0.6607944644312206, 0.7505669029320813],
    "covariance": [0.1814269173788574, 0.0012748385176071242, 0.0, 0.0, 0.0, 0.0, 0.0012748385176071797, 0.1602709329194385, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05485384185860012]
}

MOVE_BASE_PARAMS = {
    "/move_base/recovery_behavior_enabled": False,
    # Uncomment and customize as needed
    # "base_global_planner": "navfn/NavfnROS",
    # "base_local_planner": "dwa_local_planner/DWAPlannerROS",
    "global_costmap/robot_radius": 0,
    "global_costmap/inflation_radius": 0,
    "local_costmap/robot_radius": 0,
    "local_costmap/inflation_radius": 0,
    "/move_base/global_costmap/footprint_padding": 0.0,
    "/move_base/local_costmap/footprint_padding": 0.0,
    "/move_base/global_costmap/footprint": 0,
    "/move_base/local_costmap/footprint": 0,
}

CNN_DETECTION_CONFIDENCE_THRESHOLD = 0.8
CNN_DETECTION_URL = "http://tams195:5000/detect"
CNN_REQUEST_COOLDOWN = 1

PATROL_POSES = [
    {
        "x": -3.7633822320736634,
        "y": -4.365226005561541,
        "orientation": [0.0, 0.0, 0.021538153352768465, 0.9997680270693561],
    },
    {
        "x": -6.813098563205071,
        "y": -0.1964898388686835,
        "orientation": [0.0, 0.0, -0.6984114329886982, 0.715696493124476]
    },
    {
        "x": -8.461289554493236,
        "y": -8.9948642404941,
        "orientation": [0.0, 0.0, 0.45212570978433014, 0.8919542267134652]
    }
]

