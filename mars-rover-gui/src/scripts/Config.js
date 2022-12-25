const Config ={
    ROSBRIDGE_SERVER_IP: "172.17.0.1",
    ROSBRIDGE_SERVER_PORT: "9090",
    RECONNECTION_TIMER: 3000,

    CMD_VEL_TOPIC: "/turtle1/cmd_vel",
    ROBOT_COORDINATE_TOPIC : "/amcl_pose",
    ROBOT_VELOCITY_TOPIC : "/amcl_vel"
}

export default Config